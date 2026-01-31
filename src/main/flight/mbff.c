/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_MBFF

#include <math.h>
#include <string.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/config.h"

#include "drivers/dshot.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"

#include "mbff.h"

// ---------------------------------------------------------------------------
// PG Registration
// ---------------------------------------------------------------------------

PG_REGISTER_WITH_RESET_TEMPLATE(mbffConfig_t, mbffConfig, PG_MBFF_CONFIG, 0);

PG_RESET_TEMPLATE(mbffConfig_t, mbffConfig,
    .enabled = 0,                   // Disabled by default
    .ts = 30,                       // 30ms trajectory time constant (was 15, increased to reduce overshoot)
    .tp = 50,                       // 50ms preview horizon
    .ka = 100,                      // 1.0 trajectory accel gain
    .kr = 25,                       // 0.25 preview correction gain
    .b0 = 100,                      // 1.0 base effectiveness
    .b1 = 50,                       // 0.5 RPM effectiveness coefficient
    .ff_limit = 50,                 // 50% of pidSumLimit
    .gain = 100,                    // 10.0 master output gain (was 50.0, reduced to prevent saturation)
    .preview_threshold = 50,        // 50 deg/s² - preview fades in above this setpoint rate of change
);

// ---------------------------------------------------------------------------
// Runtime State
// ---------------------------------------------------------------------------

FAST_DATA_ZERO_INIT mbffRuntime_t mbffRuntime;

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------

void mbffInit(void)
{
    const mbffConfig_t *config = mbffConfig();

    // Clear runtime state
    memset(&mbffRuntime, 0, sizeof(mbffRuntime));

    // Cache enabled state
    mbffRuntime.enabled = config->enabled != 0;

    if (!mbffRuntime.enabled) {
        return;
    }

    // Convert config values to float for runtime use
    mbffRuntime.ts_sec = config->ts * 0.001f;           // ms to seconds
    mbffRuntime.tp_sec = config->tp * 0.001f;           // ms to seconds
    mbffRuntime.ka = config->ka * 0.01f;                // x100 to float
    mbffRuntime.kr = config->kr * 0.01f;                // x100 to float
    mbffRuntime.b0 = config->b0 * 0.01f;                // x100 to float
    mbffRuntime.b1 = config->b1 * 0.01f;                // x100 to float
    mbffRuntime.gain = config->gain * 0.1f;             // x10 to float (10-2000 -> 1.0-200.0)
    mbffRuntime.preview_threshold = (float)config->preview_threshold;  // deg/s threshold

    // Compute FF limit from pidSumLimit
    mbffRuntime.ff_limit = (float)PIDSUM_LIMIT * config->ff_limit * 0.01f;

    // Get PID timing info
    mbffRuntime.dT = targetPidLooptime * 1e-6f;
    mbffRuntime.pidFrequency = 1.0f / mbffRuntime.dT;

    // Initialize effectiveness to base value
    mbffRuntime.effectiveness = mbffRuntime.b0;
    mbffRuntime.avg_rpm_sq = 0.0f;
}

void mbffReset(void)
{
    if (!mbffRuntime.enabled) {
        return;
    }

    // Reset reference trajectory to zero
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mbffRuntime.axis[axis].omega_ref = 0.0f;
        mbffRuntime.axis[axis].prev_omega_ref = 0.0f;
    }

    mbffRuntime.avg_rpm_sq = 0.0f;
    mbffRuntime.effectiveness = mbffRuntime.b0;
}

void mbffUpdateRpm(void)
{
    if (!mbffRuntime.enabled) {
        return;
    }

#ifdef USE_DSHOT_TELEMETRY
    // Compute average RPM² from motor telemetry
    float rpmSqSum = 0.0f;
    int validMotors = 0;
    const int motorCount = getMotorCount();

    for (int i = 0; i < motorCount; i++) {
        if (isDshotMotorTelemetryActive(i)) {
            float rpm = getDshotRpm(i);
            rpmSqSum += rpm * rpm;
            validMotors++;
        }
    }

    if (validMotors > 0) {
        mbffRuntime.avg_rpm_sq = rpmSqSum / validMotors;
    } else {
        // Fallback: estimate from throttle if no telemetry
        // This is a rough approximation
        mbffRuntime.avg_rpm_sq = 0.0f;
    }

    // Update torque effectiveness model: g(RPM²) = b0 + b1 * (RPM² / 1e6)
    mbffRuntime.effectiveness = mbffRuntime.b0 + mbffRuntime.b1 * (mbffRuntime.avg_rpm_sq / 1e6f);

    // Ensure effectiveness doesn't go too low (prevent division issues)
    mbffRuntime.effectiveness = MAX(mbffRuntime.effectiveness, 0.1f);

#else
    // Without DShot telemetry, use base effectiveness only
    mbffRuntime.effectiveness = mbffRuntime.b0;
#endif
}

FAST_CODE float mbffUpdate(int axis, float setpoint, float gyroRate, float dT)
{
    if (!mbffRuntime.enabled) {
        return 0.0f;
    }

    mbffAxisState_t *state = &mbffRuntime.axis[axis];

    // Store previous reference for debug
    state->prev_omega_ref = state->omega_ref;

    // -------------------------------------------------------------------------
    // Step 1: Update reference trajectory
    // ω_ref[k+1] = ω_ref[k] + (dt / T_s) * (ω_sp - ω_ref[k])
    // -------------------------------------------------------------------------
    const float ts_sec = mbffRuntime.ts_sec;
    const float alpha_traj_raw = setpoint - state->omega_ref;

    // First-order trajectory update
    if (ts_sec > 0.0f) {
        state->omega_ref += (dT / ts_sec) * alpha_traj_raw;
    } else {
        state->omega_ref = setpoint;
    }

    // -------------------------------------------------------------------------
    // Step 2: Compute desired acceleration
    // α_traj = (ω_sp - ω_ref) / T_s
    // α_prev = (ω_ref - ω_meas) / T_p
    // α_des = k_a * α_traj + k_r * α_prev
    // -------------------------------------------------------------------------
    const float tp_sec = mbffRuntime.tp_sec;

    // Trajectory-following acceleration (how fast we want to reach setpoint)
    float alpha_traj = 0.0f;
    if (ts_sec > 0.0f) {
        alpha_traj = (setpoint - state->omega_ref) / ts_sec;
    }

    // Preview/tracking correction (how much we're lagging the reference)
    // Gated by setpoint rate of change to avoid amplifying noise during steady-state
    float alpha_prev = 0.0f;
    if (tp_sec > 0.0f) {
        alpha_prev = (state->omega_ref - gyroRate) / tp_sec;
        
        // Apply preview threshold gating based on |d(setpoint)/dt|
        // When stick is stationary (hover or sustained roll), preview is off
        // When stick is moving (transitions), preview helps tracking
        const float threshold = mbffRuntime.preview_threshold;
        if (threshold > 0.0f && dT > 0.0f) {
            // Compute setpoint derivative (deg/s per second = deg/s²)
            const float setpointDerivative = (setpoint - state->prev_setpoint) / dT;
            const float absDerivative = fabsf(setpointDerivative);
            
            // Threshold is in deg/s, derivative is in deg/s²
            // Scale threshold to deg/s² using a reasonable stick movement rate
            // e.g., threshold=50 means ~50 deg/s change per ~15ms = ~3333 deg/s²
            // Use threshold * 100 as the derivative threshold (deg/s² units)
            const float derivThreshold = threshold * 100.0f;
            
            // Linear ramp for responsiveness during transitions
            const float ratio = absDerivative / derivThreshold;
            const float preview_scale = (ratio >= 1.0f) ? 1.0f : ratio;
            alpha_prev *= preview_scale;
        }
    }
    
    // Store setpoint for next iteration's derivative calculation
    state->prev_setpoint = setpoint;

    // Combined desired acceleration
    const float alpha_des = mbffRuntime.ka * alpha_traj + mbffRuntime.kr * alpha_prev;

    // -------------------------------------------------------------------------
    // Step 3: Convert to feedforward command
    // u_FF = gain * α_des / g(RPM²)
    // -------------------------------------------------------------------------
    float ff_output = mbffRuntime.gain * alpha_des / mbffRuntime.effectiveness;

    // Apply FF limit
    ff_output = constrainf(ff_output, -mbffRuntime.ff_limit, mbffRuntime.ff_limit);

    // -------------------------------------------------------------------------
    // Debug output (no multipliers to avoid overflow)
    // -------------------------------------------------------------------------
    if (axis == gyro.gyroDebugAxis && debugMode == DEBUG_MBFF) {
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_OMEGA_REF, lrintf(state->omega_ref));
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_ALPHA_DES, lrintf(alpha_des));           // raw acceleration
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_EFFECTIVENESS, lrintf(mbffRuntime.effectiveness)); // raw effectiveness
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_FF_OUTPUT, lrintf(ff_output));           // raw FF output
        // MBFF_DEBUG_CLASSIC_FF is set in pid.c for comparison
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_RPM_SQ, lrintf(mbffRuntime.avg_rpm_sq / 1000000.0f)); // RPM² / 1M
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_TRAJ_ERROR, lrintf(setpoint - state->omega_ref));
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_TRACK_ERROR, lrintf(state->omega_ref - gyroRate));
    }

    return ff_output;
}

float mbffGetReferenceRate(int axis)
{
    if (!mbffRuntime.enabled || axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return 0.0f;
    }
    return mbffRuntime.axis[axis].omega_ref;
}

float mbffGetEffectiveness(void)
{
    return mbffRuntime.effectiveness;
}

bool mbffIsEnabled(void)
{
    return mbffRuntime.enabled;
}

#endif // USE_MBFF
