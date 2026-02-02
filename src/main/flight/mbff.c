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

/*
 * MBFF v2 - Model-Based Feedforward using eRPM² physics model
 *
 * Core relationship (validated R² = 0.89-0.99 from blackbox analysis):
 *   delta(gyro) = E_rpm² × eRPM²_diff
 *
 * Where:
 *   delta(gyro) = sample-to-sample gyro change (NOT time-scaled derivative!)
 *   eRPM²_diff  = differential of squared motor speeds for axis torque
 *   E_rpm²      = effectiveness coefficient (~0.003-0.006)
 *
 * Gating conditions for learning (to achieve high R²):
 *   1. |setpoint| > 400 deg/s (active maneuvering)
 *   2. |gyro_product| < 1e6 (not tumbling)
 *   3. |delta(gyro)| > 15 (sample-to-sample, significant acceleration)
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_MBFF

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/dshot.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/mbff.h"

#include "sensors/gyro.h"

// ---------------------------------------------------------------------------
// PG Configuration
// ---------------------------------------------------------------------------

PG_REGISTER_WITH_RESET_TEMPLATE(mbffConfig_t, mbffConfig, PG_MBFF_CONFIG, 2);

PG_RESET_TEMPLATE(mbffConfig_t, mbffConfig,
    .enabled = 0,
    .e_rpm2_roll = 60,          // 0.006 (deg/s² per eRPM²_diff, validated from blackbox)
    .e_rpm2_pitch = 30,         // 0.003 (deg/s² per eRPM²_diff, validated from blackbox)
    .e_rpm2_yaw = 30,           // 0.003 (estimated)
    .ff_scale = 10,             // 0.01 (CLI/1000, so 10 = 0.010)
    .ff_limit = 50,             // 50% of pidSumLimit
    .learn_enable = 1,          // Enabled by default
    .learn_lambda = 995,        // 0.995 forgetting factor
    .setpoint_thresh = 400,     // 400 deg/s
    .gyro_delta_thresh = 4,     // 4 deg/s per PID loop
);

// ---------------------------------------------------------------------------
// Runtime State
// ---------------------------------------------------------------------------

mbffRuntime_t mbffRuntime;

// ---------------------------------------------------------------------------
// Internal functions
// ---------------------------------------------------------------------------

/**
 * Compute eRPM² differential for an axis.
 * This represents the torque imbalance that causes rotation.
 *
 * Motor layout (Betaflight Quad X, props-out, viewed from above):
 *       Front
 *      3     1       M1 = front-right (CCW)
 *        \ /         M3 = front-left (CW)
 *         X
 *        / \         M0 = rear-right (CW)
 *      2     0       M2 = rear-left (CCW)
 *       Rear
 *
 * Sign convention: positive eRPM²_diff → positive angular acceleration
 *   - Roll right: LEFT motors spin faster → (left²) - (right²) > 0
 *   - Pitch up (nose up): REAR motors spin faster → (rear²) - (front²) > 0
 *   - Yaw right (CW): CCW motors spin faster → (CCW²) - (CW²) > 0
 */
static float computeERPMSquaredDiff(int axis)
{
    const float *eRPM = mbffRuntime.eRPM;
    
    // Compute squared values
    const float e0sq = sq(eRPM[0]);
    const float e1sq = sq(eRPM[1]);
    const float e2sq = sq(eRPM[2]);
    const float e3sq = sq(eRPM[3]);
    
    switch (axis) {
        case FD_ROLL:
            // Left motors (M2+M3) vs Right motors (M0+M1)
            // More thrust on left → roll right → positive gyro
            return (e2sq + e3sq) - (e0sq + e1sq);
            
        case FD_PITCH:
            // Rear motors (M0+M2) vs Front motors (M1+M3)
            // More thrust on rear → pitch up (nose up) → positive gyro  
            return (e0sq + e2sq) - (e1sq + e3sq);
            
        case FD_YAW:
            // CCW motors (M1+M2) vs CW motors (M0+M3)
            // CCW motors produce CW reaction torque → yaw right → positive gyro
            return (e1sq + e2sq) - (e0sq + e3sq);
            
        default:
            return 0.0f;
    }
}

/**
 * Check gating conditions for online learning.
 * Returns true if sample should be used for learning.
 */
static bool shouldLearn(int axis, float setpoint, float gyroDelta, bool saturated)
{
    UNUSED(axis);  // axis is used for debug output at call site
    
    uint8_t gate_reason = MBFF_GATE_NONE;
    
    // 1. Check learning enabled
    if (!mbffRuntime.learn_enabled) {
        gate_reason |= MBFF_GATE_DISABLED;
    }
    
    // 2. Check valid RPM telemetry (at least some motors spinning)
    bool hasValidRpm = false;
    for (int i = 0; i < 4; i++) {
        if (mbffRuntime.eRPM[i] > 1000.0f) {
            hasValidRpm = true;
            break;
        }
    }
    if (!hasValidRpm) {
        gate_reason |= MBFF_GATE_NO_RPM;
    }
    
    // 3. Check setpoint threshold (active maneuvering)
    if (fabsf(setpoint) < mbffRuntime.setpoint_thresh) {
        gate_reason |= MBFF_GATE_LOW_SETPOINT;
    }
    
    // 4. Check gyro delta threshold (per PID loop, NOT time-scaled!)
    // TODO: This threshold assumes 4kHz PID loop. For robustness, consider
    // scaling by pidFrequency or converting to acceleration (deg/s²).
    if (fabsf(gyroDelta) < mbffRuntime.gyro_delta_thresh) {
        gate_reason |= MBFF_GATE_LOW_GYRO_DELTA;
    }
    
    // 5. Check for tumble (gyro product threshold)
    const float gyroProduct = gyro.gyroADCf[0] * gyro.gyroADCf[1] * gyro.gyroADCf[2];
    if (fabsf(gyroProduct) > 1e6f) {
        gate_reason |= MBFF_GATE_TUMBLE;
    }
    
    // 6. Check saturation
    if (saturated) {
        gate_reason |= MBFF_GATE_SATURATED;
    }
    
    mbffRuntime.gated = (gate_reason != MBFF_GATE_NONE);
    mbffRuntime.gate_reason = gate_reason;
    
    return (gate_reason == MBFF_GATE_NONE);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void mbffInit(void)
{
    memset(&mbffRuntime, 0, sizeof(mbffRuntime_t));
    
    const mbffConfig_t *config = mbffConfig();
    
    mbffRuntime.enabled = config->enabled;
    mbffRuntime.learn_enabled = config->learn_enable;
    
    if (!mbffRuntime.enabled) {
        return;
    }
    
    // Convert config to runtime floats
    mbffRuntime.E_rpm2[FD_ROLL] = config->e_rpm2_roll / 10000.0f;
    mbffRuntime.E_rpm2[FD_PITCH] = config->e_rpm2_pitch / 10000.0f;
    mbffRuntime.E_rpm2[FD_YAW] = config->e_rpm2_yaw / 10000.0f;
    
    mbffRuntime.ff_scale = config->ff_scale / 1000.0f;  // CLI 1-100 -> 0.001-0.100
    mbffRuntime.ff_limit = config->ff_limit / 100.0f;
    mbffRuntime.lambda = config->learn_lambda / 1000.0f;
    mbffRuntime.setpoint_thresh = (float)config->setpoint_thresh;
    mbffRuntime.gyro_delta_thresh = (float)config->gyro_delta_thresh;
    
    // Get PID loop frequency for time-scaling calculations
    mbffRuntime.pidFrequency = pidGetPidFrequency();
    mbffRuntime.dT = 1.0f / mbffRuntime.pidFrequency;
    
    // Use default pidSumLimit constant
    mbffRuntime.pidSumLimit = (float)PIDSUM_LIMIT;
    
    // Initialize per-axis state
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mbffAxisState_t *state = &mbffRuntime.axis[axis];
        
        state->prev_gyro = 0.0f;
        state->prev_setpoint = 0.0f;
        
        // Initialize RLS learner with config values
        state->E = mbffRuntime.E_rpm2[axis];
        state->P = MBFF_RLS_P_INIT;
        state->learn_count = 0;
        state->confident = false;
    }
    
    // Initialize eRPM values
    for (int i = 0; i < 4; i++) {
        mbffRuntime.eRPM[i] = 0.0f;
    }
}

void mbffReset(void)
{
    if (!mbffRuntime.enabled) {
        return;
    }
    
    // Only reset eRPM (stale values from before arm)
    // Do NOT reset prev_gyro - that would break gyro delta calculation
    for (int i = 0; i < 4; i++) {
        mbffRuntime.eRPM[i] = 0.0f;
    }
    
    mbffRuntime.gated = true;
    mbffRuntime.gate_reason = MBFF_GATE_NONE;
}

void mbffUpdateRpm(void)
{
    if (!mbffRuntime.enabled) {
        return;
    }
    
#ifdef USE_DSHOT_TELEMETRY
    const int motorCount = getMotorCount();
    
    for (int i = 0; i < MIN(motorCount, 4); i++) {
        if (isDshotMotorTelemetryActive(i)) {
            // Use eRPM/100 to match blackbox logging scale
            // This makes E values directly comparable to Python validation
            mbffRuntime.eRPM[i] = (float)getDshotErpm(i);
        } else {
            mbffRuntime.eRPM[i] = 0.0f;
        }
    }
    
    // Compute eRPM² differentials for each axis
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mbffRuntime.eRPM_sq_diff[axis] = computeERPMSquaredDiff(axis);
    }
#else
    // Without DSHOT telemetry, MBFF cannot function
    for (int i = 0; i < 4; i++) {
        mbffRuntime.eRPM[i] = 0.0f;
    }
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mbffRuntime.eRPM_sq_diff[axis] = 0.0f;
    }
#endif
}

FAST_CODE float mbffUpdate(int axis, float setpoint, float setpointDelta, float gyroRate, float dT)
{
    if (!mbffRuntime.enabled) {
        return 0.0f;
    }
    
    mbffAxisState_t *state = &mbffRuntime.axis[axis];
    
    // Store dT for other functions
    mbffRuntime.dT = dT;
    
    // Get current effectiveness (learned or configured)
    float E = state->confident ? state->E : mbffRuntime.E_rpm2[axis];
    
    // Get eRPM² differential for this axis
    float eRPM_sq_diff = mbffRuntime.eRPM_sq_diff[axis];
    
    // -------------------------------------------------------------------------
    // Core physics: predict angular acceleration from eRPM² differential
    // delta(gyro) = E × eRPM²_diff
    // -------------------------------------------------------------------------
    
    // The eRPM² differential directly predicts gyro change
    // We want to produce a command that creates the desired setpoint change
    // For FF: we want to anticipate the gyro response to setpoint delta
    
    // Simple approach: scale setpoint delta by E and eRPM sensitivity
    // When eRPM²_diff is large, less command is needed (high thrust = high response)
    // When eRPM²_diff is small, more command is needed (low thrust = low response)
    
    // For now, use a direct scaling approach:
    // FF = ff_scale × setpointDelta
    // This is similar to classic FF but will be refined based on learned E
    
    float ff_output = mbffRuntime.ff_scale * setpointDelta;
    
    // TODO: Once E is learned reliably, use model-based prediction:
    // ff_output = setpointDelta / (E × sensitivity_factor);
    
    // Apply limit as percentage of pidSumLimit
    const float limit = mbffRuntime.ff_limit * mbffRuntime.pidSumLimit;
    ff_output = constrainf(ff_output, -limit, limit);
    
    // -------------------------------------------------------------------------
    // Debug output
    // -------------------------------------------------------------------------
    if (axis == gyro.gyroDebugAxis && debugMode == DEBUG_MBFF) {
        const float gyroDelta = gyroRate - state->prev_gyro;
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_ERPM_SQ_DIFF, lrintf(eRPM_sq_diff / 1e6f));
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_DELTA_GYRO, lrintf(gyroDelta));
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_E_RPM2, lrintf(E * 10000.0f));
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_FF_OUTPUT, lrintf(ff_output));
        // MBFF_DEBUG_CLASSIC_FF is set in pid.c
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_SETPOINT, lrintf(setpoint));
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_GYRO_RATE, lrintf(gyroRate));
        DEBUG_SET(DEBUG_MBFF, MBFF_DEBUG_GATE_REASON, mbffRuntime.gate_reason);
    }
    
    // NOTE: prev_gyro is updated in mbffLearnUpdate() which runs after this
    // Don't update it here or the learning gyroDelta will be wrong
    
    return ff_output;
}

void mbffLearnUpdate(int axis, float gyroRate, float setpoint, bool saturated)
{
    if (!mbffRuntime.enabled || !mbffRuntime.learn_enabled) {
        return;
    }
    
    // Only learn roll and pitch (yaw dynamics may differ)
    if (axis > FD_PITCH) {
        return;
    }
    
    mbffAxisState_t *state = &mbffRuntime.axis[axis];
    
    // Compute sample-to-sample gyro delta (NOT time-scaled!)
    const float gyroDelta = gyroRate - state->prev_gyro;
    
    // Check gating conditions FIRST (so gate_reason is valid for debug output)
    const bool canLearn = shouldLearn(axis, setpoint, gyroDelta, saturated);
    
    // Always output debug values (even when gated) so we can monitor E values
    if (axis == gyro.gyroDebugAxis && debugMode == DEBUG_MBFF_LEARN) {
        DEBUG_SET(DEBUG_MBFF_LEARN, MBFF_LEARN_DEBUG_E_ROLL, 
                  lrintf(mbffRuntime.axis[FD_ROLL].E * 10000.0f));
        DEBUG_SET(DEBUG_MBFF_LEARN, MBFF_LEARN_DEBUG_E_PITCH, 
                  lrintf(mbffRuntime.axis[FD_PITCH].E * 10000.0f));
        DEBUG_SET(DEBUG_MBFF_LEARN, MBFF_LEARN_DEBUG_GYRO_DELTA, 
                  lrintf(gyroDelta));
        DEBUG_SET(DEBUG_MBFF_LEARN, MBFF_LEARN_DEBUG_SETPOINT, 
                  lrintf(setpoint));
        DEBUG_SET(DEBUG_MBFF_LEARN, MBFF_LEARN_DEBUG_SAMPLES_ROLL, 
                  mbffRuntime.axis[FD_ROLL].learn_count);
        DEBUG_SET(DEBUG_MBFF_LEARN, MBFF_LEARN_DEBUG_SAMPLES_PITCH, 
                  mbffRuntime.axis[FD_PITCH].learn_count);
        DEBUG_SET(DEBUG_MBFF_LEARN, MBFF_LEARN_DEBUG_ERPM_SQ_DIFF, 
                  lrintf(mbffRuntime.eRPM_sq_diff[axis] / 1e6f));
        DEBUG_SET(DEBUG_MBFF_LEARN, MBFF_LEARN_DEBUG_GATE_REASON, mbffRuntime.gate_reason);
    }
    
    // Store prev values for next iteration (do this before early return)
    state->prev_gyro = gyroRate;
    state->prev_setpoint = setpoint;
    
    // Exit if gated
    if (!canLearn) {
        return;
    }
    
    // Get eRPM² differential for this axis
    const float x = mbffRuntime.eRPM_sq_diff[axis];
    
    // Observation: y = angular acceleration (deg/s²)
    // The per-PID-loop gyroDelta (deg/s) is converted to acceleration by multiplying
    // by pidFrequency. This makes E values match the Python-validated physics model:
    //   d(gyro)/dt = E × eRPM²_diff   where d(gyro)/dt is in deg/s²
    // Without this scaling, E would be ~4000x smaller than expected.
    const float y = gyroDelta * mbffRuntime.pidFrequency;
    
    // -------------------------------------------------------------------------
    // RLS Update (zero-intercept: y = E × x)
    //
    // Standard RLS with forgetting:
    //   K = P × x / (lambda + x × P × x)
    //   E = E + K × (y - E × x)
    //   P = (P - K × x × P) / lambda
    // -------------------------------------------------------------------------
    
    const float lambda = mbffRuntime.lambda;
    const float P = state->P;
    const float E = state->E;
    
    // Skip if x is too small (would cause numerical issues)
    if (fabsf(x) < 1e-6f) {
        return;
    }
    
    // Compute Kalman gain
    const float xP = x * P;
    const float denom = lambda + x * xP;
    if (fabsf(denom) < 1e-10f) {
        return;  // Prevent division by zero
    }
    const float K = xP / denom;
    
    // Compute prediction error
    const float prediction = E * x;
    const float error = y - prediction;
    
    // Update estimate
    float E_new = E + K * error;
    
    // Sanity clamp (E should be positive and reasonable)
    E_new = constrainf(E_new, 0.0001f, 0.1f);
    
    // Update covariance
    float P_new = (P - K * xP) / lambda;
    
    // Prevent covariance from going too small (lose adaptability)
    // or too large (numerical issues)
    P_new = constrainf(P_new, 0.1f, 10000.0f);
    
    // Store updated values
    state->E = E_new;
    state->P = P_new;
    state->learn_count++;
    
    // Check confidence
    if (state->learn_count >= MBFF_LEARN_MIN_SAMPLES) {
        state->confident = true;
    }
    // Debug output is at start of function (runs unconditionally)
}

void mbffLearnReset(void)
{
    // Reset prev_gyro but keep learned E values (they persist across flights)
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mbffRuntime.axis[axis].prev_gyro = 0.0f;
    }
}

float mbffGetLearnedE(int axis)
{
    if (axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return 0.0f;
    }
    return mbffRuntime.axis[axis].E;
}

bool mbffIsEnabled(void)
{
    return mbffRuntime.enabled;
}

bool mbffLearnIsGated(void)
{
    return mbffRuntime.gated;
}

bool mbffLearnIsConfident(int axis)
{
    if (axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return false;
    }
    return mbffRuntime.axis[axis].confident;
}

float mbffGetERPMSquaredDiff(int axis)
{
    if (axis < 0 || axis >= XYZ_AXIS_COUNT) {
        return 0.0f;
    }
    return mbffRuntime.eRPM_sq_diff[axis];
}

#endif // USE_MBFF
