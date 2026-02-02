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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "pg/pg.h"

/*
 * Model-Based Feedforward (MBFF) v2
 *
 * Physics-based feedforward using eRPM² differential model:
 *   delta(gyro) ~ E_rpm² × eRPM²_diff
 *
 * Key relationships (validated R² = 0.89-0.99):
 *   Roll:  eRPM²_diff = (eRPM[0]² + eRPM[1]²) - (eRPM[2]² + eRPM[3]²)
 *   Pitch: eRPM²_diff = (eRPM[0]² + eRPM[2]²) - (eRPM[1]² + eRPM[3]²)
 *   Yaw:   eRPM²_diff = (eRPM[0]² + eRPM[3]²) - (eRPM[1]² + eRPM[2]²)
 *
 * See docs/mbff/PRD.md for detailed design rationale.
 */

// Feature flag - define in target.h to enable MBFF
// #define USE_MBFF

#ifdef USE_MBFF

// ---------------------------------------------------------------------------
// Configuration structure (stored in PG system)
// ---------------------------------------------------------------------------

typedef struct mbffConfig_s {
    uint8_t  enabled;               // 0 = off, 1 = on
    
    // eRPM² effectiveness per axis [scaled: value/10000]
    // These represent angular acceleration per unit eRPM²_diff: deg/s² per eRPM²_diff
    // E ≈ 0.006 for roll, 0.003 for pitch (validated via blackbox analysis)
    uint16_t e_rpm2_roll;           // Roll effectiveness [x10000], range 10-200 (0.001-0.02)
    uint16_t e_rpm2_pitch;          // Pitch effectiveness [x10000], range 10-200 (0.001-0.02)
    uint16_t e_rpm2_yaw;            // Yaw effectiveness [x10000], range 10-200 (0.001-0.02)
    
    // Output scaling
    uint8_t  ff_scale;              // FF output scale [x100], range 1-100 (0.01-1.0)
    uint8_t  ff_limit;              // FF limit as % of pidSumLimit, 10-100
    
    // Online learning config (RLS-based)
    uint8_t  learn_enable;          // 0 = off, 1 = on (learning of E_rpm²)
    uint16_t learn_lambda;          // RLS forgetting factor [x1000], 990-999 (0.99-0.999)
    uint16_t setpoint_thresh;       // Setpoint threshold for learning [deg/s], 100-800
    // TODO: gyro_delta_thresh assumes 4kHz PID. Should scale with pidFrequency.
    uint8_t  gyro_delta_thresh;     // Per-PID-loop gyro delta threshold, 1-50
} mbffConfig_t;

PG_DECLARE(mbffConfig_t, mbffConfig);

// ---------------------------------------------------------------------------
// Runtime state (per axis)
// ---------------------------------------------------------------------------

typedef struct mbffAxisState_s {
    float prev_gyro;            // Previous gyro rate for delta(gyro) calculation
    float prev_setpoint;        // Previous setpoint for delta calculation
    
    // RLS learner state (zero-intercept: y = E × x)
    float E;                    // Learned eRPM² effectiveness
    float P;                    // RLS covariance (scalar for 1-parameter model)
    uint32_t learn_count;       // Number of samples used for learning
    bool confident;             // Has enough samples to use learned value
} mbffAxisState_t;

// ---------------------------------------------------------------------------
// Online learner gating
// ---------------------------------------------------------------------------

// Gate reason flags
#define MBFF_GATE_NONE              0
#define MBFF_GATE_DISABLED          (1 << 0)
#define MBFF_GATE_NO_RPM            (1 << 1)
#define MBFF_GATE_LOW_SETPOINT      (1 << 2)
#define MBFF_GATE_LOW_GYRO_DELTA    (1 << 3)
#define MBFF_GATE_TUMBLE            (1 << 4)
#define MBFF_GATE_SATURATED         (1 << 5)

// Minimum samples before using learned E value
#define MBFF_LEARN_MIN_SAMPLES      20

// Initial RLS covariance (large = fast initial learning)
#define MBFF_RLS_P_INIT             1000.0f

typedef struct mbffRuntime_s {
    // Per-axis state
    mbffAxisState_t axis[XYZ_AXIS_COUNT];

    // Cached config values (converted to float at init)
    float E_rpm2[XYZ_AXIS_COUNT];   // eRPM² effectiveness per axis
    float ff_scale;             // FF output scale
    float ff_limit;             // Max FF output
    float lambda;               // RLS forgetting factor
    float setpoint_thresh;      // Setpoint threshold for learning
    float gyro_delta_thresh;    // Gyro delta threshold for learning
    
    // Runtime data
    float eRPM[4];              // Current motor eRPM values
    float eRPM_sq_diff[XYZ_AXIS_COUNT];  // Current eRPM² differential per axis
    float dT;                   // Loop time [s]
    float pidFrequency;         // PID loop frequency [Hz]
    float pidSumLimit;          // PID sum limit for FF limiting
    
    // Gating state
    bool gated;                 // True if learning is currently gated
    uint8_t gate_reason;        // Reason for gating (for debug)

    bool enabled;               // Runtime enable flag
    bool learn_enabled;         // Learning enable flag
} mbffRuntime_t;

extern mbffRuntime_t mbffRuntime;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

/**
 * Initialize MBFF runtime from config.
 * Call once at startup and when config changes.
 */
void mbffInit(void);

/**
 * Reset MBFF state (e.g., on arm/disarm).
 * Resets learning state but keeps learned E values.
 */
void mbffReset(void);

/**
 * Compute feedforward for a single axis.
 *
 * @param axis          Axis index (FD_ROLL, FD_PITCH, FD_YAW)
 * @param setpoint      Rate setpoint from RC [deg/s]
 * @param setpointDelta Delta of setpoint from last loop (for acceleration)
 * @param gyroRate      Measured gyro rate [deg/s]
 * @param dT            Loop time [s]
 * @return              Feedforward command to add to PID output
 */
float mbffUpdate(int axis, float setpoint, float setpointDelta, float gyroRate, float dT);

/**
 * Update eRPM data from motor telemetry.
 * Call once per PID loop before axis updates.
 */
void mbffUpdateRpm(void);

/**
 * Check if MBFF is enabled.
 */
bool mbffIsEnabled(void);

/**
 * Update the online RLS learner for an axis.
 * Call once per axis during active maneuvers.
 *
 * @param axis          Axis index (FD_ROLL, FD_PITCH, FD_YAW)
 * @param gyroRate      Measured gyro rate [deg/s]
 * @param setpoint      Current setpoint [deg/s]
 * @param saturated     True if axis command is saturated
 */
void mbffLearnUpdate(int axis, float gyroRate, float setpoint, bool saturated);

/**
 * Reset the learner state (e.g., on disarm).
 * Keeps learned E values but resets prev_gyro.
 */
void mbffLearnReset(void);

/**
 * Get learned E value for an axis.
 */
float mbffGetLearnedE(int axis);

/**
 * Check if learner is currently gated (not updating).
 */
bool mbffLearnIsGated(void);

/**
 * Check if learner has enough confidence for an axis.
 */
bool mbffLearnIsConfident(int axis);

/**
 * Get the eRPM² differential for an axis (for debug).
 */
float mbffGetERPMSquaredDiff(int axis);

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------

// Debug mode indices when DEBUG_MODE = DEBUG_MBFF
typedef enum {
    MBFF_DEBUG_ERPM_SQ_DIFF = 0,    // eRPM² differential [/1e6]
    MBFF_DEBUG_DELTA_GYRO,          // delta(gyro) [deg/s, sample-to-sample]
    MBFF_DEBUG_E_RPM2,              // Current E_rpm² [x10000]
    MBFF_DEBUG_FF_OUTPUT,           // FF output (raw)
    MBFF_DEBUG_CLASSIC_FF,          // Classic FF for comparison
    MBFF_DEBUG_SETPOINT,            // Current setpoint
    MBFF_DEBUG_GYRO_RATE,           // Current gyro rate
    MBFF_DEBUG_GATE_REASON,         // Gating reason flags
} mbffDebugIndex_e;

// Debug mode indices when DEBUG_MODE = DEBUG_MBFF_LEARN
typedef enum {
    MBFF_LEARN_DEBUG_E_ROLL = 0,    // Learned E for roll [x10000]
    MBFF_LEARN_DEBUG_E_PITCH,       // Learned E for pitch [x10000]
    MBFF_LEARN_DEBUG_GYRO_DELTA,    // Computed gyro delta [deg/s]
    MBFF_LEARN_DEBUG_SETPOINT,      // Current setpoint [deg/s]
    MBFF_LEARN_DEBUG_SAMPLES_ROLL,  // Learn count roll
    MBFF_LEARN_DEBUG_SAMPLES_PITCH, // Learn count pitch
    MBFF_LEARN_DEBUG_ERPM_SQ_DIFF,  // Current eRPM² diff [/1e6]
    MBFF_LEARN_DEBUG_GATE_REASON,   // Gate reason flags
} mbffLearnDebugIndex_e;

#endif // USE_MBFF
