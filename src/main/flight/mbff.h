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
 * Model-Based Feedforward (MBFF)
 *
 * Replaces classic feedforward (F) with a trajectory- and model-based approach:
 * 1. Reference trajectory smoothly tracks pilot intent
 * 2. Desired acceleration computed from trajectory + preview correction
 * 3. Torque effectiveness model scales output based on RPM
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
    uint8_t  enabled;           // 0 = off, 1 = on
    uint8_t  ts;                // Trajectory time constant [ms], range 5-50
    uint8_t  tp;                // Preview horizon [ms], range 10-100
    uint8_t  ka;                // Trajectory accel gain [x100], 0-200
    uint8_t  kr;                // Preview/tracking correction gain [x100], 0-200
    uint16_t b0;                // Base torque effectiveness [x100], 10-500
    uint8_t  b1;                // RPM-based effectiveness [x100], 0-200
    uint8_t  ff_limit;          // FF limit as % of pidSumLimit, 10-100
    uint16_t gain;              // Master output gain [x10], 10-2000 (1.0 to 200.0)
    uint8_t  preview_threshold; // Setpoint threshold for preview term [deg/s], 0-200
} mbffConfig_t;

PG_DECLARE(mbffConfig_t, mbffConfig);

// ---------------------------------------------------------------------------
// Runtime state (per axis)
// ---------------------------------------------------------------------------

typedef struct mbffAxisState_s {
    float omega_ref;            // Reference rate [deg/s]
    float prev_omega_ref;       // Previous reference rate (for debug)
    float prev_setpoint;        // Previous setpoint for derivative calculation
} mbffAxisState_t;

typedef struct mbffRuntime_s {
    // Per-axis state
    mbffAxisState_t axis[XYZ_AXIS_COUNT];

    // Cached config values (converted to float at init)
    float ts_sec;               // Trajectory time constant [s]
    float tp_sec;               // Preview horizon [s]
    float ka;                   // Trajectory acceleration gain
    float kr;                   // Preview correction gain
    float b0;                   // Base effectiveness
    float b1;                   // RPM effectiveness coefficient
    float ff_limit;             // Max FF output
    float gain;                 // Master output gain
    float preview_threshold;    // Setpoint threshold for preview gating [deg/s]

    // Runtime data
    float avg_rpm_sq;           // Average motor RPM squared
    float effectiveness;        // Current torque effectiveness g(RPM²)
    float dT;                   // Loop time [s]
    float pidFrequency;         // PID loop frequency [Hz]

    bool enabled;               // Runtime enable flag
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
 * Clears reference trajectory to current setpoint.
 */
void mbffReset(void);

/**
 * Update MBFF for a single axis.
 *
 * @param axis          Axis index (FD_ROLL, FD_PITCH, FD_YAW)
 * @param setpoint      Rate setpoint from RC [deg/s]
 * @param gyroRate      Measured gyro rate [deg/s]
 * @param dT            Loop time [s]
 * @return              Feedforward command to add to PID output
 */
float mbffUpdate(int axis, float setpoint, float gyroRate, float dT);

/**
 * Update RPM data for effectiveness model.
 * Call once per PID loop with current motor RPM data.
 */
void mbffUpdateRpm(void);

/**
 * Get current reference rate for an axis.
 * Useful for modifying PID error (error = omega_ref - gyro) if desired.
 */
float mbffGetReferenceRate(int axis);

/**
 * Get current torque effectiveness.
 * For debug/logging.
 */
float mbffGetEffectiveness(void);

/**
 * Check if MBFF is enabled.
 */
bool mbffIsEnabled(void);

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------

// Debug mode indices when DEBUG_MODE = DEBUG_MBFF
typedef enum {
    MBFF_DEBUG_OMEGA_REF = 0,       // Reference rate [deg/s]
    MBFF_DEBUG_ALPHA_DES,           // Desired acceleration [deg/s²]
    MBFF_DEBUG_EFFECTIVENESS,       // Torque effectiveness [x1000]
    MBFF_DEBUG_FF_OUTPUT,           // MBFF output [x100]
    MBFF_DEBUG_CLASSIC_FF,          // Classic FF for comparison [x100]
    MBFF_DEBUG_RPM_SQ,              // Avg RPM² / 1000
    MBFF_DEBUG_TRAJ_ERROR,          // ω_sp - ω_ref [deg/s]
    MBFF_DEBUG_TRACK_ERROR,         // ω_ref - ω_meas [deg/s]
} mbffDebugIndex_e;

#endif // USE_MBFF
