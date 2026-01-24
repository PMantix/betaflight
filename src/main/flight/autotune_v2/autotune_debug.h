/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "build/debug.h"

// ============================================================================
// Reason Codes
// ============================================================================

typedef enum {
    // Normal progression (0-9)
    AUTOTUNE_REASON_NONE = 0,
    AUTOTUNE_REASON_EVENT_DETECTED = 1,
    AUTOTUNE_REASON_QUALITY_OK = 2,
    AUTOTUNE_REASON_DECISION_INCREASE = 3,
    AUTOTUNE_REASON_DECISION_DECREASE = 4,
    AUTOTUNE_REASON_DECISION_HOLD = 5,
    AUTOTUNE_REASON_AXIS_COMPLETE = 6,
    AUTOTUNE_REASON_PHASE_COMPLETE = 7,
    AUTOTUNE_REASON_HOVER_LOCKED = 8,
    AUTOTUNE_REASON_FILTERS_SET = 9,
    AUTOTUNE_REASON_TARGET_REACHED = 10,

    // Quality gate failures (11-19)
    AUTOTUNE_REASON_INSUFFICIENT_DEFLECTION = 11,
    AUTOTUNE_REASON_CROSS_AXIS_CONTAMINATION = 12,
    AUTOTUNE_REASON_THROTTLE_OUT_OF_BAND = 13,
    AUTOTUNE_REASON_ABNORMAL_DURATION = 14,
    AUTOTUNE_REASON_INVALID_METRICS = 15,
    AUTOTUNE_REASON_EVENT_TIMEOUT = 16,
    AUTOTUNE_REASON_STICK_NOT_CENTERED = 17,

    // Safety events (20-29)
    AUTOTUNE_REASON_ROLLBACK_OSCILLATION = 20,
    AUTOTUNE_REASON_ROLLBACK_OVERSHOOT = 21,
    AUTOTUNE_REASON_GAIN_LIMIT_MIN = 22,
    AUTOTUNE_REASON_GAIN_LIMIT_MAX = 23,
    AUTOTUNE_REASON_EVENT_LIMIT = 24,
    AUTOTUNE_REASON_NOISE_TOO_HIGH = 25,
    AUTOTUNE_REASON_TRUST_DEPLETED = 26,

    // Abort conditions (30-39)
    AUTOTUNE_REASON_ABORT_SWITCH = 30,
    AUTOTUNE_REASON_ABORT_DISARM = 31,
    AUTOTUNE_REASON_ABORT_FAILSAFE = 32,
    AUTOTUNE_REASON_ABORT_TIMEOUT = 33,
    AUTOTUNE_REASON_ABORT_USER = 34,
    AUTOTUNE_REASON_ABORT_ERROR = 35,

    AUTOTUNE_REASON_COUNT
} autotuneReason_e;

// ============================================================================
// Timing Constants
// ============================================================================

#define AUTOTUNE_HOVER_LOCK_DURATION_US     1000000     // 1 second stable hover
#define AUTOTUNE_HOVER_LOCK_TIMEOUT_US      10000000    // 10 second timeout
#define AUTOTUNE_EVENT_MAX_DURATION_US      500000      // 500ms max event duration
#define AUTOTUNE_EVENT_MIN_DURATION_US      50000       // 50ms min event duration
#define AUTOTUNE_SETTLING_WAIT_US           200000      // 200ms settling after apply

// ============================================================================
// Timeout Constants (for graceful abort)
// ============================================================================

#define AUTOTUNE_TOTAL_TIMEOUT_US           (180 * 1000000)  // 3 minutes total
#define AUTOTUNE_AXIS_TIMEOUT_US            (60 * 1000000)   // 60 seconds per axis
#define AUTOTUNE_EVENT_TIMEOUT_US           (30 * 1000000)   // 30 seconds for event

// ============================================================================
// Stick Thresholds (normalized 0.0-1.0 scale)
// ============================================================================

#define AUTOTUNE_STICK_DEFLECTION_START     0.15f       // 15% stick to start event
#define AUTOTUNE_STICK_CENTER_THRESHOLD     0.05f       // 5% considered centered
#define AUTOTUNE_STICK_DEFLECTION_MIN       0.15f       // 15% minimum deflection (normalized)
#define AUTOTUNE_CROSS_AXIS_MAX             0.10f       // 10% maximum cross-axis (normalized)

// ============================================================================
// Throttle Thresholds
// ============================================================================

#define AUTOTUNE_THROTTLE_LOW_LIMIT         0.25f       // 25% minimum throttle
#define AUTOTUNE_THROTTLE_HIGH_LIMIT        0.75f       // 75% maximum throttle
#define AUTOTUNE_THROTTLE_STABLE_BAND       0.05f       // 5% throttle change = stable

// ============================================================================
// Response Metrics Thresholds
// ============================================================================

#define AUTOTUNE_OVERSHOOT_TARGET_LOW       5.0f        // 5% lower bound target
#define AUTOTUNE_OVERSHOOT_TARGET_HIGH      10.0f       // 10% upper bound target
#define AUTOTUNE_OVERSHOOT_TARGET_DEFAULT   7.5f        // Default target
#define AUTOTUNE_OVERSHOOT_HYSTERESIS       2.0f        // Hysteresis band
#define AUTOTUNE_REBOUND_THRESHOLD          0.05f       // 5% rebound = oscillation
#define AUTOTUNE_SETTLING_BAND              0.05f       // 5% settling band
#define AUTOTUNE_NOISE_ACCEPTABLE           15.0f       // Acceptable noise ceiling

// ============================================================================
// Gain Limits
// ============================================================================

#define AUTOTUNE_GAIN_MIN                   20          // Minimum PID gain
#define AUTOTUNE_GAIN_MAX                   200         // Maximum PID gain
#define AUTOTUNE_F_GAIN_MIN                 0           // Minimum F gain
#define AUTOTUNE_F_GAIN_MAX                 200         // Maximum F gain

// ============================================================================
// Step Size Limits
// ============================================================================

#define AUTOTUNE_STEP_MIN_PCT               2           // 2% minimum step
#define AUTOTUNE_STEP_MAX_PCT               15          // 15% maximum step
#define AUTOTUNE_P_STEP_BASE                0.10f       // 10% base P step
#define AUTOTUNE_D_STEP_BASE                0.10f       // 10% base D step
#define AUTOTUNE_F_STEP_BASE                0.05f       // 5% base F step

// ============================================================================
// Trust System
// ============================================================================

#define AUTOTUNE_TRUST_INITIAL              0.5f        // Initial trust score
#define AUTOTUNE_TRUST_INCREASE             0.1f        // Trust increase on good
#define AUTOTUNE_TRUST_DECREASE             0.2f        // Trust decrease on bad
#define AUTOTUNE_TRUST_MIN                  0.1f        // Minimum trust score
#define AUTOTUNE_TRUST_MAX                  1.0f        // Maximum trust score

// ============================================================================
// Event Limits
// ============================================================================

#define AUTOTUNE_MAX_RETRIES                2           // Max retries per step
#define AUTOTUNE_MAX_EVENTS_PER_AXIS        10          // Max events per axis
#define AUTOTUNE_MAX_TOTAL_EVENTS           30          // Max total events

// ============================================================================
// Debug Channel Mapping (Blackbox)
// ============================================================================
//
// CRITICAL: These channel assignments are used by log analysis scripts.
// Do NOT change without updating LOG_ANALYSIS.md and all Python scripts.
//
// Blackbox Channel  | Macro Name              | Content              | Scale
// ------------------|-------------------------|----------------------|--------
// debug[0]          | AUTOTUNE_DEBUG_STATE    | Master state enum    | 0-8
// debug[1]          | AUTOTUNE_DEBUG_AXIS     | Current axis         | 0/1/2
// debug[2]          | AUTOTUNE_DEBUG_REASON   | Reason code          | 0-35
// debug[3]          | AUTOTUNE_DEBUG_DECISION | Decision/delta       | varies
// debug[4]          | AUTOTUNE_DEBUG_OVERSHOOT| Overshoot %          | ×10
// debug[5]          | AUTOTUNE_DEBUG_GAIN_P   | Current P gain       | direct
// debug[6]          | AUTOTUNE_DEBUG_GAIN_D   | Current D gain       | direct
// debug[7]          | AUTOTUNE_DEBUG_GAIN_F   | Current F gain       | direct
//
// DEBUG_AUTOTUNE_V2 should be defined in debug.h

#define AUTOTUNE_DEBUG_STATE                0   // debug[0]: Master state (autotuneState_e)
#define AUTOTUNE_DEBUG_AXIS                 1   // debug[1]: Current axis (0=Roll, 1=Pitch, 2=Yaw)
#define AUTOTUNE_DEBUG_REASON               2   // debug[2]: Reason code (autotuneReason_e)
#define AUTOTUNE_DEBUG_DECISION             3   // debug[3]: Decision type or gain delta
#define AUTOTUNE_DEBUG_OVERSHOOT            4   // debug[4]: Overshoot percentage × 10
#define AUTOTUNE_DEBUG_GAIN_P               5   // debug[5]: Current P gain value
#define AUTOTUNE_DEBUG_GAIN_D               6   // debug[6]: Current D gain value
#define AUTOTUNE_DEBUG_GAIN_F               7   // debug[7]: Current F gain value

// Convenience macro for setting autotune debug values
#ifdef USE_AUTOTUNE_V2
#define AUTOTUNE_DEBUG_SET(idx, val) DEBUG_SET(DEBUG_AUTOTUNE_V2, idx, val)
#else
#define AUTOTUNE_DEBUG_SET(idx, val)
#endif
