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

#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"

// ============================================================================
// Fixed-Point Math Macros
// ============================================================================

// Q16.16 fixed-point representation (32-bit with 16 fractional bits)
#define Q16_SHIFT       16
#define Q16_ONE         (1 << Q16_SHIFT)
#define FLOAT_TO_Q16(f) ((int32_t)((f) * Q16_ONE))
#define Q16_TO_FLOAT(q) ((float)(q) / Q16_ONE)

// Q8.8 fixed-point representation (16-bit with 8 fractional bits)
#define Q8_SHIFT        8
#define Q8_ONE          (1 << Q8_SHIFT)
#define FLOAT_TO_Q8(f)  ((int16_t)((f) * Q8_ONE))
#define Q8_TO_FLOAT(q)  ((float)(q) / Q8_ONE)

// ============================================================================
// Master State Machine
// ============================================================================

typedef enum {
    AUTOTUNE_STATE_IDLE = 0,            // Inactive, waiting for activation
    AUTOTUNE_STATE_HOVER_LOCK,          // Waiting for stable hover
    AUTOTUNE_STATE_THROTTLE_SWEEP,      // Filter characterization phase
    AUTOTUNE_STATE_NOISE_CONFIRM,       // Validate filter settings
    AUTOTUNE_STATE_PD_RATIO_SEEK,       // Find critical damping (D fixed, sweep P)
    AUTOTUNE_STATE_PD_SCALE_UP,         // Scale P and D together
    AUTOTUNE_STATE_F_TUNE,              // Feedforward tuning (if needed)
    AUTOTUNE_STATE_PD_RETUNE_AFTER_F,   // Re-validate P/D after F change
    AUTOTUNE_STATE_COMPLETE,            // Tuning finished successfully
    AUTOTUNE_STATE_COUNT
} autotuneState_e;

typedef enum {
    AXIS_SUBSTATE_WAIT_EVENT = 0,       // Waiting for valid pilot maneuver
    AXIS_SUBSTATE_ANALYZING,            // Computing metrics from event
    AXIS_SUBSTATE_APPLYING,             // Applying parameter change
    AXIS_SUBSTATE_CONFIRMING,           // Waiting for next event to confirm
    AXIS_SUBSTATE_COUNT
} axisSubstate_e;

typedef enum {
    AUTOTUNE_DECISION_NONE = 0,
    AUTOTUNE_DECISION_INCREASE,
    AUTOTUNE_DECISION_DECREASE,
    AUTOTUNE_DECISION_HOLD,
    AUTOTUNE_DECISION_ROLLBACK,
    AUTOTUNE_DECISION_ADVANCE,
    AUTOTUNE_DECISION_COUNT
} autotuneDecision_e;

// ============================================================================
// Axis Enumeration
// ============================================================================

typedef enum {
    AUTOTUNE_AXIS_ROLL = 0,
    AUTOTUNE_AXIS_PITCH,
    AUTOTUNE_AXIS_YAW,
    AUTOTUNE_AXIS_COUNT
} autotuneAxis_e;

// Axis bitmask for configuration
#define AUTOTUNE_AXIS_BITMASK_ROLL   (1 << AUTOTUNE_AXIS_ROLL)
#define AUTOTUNE_AXIS_BITMASK_PITCH  (1 << AUTOTUNE_AXIS_PITCH)
#define AUTOTUNE_AXIS_BITMASK_YAW    (1 << AUTOTUNE_AXIS_YAW)
#define AUTOTUNE_AXIS_BITMASK_ALL    (AUTOTUNE_AXIS_BITMASK_ROLL | AUTOTUNE_AXIS_BITMASK_PITCH | AUTOTUNE_AXIS_BITMASK_YAW)

// ============================================================================
// Event Detection
// ============================================================================

typedef enum {
    EVENT_STATE_IDLE = 0,               // No event in progress
    EVENT_STATE_DEFLECTING,             // Stick moving away from center
    EVENT_STATE_RETURNING,              // Stick returning to center
    EVENT_STATE_COMPLETE,               // Event captured, ready for analysis
    EVENT_STATE_COUNT
} eventState_e;

typedef struct {
    float stickDeflection;              // Peak stick deflection (degrees)
    float crossAxisMovement;            // Max movement on other axes (degrees)
    float throttle;                     // Throttle during event
    bool qualityGatesPassed;            // All gates met
    timeUs_t startTimeUs;               // Event start timestamp
    timeUs_t endTimeUs;                 // Event end timestamp
} eventData_t;

typedef struct {
    eventState_e state;                 // Current event detection state
    float peakDeflection;               // Max stick deflection seen
    float crossAxisMax;                 // Max cross-axis seen
    float throttleAtStart;              // Throttle when event started
    int8_t stickSign;                   // Direction of deflection (+1 or -1)
    bool hasReversed;                   // Stick crossed zero
    timeUs_t deflectionStartUs;         // When deflection began
    timeUs_t deflectionEndUs;           // When deflection ended
} eventDetector_t;

// ============================================================================
// Event Buffer (for metrics analysis)
// ============================================================================

#define EVENT_BUFFER_SIZE 128

typedef struct {
    float gyro[EVENT_BUFFER_SIZE];      // Gyro samples during event
    float setpoint[EVENT_BUFFER_SIZE];  // Setpoint samples during event
    uint16_t count;                     // Number of samples captured
    uint16_t head;                      // Current write position
    bool capturing;                     // Currently capturing an event
} eventBuffer_t;

// ============================================================================
// Metrics (Response Analysis)
// ============================================================================

typedef struct {
    float overshootPct;                 // Peak overshoot percentage
    float reboundPct;                   // Rebound after first peak (oscillation indicator)
    float settlingTimeMs;               // Time to enter settling band
    float lagMs;                        // Time to 50% of target (response delay)
    bool hasRebound;                    // Any peaks after first > threshold
    bool valid;                         // Measurement completed successfully
} eventMetrics_t;

// ============================================================================
// Parameter Rollback System
// ============================================================================

typedef struct {
    float currentValue;                 // Current parameter value
    float previousValue;                // Value before last change (for rollback)
    float trustScore;                   // 0.0 - 1.0, affects step size
    float baseStepSize;                 // e.g., 0.15 for P, 0.10 for D
    uint8_t consecutiveGood;            // Count of good responses in a row
    uint8_t consecutiveBad;             // Count of bad responses in a row
} parameterState_t;

// ============================================================================
// Filter Characterization
// ============================================================================

#define FILTER_CHAR_MAX_SAMPLES 10

typedef struct {
    float noiseLevel[FILTER_CHAR_MAX_SAMPLES];      // Noise at different throttle points
    float throttlePoints[FILTER_CHAR_MAX_SAMPLES];  // Throttle values sampled
    uint8_t sampleCount;                            // Number of samples collected
    float minThrottleSeen;                          // Minimum throttle during sweep
    float maxThrottleSeen;                          // Maximum throttle during sweep
    float recommendedLpf;                           // Recommended lowpass frequency
    float recommendedNotch1;                        // Recommended notch 1 frequency
    float recommendedNotch2;                        // Recommended notch 2 frequency
    bool characterizationDone;                      // Filter characterization complete
    
    // Sweep tracking for robust completion detection
    uint8_t sweepCount;                             // Number of complete throttle sweeps
    bool hasSeenHighThrottle;                       // Seen throttle > 60%
    bool hasSeenLowThrottle;                        // Seen throttle < 30%
    timeUs_t lastHighThrottleUs;                    // When we last saw high throttle
    timeUs_t hoverStableStartUs;                    // When hover became stable (for post-sweep check)
} filterCharState_t;

// ============================================================================
// Bracket Search State (for Newton refinement)
// ============================================================================

typedef struct {
    float pLow;         // P value with overshoot <= target
    float pHigh;        // P value with overshoot > target  
    float osLow;        // Overshoot percentage at pLow
    float osHigh;       // Overshoot percentage at pHigh
    bool haveLow;       // Have found lower bound
    bool haveHigh;      // Have found upper bound
} bracketState_t;

// ============================================================================
// Scale History (for PD Scale Up Newton estimation)
// ============================================================================

#define MAX_SCALE_HISTORY 4

typedef struct {
    float scale[MAX_SCALE_HISTORY];     // Scale factor at each step
    float lag[MAX_SCALE_HISTORY];       // Lag measured at each scale
    uint8_t count;                      // Number of entries
} scaleHistory_t;

// ============================================================================
// F Tune History
// ============================================================================

#define MAX_F_HISTORY 3

typedef struct {
    float fValues[MAX_F_HISTORY];       // F value at each step
    float lagValues[MAX_F_HISTORY];     // Lag measured at each F
    uint8_t count;                      // Number of entries
} fHistory_t;

// ============================================================================
// Axis Tuning State
// ============================================================================

typedef struct {
    uint8_t axis;                       // FD_ROLL, FD_PITCH, FD_YAW
    axisSubstate_e substate;            // Current sub-state within axis tuning

    parameterState_t pState;            // P gain state with rollback
    parameterState_t dState;            // D gain state with rollback
    parameterState_t fState;            // F gain state with rollback

    float pdRatio;                      // Locked P/D ratio after PD_RATIO_SEEK

    bracketState_t bracket;             // Bracket state for Newton refinement

    float originalP;                    // Original P for abort recovery
    float originalD;                    // Original D for abort recovery
    float originalF;                    // Original F for abort recovery

    float ratioSeekP;                   // P value at end of ratio seek
    float ratioSeekD;                   // D value at end of ratio seek
    float currentScale;                 // Current scale factor (1.0 = ratio seek values)
    scaleHistory_t scaleHistory;        // History for Newton estimation
    fHistory_t fHistory;                // History for F tune
    float preFTuneOvershoot;            // Overshoot before F tuning
    float preFTuneLag;                  // Lag before F tuning
    bool pdRetuneNeeded;                // Flag for PD retune after F

    eventMetrics_t lastMetrics;         // Most recent event metrics
    uint8_t eventCount;                 // Events processed for this axis
    uint8_t consecutiveBadEvents;       // Count of consecutive bad events (for repeated failure detection)
    bool complete;                      // Axis tuning complete
} axisTuneState_t;

// ============================================================================
// Core Runtime State
// ============================================================================

typedef struct {
    autotuneState_e masterState;        // Current master state

    // Current tuning context
    uint8_t currentAxisIndex;           // 0=Roll, 1=Pitch, 2=Yaw
    axisTuneState_t axisState[AUTOTUNE_AXIS_COUNT];

    // Filter state
    filterCharState_t filterChar;

    // Event detection
    eventDetector_t eventDetector;
    eventData_t currentEvent;
    eventBuffer_t eventBuffer;

    // Timing
    timeUs_t startTimeUs;               // When autotune session started (for total timeout)
    timeUs_t stateEntryTimeUs;          // When current state was entered
    timeUs_t lastEventTimeUs;           // Timestamp of last valid event
    timeUs_t hoverLockStartUs;          // When hover lock began

    // Throttle tracking
    float lastThrottle;                 // Last throttle reading

    // Configuration (from settings)
    float aggressiveness;               // 0.0 - 1.0, from AGGRESSIVENESS setting
    float targetOvershootPct;           // Computed from aggressiveness
    float lagTargetMs;                  // Target lag in milliseconds

    // Progress tracking
    uint8_t totalEventsProcessed;       // Total events processed across all axes

    // Debug/telemetry
    uint16_t reasonCode;                // Last reason code for debug
    timeUs_t reasonCodeSetTimeUs;       // When reason code was last set (for pulse clearing)
    autotuneDecision_e lastDecision;    // Last decision made
} autotuneRuntime_t;
