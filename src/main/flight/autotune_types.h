/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "common/time.h"

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

#define AUTOTUNE_SAMPLE_COUNT           200     // Sample buffer size
#define AUTOTUNE_TARGET_SAMPLE_RATE_HZ  200     // 5ms between samples
#define AUTOTUNE_HISTORY_SIZE           20      // Iterations to track

// Maneuver detection thresholds
#define MANEUVER_ROLL_RATE_THRESHOLD    200.0f  // deg/s to detect roll
#define MANEUVER_PITCH_RATE_THRESHOLD   200.0f  // deg/s to detect flip
#define MANEUVER_THROTTLE_THRESHOLD     0.2f    // throttle delta for punch (20%)
#define MANEUVER_MIN_DURATION_MS        100     // minimum maneuver duration

// Hover detection thresholds
#define HOVER_GYRO_THRESHOLD            30.0f   // deg/s max for "settled"
#define HOVER_STICK_THRESHOLD           0.15f   // max stick deflection
#define HOVER_STABLE_TIME_MS            500     // time to confirm hover

// Analysis thresholds
#define NOISE_TARGET_RMS               15.0f    // deg/s RMS noise backstop (resonance is primary concern)
#define OVERSHOOT_TARGET_MIN            5.0f    // minimum acceptable overshoot %
#define OVERSHOOT_TARGET_MAX           18.0f    // maximum acceptable overshoot %
#define OSCILLATION_THRESHOLD          80.0f    // D-term oscillation amplitude

// Gain adjustment limits
#define GAIN_ADJUST_STEP_PERCENT        8.0f    // Base adjustment step
#define GAIN_MIN_VALUE                  10      // Minimum PID value
#define GAIN_MAX_VALUE                  250     // Maximum PID value

// ============================================================================
// FILTER FREQUENCY LIMITS (Hz)
// ============================================================================
// Format: [MIN - DEFAULT - MAX]
// These limits apply to:
// - Static filters: the static_hz value
// - Dynamic filters: the dyn_min_hz value (lower bound of dynamic range)
//
// Gyro LPF1:   125 - 250 - 375  (dyn_min when dynamic, static_hz when static)
// Gyro LPF2:   250 - 500 - 750  (always static)
// Dterm LPF1:   37 -  75 - 112  (dyn_min when dynamic, static_hz when static)
// Dterm LPF2:   75 - 150 - 225  (always static)

#define GYRO_LPF1_MIN_HZ                125
#define GYRO_LPF1_DEFAULT_HZ            250
#define GYRO_LPF1_MAX_HZ                375

#define GYRO_LPF2_MIN_HZ                250
#define GYRO_LPF2_DEFAULT_HZ            500
#define GYRO_LPF2_MAX_HZ                750

#define DTERM_LPF1_MIN_HZ                37
#define DTERM_LPF1_DEFAULT_HZ            75
#define DTERM_LPF1_MAX_HZ               112

#define DTERM_LPF2_MIN_HZ                75
#define DTERM_LPF2_DEFAULT_HZ           150
#define DTERM_LPF2_MAX_HZ               225

// Filter adjustment step size
#define FILTER_STEP_HZ                   10

// Diagnostic test parameters
#define DIAG_FILTER_STEP_HZ              50    // Larger step for clear differentiation
#define DIAG_IMPROVEMENT_THRESHOLD     10.0f  // 10% improvement to be significant
#define DIAG_MAX_ITERATIONS              5    // Max diagnostic cycles before giving up

// ============================================================================
// REASON CODES (for debug[7] - explains what autotune is doing)
// ============================================================================
// Format: XYZZ where X=category, Y=subcategory, ZZ=specific reason
// Category 1xxx: Hover tune
// Category 2xxx: Filter tune  
// Category 3xxx: PID tune (roll/pitch)
// Category 9xxx: No action / waiting

// Hover tune reasons (1xxx)
// Basic states (10xx)
#define REASON_HOVER_WAITING            1000    // Waiting for stable hover
#define REASON_HOVER_MEASURING          1001    // Measuring motor RMS

// Noise source: Gyro resonance (11xx) - raw gyro noise is dominant
// When gyroRms/motorRms > 0.5, gyro resonance is driving motor noise
#define REASON_HOVER_GYRO_LPF1_DOWN     1100    // Gyro noise high, lowering gyro LPF1
#define REASON_HOVER_GYRO_LPF2_DOWN     1101    // Gyro noise high, lowering gyro LPF2
#define REASON_HOVER_GYRO_AT_LIMIT      1102    // Gyro filters at minimum, can't help more

// Noise source: D-term amplification (112x) - D-term is amplifying gyro noise
// When dTermRms/gyroRms > expected D response, D-term is amplifying
#define REASON_HOVER_DTERM_LPF1_DOWN    1120    // D-term noisy, lowering dterm LPF1
#define REASON_HOVER_DTERM_LPF2_DOWN    1121    // D-term noisy, lowering dterm LPF2
#define REASON_HOVER_DTERM_D_DOWN       1122    // D-term noisy, reducing D gain
#define REASON_HOVER_DTERM_AT_LIMIT     1123    // D-term filters at minimum

// Noise source: P-term oscillation (113x) - P-term oscillating independently
// When pTermRms >> dTermRms, P is oscillating (feedback instability)
#define REASON_HOVER_POSC_P_DOWN        1130    // P oscillation, reducing P gain
#define REASON_HOVER_POSC_D_UP          1131    // P oscillation, raising D for damping
#define REASON_HOVER_POSC_AT_LIMIT      1132    // P oscillation but at limits

// Low noise: relaxing filters (114x) - motor RMS excellent, trying to raise filters
#define REASON_HOVER_RELAX_DTERM_LPF1   1140    // Excellent RMS, raising dterm LPF1
#define REASON_HOVER_RELAX_DTERM_LPF2   1141    // Excellent RMS, raising dterm LPF2
#define REASON_HOVER_RELAX_GYRO_LPF1    1142    // Excellent RMS, raising gyro LPF1
#define REASON_HOVER_RELAX_GYRO_LPF2    1143    // Excellent RMS, raising gyro LPF2
#define REASON_HOVER_RELAX_D_UP         1144    // Excellent RMS, raising D gain
#define REASON_HOVER_RELAX_AT_MAX       1145    // Already at max filter/gain settings

// Legacy/fallback codes (for backwards compatibility)
#define REASON_HOVER_RMS_HIGH_D_DOWN    1110    // (deprecated) use REASON_HOVER_DTERM_D_DOWN
#define REASON_HOVER_RMS_HIGH_LPF_DOWN  1120    // (deprecated) use specific source codes
#define REASON_HOVER_RMS_LOW_LPF_UP     1210    // (deprecated) use REASON_HOVER_RELAX_*
#define REASON_HOVER_RMS_LOW_D_UP       1220    // (deprecated) use REASON_HOVER_RELAX_D_UP

// Convergence states (13xx)
#define REASON_HOVER_STABLE             1300    // Motor RMS stable, converged
#define REASON_HOVER_COMPLETE           1999    // Hover tune complete

// Procedural diagnostic reasons (15xx)
#define REASON_DIAG_BASELINE            1500    // Measuring baseline RMS
#define REASON_DIAG_ROLL_TEST           1510    // Testing with Roll * 0.5
#define REASON_DIAG_PITCH_TEST          1520    // Testing with Pitch * 0.5
#define REASON_DIAG_GYRO_LPF1_TEST      1530    // Testing with Gyro LPF1 - 50Hz
#define REASON_DIAG_DTERM_LPF1_TEST     1540    // Testing with Dterm LPF1 - 50Hz
#define REASON_DIAG_VERIFY_BASELINE     1545    // Reconfirm baseline after tests complete
#define REASON_DIAG_ANALYZING           1550    // Analyzing results
#define REASON_DIAG_FIX_ROLL            1560    // Identified Roll as dominant, applying fix
#define REASON_DIAG_FIX_PITCH           1561    // Identified Pitch as dominant, applying fix
#define REASON_DIAG_FIX_GYRO_LPF1       1562    // Identified Gyro LPF1 as dominant, applying fix
#define REASON_DIAG_FIX_DTERM_LPF1      1563    // Identified Dterm LPF1 as dominant, applying fix
#define REASON_DIAG_NO_IMPROVEMENT      1570    // No test showed >10% improvement
#define REASON_DIAG_TARGET_REACHED      1580    // Motor RMS within target
#define REASON_DIAG_MAX_ITERATIONS      1590    // Max diagnostic iterations reached

// Filter tune reasons (2xxx)
#define REASON_FILTER_WAITING           2000    // Waiting for throttle pump
#define REASON_FILTER_COLLECTING        2001    // Collecting noise data
#define REASON_FILTER_RESONANCE_LPF     2110    // Resonance detected, lowering LPF
#define REASON_FILTER_RESONANCE_NOTCH   2120    // Resonance detected, adding notch (future)
#define REASON_FILTER_NOISE_HIGH_LPF    2210    // Noise high (no resonance), lowering LPF
#define REASON_FILTER_NOISE_LOW_LPF     2310    // Noise low, raising LPF
#define REASON_FILTER_NOISE_OK          2400    // Noise in acceptable range
#define REASON_FILTER_NO_CHANGE         2500    // No change needed/possible
#define REASON_FILTER_AT_LIMIT          2600    // All filters at limit, can't adjust further
#define REASON_FILTER_COMPLETE          2999    // Filter tune complete

// PID tune reasons (3xxx)
#define REASON_PID_WAITING              3000    // Waiting for maneuver
#define REASON_PID_COLLECTING           3001    // Collecting response data
#define REASON_PID_OVERSHOOT_P_DOWN     3110    // Overshoot high, reducing P
#define REASON_PID_OVERSHOOT_D_UP       3120    // Overshoot high, raising D
#define REASON_PID_SLUGGISH_P_UP        3210    // Response slow, raising P
#define REASON_PID_SLUGGISH_D_DOWN      3220    // Response slow, reducing D
#define REASON_PID_OSCILLATION_D_UP     3310    // Oscillation detected, raising D
#define REASON_PID_OSCILLATION_P_DOWN   3320    // Oscillation detected, reducing P
#define REASON_PID_NOISE_D_DOWN         3410    // Noise high, reducing D
#define REASON_PID_RESPONSE_GOOD        3500    // Response good, no change
#define REASON_PID_AT_LIMIT             3600    // Gains at min/max, can't adjust further
#define REASON_ROLL_COMPLETE            3910    // Roll axis tuning complete
#define REASON_PITCH_COMPLETE           3920    // Pitch axis tuning complete
#define REASON_PID_COMPLETE             3999    // PID tune complete (all axes)

// I-term tune reasons (34xx)
#define REASON_PID_DRIFT_I_UP           3420    // Drift detected, raising I
#define REASON_PID_BOUNCEBACK_I_DOWN    3430    // Bounceback detected, lowering I
#define REASON_PID_SLOW_OSC_I_DOWN      3440    // Slow oscillation, lowering I

// F-term (feedforward) tune reasons (35xx)
#define REASON_PID_LAG_F_UP             3510    // Stick lag, raising F
#define REASON_PID_LEAD_F_DOWN          3520    // Gyro leading stick, lowering F
#define REASON_PID_PHASE_F_UP           3530    // Large phase lag, raising F

// No action reasons (9xxx)
#define REASON_IDLE                     9000    // Autotune idle
#define REASON_GRACE_PERIOD             9100    // In grace period, checking mode
#define REASON_DATA_INVALID             9200    // Data invalid, skipping adjustment
#define REASON_AT_LIMIT                 9300    // Value at min/max limit

// ============================================================================
// ENUMERATIONS
// ============================================================================

// Simplified state machine
typedef enum {
    AUTOTUNE_STATE_IDLE = 0,        // Waiting for activation
    AUTOTUNE_STATE_ARMED,           // Active, waiting for maneuver
    AUTOTUNE_STATE_DETECTING,       // Maneuver in progress
    AUTOTUNE_STATE_COLLECTING,      // Collecting post-maneuver data
    AUTOTUNE_STATE_SETTLING,        // Waiting for hover
    AUTOTUNE_STATE_ANALYZING,       // Running analysis
    AUTOTUNE_STATE_ADJUSTING,       // Applying gain changes
    AUTOTUNE_STATE_SIGNALING,       // Wiggle to signal ready
    AUTOTUNE_STATE_COMPLETE,        // Tuning complete
    AUTOTUNE_STATE_ABORTED,         // Safety abort
    AUTOTUNE_STATE_COUNT
} autotuneState_e;

// Tuning mode based on detected maneuver
typedef enum {
    TUNE_MODE_NONE = 0,
    TUNE_MODE_ROLL,                 // Roll maneuvers detected -> tune roll P/D
    TUNE_MODE_PITCH,                // Flip maneuvers detected -> tune pitch P/D
    TUNE_MODE_FILTER,               // Throttle punches -> tune filters
} autotuneTuneMode_e;

// Procedural hover diagnostic phases
// Each phase applies one test, measures for 500ms, then restores settings
// Note: RMS is computed at END of each phase, so debug output shows PREVIOUS phase's result
typedef enum {
    HOVER_DIAG_IDLE = 0,            // Not running diagnostics
    HOVER_DIAG_BASELINE,            // Measuring with current settings (initial)
    HOVER_DIAG_ROLL_TEST,           // Roll gains * 0.5
    HOVER_DIAG_PITCH_TEST,          // Pitch gains * 0.5  
    HOVER_DIAG_GYRO_LPF1_TEST,      // Gyro LPF1 - 50Hz
    HOVER_DIAG_DTERM_LPF1_TEST,     // Dterm LPF1 - 50Hz
    HOVER_DIAG_VERIFY_BASELINE,     // Reconfirm baseline after all tests (settings restored)
    HOVER_DIAG_ANALYZING,           // Comparing results
    HOVER_DIAG_COMPLETE,            // Diagnostic cycle done
    HOVER_DIAG_PHASE_COUNT
} hoverDiagPhase_e;

// Response quality classification
typedef enum {
    RESPONSE_UNKNOWN = 0,
    RESPONSE_UNDERDAMPED,           // Too much overshoot, oscillating
    RESPONSE_OVERDAMPED,            // Too slow, sluggish
    RESPONSE_CRITICAL,              // Good response, slight overshoot
    RESPONSE_NOISY,                 // Noise limiting further gains
    RESPONSE_EXCELLENT,             // Optimal response achieved
} autotuneResponseClass_e;

// Which gain is causing the problem
typedef enum {
    GAIN_ATTRIBUTION_NONE = 0,
    GAIN_ATTRIBUTION_P,             // P gain is the issue
    GAIN_ATTRIBUTION_I,             // I gain is the issue
    GAIN_ATTRIBUTION_D,             // D gain is the issue
    GAIN_ATTRIBUTION_F,             // Feedforward is the issue
    GAIN_ATTRIBUTION_FILTER,        // Filter settings are the issue
} autotuneGainAttribution_e;

// Adjustment direction
typedef enum {
    ADJUST_NONE = 0,
    ADJUST_INCREASE,
    ADJUST_DECREASE,
} autotuneAdjustDir_e;

// Status codes for debugging
typedef enum {
    STATUS_OK = 0,
    STATUS_WAITING_MANEUVER,
    STATUS_MANEUVER_DETECTED,
    STATUS_COLLECTING_DATA,
    STATUS_WAITING_SETTLE,
    STATUS_ANALYZING,
    STATUS_ADJUSTING_P,
    STATUS_ADJUSTING_D,
    STATUS_ADJUSTING_I,
    STATUS_ADJUSTING_F,
    STATUS_ADJUSTING_FILTER,
    STATUS_SIGNAL_READY,
    STATUS_COMPLETE,
    STATUS_ABORT_ATTITUDE,
    STATUS_ABORT_GYRO,
    STATUS_ABORT_THROTTLE,
    STATUS_ABORT_USER,
    STATUS_COUNT
} autotuneStatus_e;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Response metrics from analysis
typedef struct {
    float overshootPercent;         // Peak overshoot as percentage
    float riseTimeMs;               // Time to reach 90% of target
    float settlingTimeMs;           // Time to stay within 5% band
    float trackingError;            // Average error after settling
    float oscillationAmplitude;     // Peak-to-peak oscillation
    float oscillationFreqHz;        // Dominant oscillation frequency
    float noiseRms;                 // Gyro noise RMS in stable regions
    float steadyStateError;         // Error after full settling
    
    // I-term specific metrics
    float driftRate;                // Rate of drift after settling (deg/s per second)
    float bouncebackPercent;        // Overshoot in opposite direction after settling
    float slowOscillationHz;        // Low frequency oscillation (I-term windup sign)
    
    // F-term specific metrics
    float stickLeadError;           // How much gyro lags behind stick input
    float stickTrackingPhase;       // Phase difference between stick and response
    float initialResponseDelay;     // Delay before response starts (ms)
    float peakStickVelocity;        // Maximum stick velocity during maneuver (deg/s/sample)
    float velocityWeightedLag;      // Tracking error weighted by stick velocity (sensitive to F issues)
    
    bool isValid;                   // Analysis succeeded
} autotuneMetrics_t;

// Gain attribution result
typedef struct {
    autotuneGainAttribution_e primary;    // Main culprit
    autotuneGainAttribution_e secondary;  // Secondary contributor
    autotuneAdjustDir_e pDirection;       // Suggested P adjustment
    autotuneAdjustDir_e iDirection;       // Suggested I adjustment
    autotuneAdjustDir_e dDirection;       // Suggested D adjustment
    autotuneAdjustDir_e fDirection;       // Suggested F adjustment
    float confidence;                      // 0-1 confidence in attribution
} autotuneAttribution_t;

// Iteration history entry
typedef struct {
    uint8_t pGain;
    uint8_t iGain;
    uint8_t dGain;
    uint16_t fGain;
    float overshoot;
    float riseTime;
    float noise;
    float score;
    autotuneResponseClass_e responseClass;
} autotuneHistoryEntry_t;

// Filter tuning data
typedef struct {
    float noiseFloor;               // Baseline noise level
    float peakFrequency;            // Resonance peak frequency (Hz)
    float peakAmplitude;            // Resonance peak amplitude
    bool resonanceDetected;         // Static resonance found
    uint16_t suggestedNotchHz;      // Suggested notch center
} autotuneFilterAnalysis_t;

// ============================================================================
// NEWTON'S METHOD HISTORY TRACKING
// ============================================================================

// History tracking for calculated adjustments
#define NEWTON_HISTORY_SIZE         5       // Track last 5 parameter/metric pairs
#define NEWTON_DAMPING_FACTOR       0.90f   // Apply 90% of calculated change
#define NEWTON_MAX_STEP_PERCENT     50.0f   // Never change more than 50% at once
#define NEWTON_MIN_DERIVATIVE       0.1f    // Avoid division by near-zero

// Tune metric types - each PID term optimizes for a different metric
typedef enum {
    METRIC_MOTOR_RMS = 0,           // Hover tune - motor noise
    METRIC_OSCILLATION,             // D term - high-frequency oscillation
    METRIC_SETPOINT_TRACKING,       // P term - response to setpoint
    METRIC_STICK_TRACKING,          // F term - tracking during rapid moves
    METRIC_LONG_TERM_ERROR,         // I term - accumulated drift/offset
    METRIC_OVERSHOOT,               // Step response overshoot %
    METRIC_SETTLING_TIME,           // Step response settling time
    METRIC_COUNT
} tuneMetric_e;

// Target values for each metric
#define TARGET_MOTOR_RMS            15.0f   // Motor RMS target
#define TARGET_OSCILLATION          5.0f    // Low oscillation amplitude
#define TARGET_SETPOINT_TRACKING    0.95f   // 95% tracking accuracy
#define TARGET_STICK_TRACKING       0.90f   // 90% during rapid moves  
#define TARGET_LONG_TERM_ERROR      2.0f    // Minimal accumulated error
#define TARGET_OVERSHOOT            10.0f   // 10% max overshoot
#define TARGET_SETTLING_TIME        150.0f  // 150ms settling time

// Tunable parameter types
typedef enum {
    TUNE_PARAM_P = 0,
    TUNE_PARAM_I,
    TUNE_PARAM_D,
    TUNE_PARAM_F,
    TUNE_PARAM_DTERM_LPF1,
    TUNE_PARAM_DTERM_LPF2,
    TUNE_PARAM_GYRO_LPF1,
    TUNE_PARAM_GYRO_LPF2,
    TUNE_PARAM_COUNT
} tuneParameter_e;

// Single history entry: parameter value and resulting metric
typedef struct {
    float parameterValue;           // e.g., P gain = 45
    float metricValue;              // e.g., motor RMS = 72.5
    timeUs_t timestamp;
} newtonHistoryEntry_t;

// History buffer for one parameter
typedef struct {
    newtonHistoryEntry_t entries[NEWTON_HISTORY_SIZE];
    uint8_t count;                  // Number of valid entries (0-5)
    uint8_t writeIndex;             // Next write position (circular)
    float lastSensitivity;          // Last calculated sensitivity
} newtonHistory_t;

// Per-axis history for all tunable parameters
typedef struct {
    newtonHistory_t p;
    newtonHistory_t i;
    newtonHistory_t d;
    newtonHistory_t f;
    newtonHistory_t dtermLpf1;
} axisNewtonHistory_t;

// Complete tune history (per-axis + common filters)
typedef struct {
    axisNewtonHistory_t roll;
    axisNewtonHistory_t pitch;
    axisNewtonHistory_t yaw;
    newtonHistory_t gyroLpf1;       // Common to all axes
    newtonHistory_t gyroLpf2;       // Common to all axes
    newtonHistory_t dtermLpf1;      // Common D-term filter
} tuneNewtonHistory_t;

// ============================================================================
// SEQUENTIAL MULTI-VARIABLE ADJUSTMENT
// ============================================================================

// Pending adjustment in the queue
typedef struct {
    tuneParameter_e parameter;      // Which parameter to adjust
    int8_t axis;                    // FD_ROLL, FD_PITCH, FD_YAW or -1 for common
    float currentValue;             // Current parameter value
    float calculatedAdjustment;     // Calculated change (with 90% damping)
    float sensitivityEstimate;      // From diagnostic test
    float improvementPercent;       // Expected improvement
    tuneMetric_e metric;            // Which metric this optimizes
} pendingAdjustment_t;

#define ADJUSTMENT_QUEUE_SIZE       8   // Max pending adjustments

// Adjustment queue for sequential application
typedef struct {
    pendingAdjustment_t queue[ADJUSTMENT_QUEUE_SIZE];
    uint8_t count;                  // Number of pending adjustments
    uint8_t currentIndex;           // Currently applying this one
    float preAdjustMetric;          // Metric before current adjustment
    float preAdjustParamValue;      // Parameter value before adjustment (for revert)
} adjustmentQueue_t;

// State machine for sequential adjustment
typedef enum {
    ADJ_STATE_IDLE = 0,
    ADJ_STATE_APPLY_NEXT,           // Apply next adjustment from queue
    ADJ_STATE_MEASURING,            // Measure result of adjustment
    ADJ_STATE_VERIFY,               // Check if adjustment helped or hurt
    ADJ_STATE_REVERT,               // Revert if adjustment made things worse
    ADJ_STATE_COMPLETE,             // All adjustments verified, wiggle
} adjustmentState_e;

// ============================================================================
// MAIN RUNTIME STATE
// ============================================================================

typedef struct {
    // State machine
    autotuneState_e state;
    autotuneState_e prevState;
    timeUs_t stateEnteredAt;
    bool stateJustEntered;
    
    // Tuning context
    autotuneTuneMode_e tuneMode;
    uint8_t currentAxis;            // FD_ROLL, FD_PITCH, or FD_YAW
    uint8_t iteration;              // Current iteration count
    autotuneStatus_e status;        // Current status code
    
    // Maneuver detection
    timeUs_t maneuverStartTime;
    timeUs_t maneuverEndTime;
    float peakRate;                 // Peak rate during maneuver
    float peakThrottle;             // Peak throttle during punch
    bool maneuverActive;
    
    // Hover calibration
    int8_t hoverThrottle;           // Calibrated hover throttle (percent)
    bool hoverCalibrated;           // True once we've established hover reference
    timeUs_t hoverStableStartTime;  // When stable hover was first detected
    
    // Per-axis/mode completion tracking
    bool rollComplete;              // Roll axis tuning finished (at limit or excellent)
    bool pitchComplete;             // Pitch axis tuning finished
    bool filterComplete;            // Filter tuning finished
    
    // Hover-based motor RMS tuning (Phase 0)
    bool hoverTuneActive;           // Currently doing hover-based filter tuning
    float lastMotorRms;             // Last computed motor RMS
    float bestMotorRms;             // Best (lowest) motor RMS achieved
    timeUs_t lastMotorRmsTime;      // Last time RMS was computed
    uint8_t hoverTuneIteration;     // Hover tune iteration counter
    uint8_t stableCount;            // Iterations without improvement (for convergence)
    
    // Procedural diagnostic state
    hoverDiagPhase_e diagPhase;     // Current diagnostic phase
    uint8_t diagIteration;          // Diagnostic iteration counter (max 5)
    float diagRms[HOVER_DIAG_PHASE_COUNT];  // RMS recorded for each phase
    float diagImprovement[HOVER_DIAG_PHASE_COUNT]; // Improvement % vs baseline
    
    // Saved settings for restoration after each test
    struct {
        uint8_t rollP, rollI, rollD;
        uint16_t rollF;
        uint8_t pitchP, pitchI, pitchD;
        uint16_t pitchF;
        uint16_t gyroLpf1Hz;        // Static or dyn_min depending on mode
        uint16_t dtermLpf1Hz;       // Static or dyn_min depending on mode
        bool gyroLpf1IsDynamic;
        bool dtermLpf1IsDynamic;
    } savedSettings;
    
    // Sample buffers
    float gyroHistory[AUTOTUNE_SAMPLE_COUNT];
    float setpointHistory[AUTOTUNE_SAMPLE_COUNT];
    float dtermHistory[AUTOTUNE_SAMPLE_COUNT];
    float throttleHistory[AUTOTUNE_SAMPLE_COUNT];
    uint16_t sampleIndex;
    uint16_t sampleCount;
    uint16_t sampleCounter;
    uint16_t sampleIntervalLoops;
    
    // Current gains (cached for modification)
    uint8_t currentP;
    uint8_t currentI;
    uint8_t currentD;
    uint16_t currentF;
    
    // Best gains found
    uint8_t bestP;
    uint8_t bestI;
    uint8_t bestD;
    uint16_t bestF;
    float bestScore;
    
    // Analysis results
    autotuneMetrics_t metrics;
    autotuneAttribution_t attribution;
    autotuneResponseClass_e responseClass;
    autotuneFilterAnalysis_t filterAnalysis;
    bool dataValid;                 // True if last analysis had valid data (no crash)
    
    // Iteration history
    autotuneHistoryEntry_t history[AUTOTUNE_HISTORY_SIZE];
    uint8_t historyCount;
    
    // Signal wiggle state
    uint8_t wigglePhase;
    timeUs_t wiggleStartTime;
    
    // Debug reason code (explains what autotune is doing/did)
    uint16_t lastReasonCode;
    
    // Newton's method history tracking (per-axis)
    tuneNewtonHistory_t newtonHistory;
    
    // Sequential multi-variable adjustment
    adjustmentQueue_t adjQueue;
    adjustmentState_e adjState;
    timeUs_t adjPhaseStartTime;     // When current adjustment phase started
    
} autotuneRuntime_t;

// ============================================================================
// DEBUG MACROS
// ============================================================================

// Debug slot assignments for DEBUG_AUTOTUNE mode
// [0] = state
// [1] = tuneMode | (iteration << 4)
// [2] = P gain
// [3] = D gain
// [4] = status code
// [5] = current metric (overshoot * 10)
// [6] = current metric (noise * 10)
// [7] = score * 100
