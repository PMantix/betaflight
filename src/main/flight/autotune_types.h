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
#define NOISE_TARGET_RMS                4.0f    // deg/s target noise level
#define OVERSHOOT_TARGET_MIN            5.0f    // minimum acceptable overshoot %
#define OVERSHOOT_TARGET_MAX            18.0f   // maximum acceptable overshoot %
#define OSCILLATION_THRESHOLD           80.0f   // D-term oscillation amplitude

// Gain adjustment limits
#define GAIN_ADJUST_STEP_PERCENT        8.0f    // Base adjustment step
#define GAIN_MIN_VALUE                  10      // Minimum PID value
#define GAIN_MAX_VALUE                  250     // Maximum PID value

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
