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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_AUTOTUNE

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/pid_init.h"

#include "io/beeper.h"

#include "pg/autotune.h"

#include "sensors/gyro.h"

#include "autotune.h"

#define AUTOTUNE_SAMPLE_COUNT 200       // Number of samples to collect
#define AUTOTUNE_TARGET_SAMPLE_RATE_HZ 200  // Target sample rate (Hz) - 5ms between samples
#define AUTOTUNE_NOISE_THRESHOLD 4.0f  // Max gyro RMS (deg/s) before considering noise-limited

// Tuning phases
typedef enum {
    TUNE_PHASE_ESTABLISH_RATIO = 0,
    TUNE_PHASE_SCALE_UP = 1,
    TUNE_PHASE_FINE_TUNE = 2,
    TUNE_PHASE_COMPLETE = 3
} tunePhase_e;

// Phase progress tracking
typedef struct {
    tunePhase_e phase;
    uint8_t phaseIteration;
    float establishedRatio;
    float phase1StartP;
    float phase1StartD;
    float maxCleanP;
    float maxCleanD;
    uint8_t noiseHitIteration;
    bool noiseHit;
} tuneProgress_t;

// Iteration history for convergence detection
typedef struct {
    float P;
    float D;
    uint8_t phase;
} iterationHistory_t;

// Analysis results
typedef struct autotuneAnalysis_s {
    float riseTime;           // microseconds (10-90%)
    float riseTimeToSetpoint; // microseconds (first entry to ±5% band) - NEW
    float trackingError;      // deg/s average error after reaching setpoint - NEW
    float overshoot;          // percentage (can be negative if undershoot)
    float settlingTime;       // microseconds (deprecated - too strict)
    float dtermOscillationFreq;  // Hz (0 if none) - D-term output oscillation
    float dtermOscillationAmplitude; // D-term oscillation magnitude (RMS)
    float gyroOscillationFreq;   // Hz (0 if none) - Actual gyro oscillation
    float steadyStateError;   // deg/s
    float gyroNoiseRms;       // deg/s RMS in stable regions
    
    bool noiseLimited;        // true if tuning constrained by noise
    bool valid;               // analysis succeeded
} autotuneAnalysis_t;

// Runtime state
typedef struct autotuneRuntime_s {
    autotuneState_e state;
    autotuneState_e prevState;
    timeUs_t stateEnteredAt;
    uint8_t currentAxis;
    bool active;
    bool stateJustEntered;          // Flag set on state change, cleared after first execution
    
    // Measurement data
    timeUs_t excitationStartTime;
    float gyroHistory[AUTOTUNE_SAMPLE_COUNT];      // Gyro response (deg/s)
    float setpointHistory[AUTOTUNE_SAMPLE_COUNT];  // Commanded rate (deg/s)
    float dtermHistory[AUTOTUNE_SAMPLE_COUNT];     // D-term output
    uint16_t sampleIndex;
    uint16_t sampleCount;
    uint16_t sampleCounter;         // Counter for sample interval
    uint16_t sampleIntervalLoops;   // Calculated sample interval based on PID frequency
    
    // Analysis results
    autotuneAnalysis_t analysis;
    
    // Original PID gains (for comparison and rollback)
    uint8_t originalP;
    uint8_t originalI;
    uint8_t originalD;
    
    // Phase tracking
    tuneProgress_t tuneProgress;
    
    // Iteration tracking
    uint8_t iteration;
    iterationHistory_t history[20];  // Last 20 iterations for convergence detection
    bool axisComplete[3];            // Track completion per axis
    
} autotuneRuntime_t;

static autotuneRuntime_t runtime = {
    .state = AUTOTUNE_STATE_IDLE,
    .active = false,
    .tuneProgress.phase = TUNE_PHASE_ESTABLISH_RATIO,
    .tuneProgress.establishedRatio = 0.70f
};

// State names for debugging
static const char* const stateNames[] = {
    "IDLE",
    "SETUP",
    "WAIT_STABLE",
    "EXCITE",
    "MEASURE",
    "ANALYZE",
    "ADJUST",
    "COMPLETE",
    "ABORTED"
};

// ============================================================================
// UNIFIED DEBUG OUTPUT
// ============================================================================
// debug[0] = State (0-8)
// debug[1] = Iteration (global)
// debug[2] = Phase (0=Establish, 1=Scale, 2=Fine-tune, 3=Complete)
// debug[3] = Phase iteration (within current phase)
// debug[4] = Current P value
// debug[5] = Current D value
// debug[6] = Status code (see below)
// debug[7] = Damping info: (dampingState * 100) + zeroCrossings
//
// Status codes (debug[6]):
//   0 = Normal operation
//   1 = Underdamped (oscillating)
//   2 = Overdamped (sluggish)
//   3 = Critically damped (good)
//   5-7 = Unknown damping fallback cases
//   10 = Noise limit hit
//   11 = Cautious scaling
//   12 = Aggressive scaling
//   20 = D-term noise detected
//   100+ = Safety check failed
//   110+ = Validation warnings
//
// Damping state values (debug[7] / 100):
//   0 = Unknown
//   1 = Underdamped
//   2 = Critically damped
//   3 = Overdamped
// Zero crossings = debug[7] % 100
// ============================================================================

static int16_t g_lastStatusCode = 0;  // Persistent status for debug output

static void autotuneUpdateDebug(void)
{
    const uint8_t axis = runtime.currentAxis;
    
    DEBUG_SET(DEBUG_AUTOTUNE, 0, runtime.state);
    DEBUG_SET(DEBUG_AUTOTUNE, 1, runtime.iteration);
    DEBUG_SET(DEBUG_AUTOTUNE, 2, runtime.tuneProgress.phase);
    DEBUG_SET(DEBUG_AUTOTUNE, 3, runtime.tuneProgress.phaseIteration);
    DEBUG_SET(DEBUG_AUTOTUNE, 4, currentPidProfile->pid[axis].P);
    DEBUG_SET(DEBUG_AUTOTUNE, 5, currentPidProfile->pid[axis].D);
    DEBUG_SET(DEBUG_AUTOTUNE, 6, g_lastStatusCode);
    DEBUG_SET(DEBUG_AUTOTUNE, 7, 0);  // Unused
}

static void autotuneUpdateMetricsDebug(void)
{
    const autotuneAnalysis_t *a = &runtime.analysis;
    const uint8_t axis = runtime.currentAxis;
    
    // Hybrid layout: Essential tracking + key metrics in 8 channels
    // This allows single debug mode to capture both state and metrics
    
    // Channel 0: State (0-8) - so we can find ANALYZE periods
    DEBUG_SET(DEBUG_AUTOTUNE_METRICS, 0, runtime.state);
    
    // Channel 1: Iteration number
    DEBUG_SET(DEBUG_AUTOTUNE_METRICS, 1, runtime.iteration);
    
    // Channel 2: Current P gain
    DEBUG_SET(DEBUG_AUTOTUNE_METRICS, 2, currentPidProfile->pid[axis].P);
    
    // Channel 3: Current D gain
    DEBUG_SET(DEBUG_AUTOTUNE_METRICS, 3, currentPidProfile->pid[axis].D);
    
    // Channel 4: Overshoot (% × 10) - critical for damping assessment
    DEBUG_SET(DEBUG_AUTOTUNE_METRICS, 4, (int16_t)(a->overshoot * 10.0f));
    
    // Channel 5: D-term oscillation amplitude × 10 - key overdamping indicator
    DEBUG_SET(DEBUG_AUTOTUNE_METRICS, 5, (int16_t)(a->dtermOscillationAmplitude * 10.0f));
    
    // Channel 6: Rise time to setpoint (ms × 10) - NEW metric
    DEBUG_SET(DEBUG_AUTOTUNE_METRICS, 6, (int16_t)(a->riseTimeToSetpoint / 100.0f));
    
    // Channel 7: Tracking error (deg/s × 10) - NEW metric
    DEBUG_SET(DEBUG_AUTOTUNE_METRICS, 7, (int16_t)(a->trackingError * 10.0f));
}

static void autotuneSetStatus(int16_t code)
{
    g_lastStatusCode = code;
}

// Initialize autotune system
void autotuneInit(void)
{
    memset(&runtime, 0, sizeof(runtime));
    runtime.state = AUTOTUNE_STATE_IDLE;
    runtime.prevState = AUTOTUNE_STATE_IDLE;
    runtime.active = false;
    runtime.currentAxis = FD_ROLL;  // Start with roll
    
    // Initialize phase tracking
    runtime.tuneProgress.phase = TUNE_PHASE_ESTABLISH_RATIO;
    runtime.tuneProgress.phaseIteration = 0;
    runtime.tuneProgress.establishedRatio = 0.70f;  // Default starting ratio
    runtime.tuneProgress.phase1StartP = 0.0f;
    runtime.tuneProgress.phase1StartD = 0.0f;
    runtime.tuneProgress.maxCleanP = 0.0f;
    runtime.tuneProgress.maxCleanD = 0.0f;
    runtime.tuneProgress.noiseHitIteration = 0;
    runtime.tuneProgress.noiseHit = false;
    
    runtime.iteration = 0;
    g_lastStatusCode = 0;
    
    // Calculate sample interval based on actual PID loop frequency
    // Target: collect samples at AUTOTUNE_TARGET_SAMPLE_RATE_HZ (e.g., 200Hz = 5ms)
    // sampleInterval = pidFrequency / targetSampleRate
    // e.g., 8000Hz / 200Hz = 40 loops
    //       4000Hz / 200Hz = 20 loops
    const uint16_t pidFreq = pidGetPidFrequency();
    runtime.sampleIntervalLoops = pidFreq / AUTOTUNE_TARGET_SAMPLE_RATE_HZ;
    
    // Sanity check - ensure at least 1 loop interval
    if (runtime.sampleIntervalLoops < 1) {
        runtime.sampleIntervalLoops = 1;
    }
}

// Check if it's safe to run autotune
static bool autotuneSafetyCheck(void)
{
    // Check attitude limits (in decidegrees, so 300 = 30 degrees)
    const int safetyMarginDeci = autotuneConfig()->safety_margin * 10;
    
    if (ABS(attitude.raw[FD_ROLL]) > safetyMarginDeci ||
        ABS(attitude.raw[FD_PITCH]) > safetyMarginDeci) {
        autotuneSetStatus(101);  // Attitude check failed
        return false;
    }
    
    // Check gyro rates not excessive (deg/sec)
    for (int axis = 0; axis < 3; axis++) {
        if (fabsf(gyro.gyroADCf[axis]) > 500.0f) {
            autotuneSetStatus(102);  // Gyro rate check failed
            return false;
        }
    }
    
    // Check throttle is in reasonable range (hover)
    const int8_t throttlePercent = calculateThrottlePercent();
    if (throttlePercent < 20 || throttlePercent > 80) {
        autotuneSetStatus(103);  // Throttle check failed
        return false;
    }
    
    // Check no stick input (pilot not commanding) - relaxed to 20%
    const float maxStickDeflection = MAX(fabsf(getRcDeflection(FD_ROLL)), 
                                         MAX(fabsf(getRcDeflection(FD_PITCH)), 
                                             fabsf(getRcDeflection(FD_YAW))));
    
    if (maxStickDeflection > 0.2f) {
        autotuneSetStatus(104);  // Stick deflection check failed
        return false;
    }
    
    return true;
}

// Collect sample data
static void autotuneSampleData(float gyro, float setpoint, float dterm)
{
    if (runtime.sampleIndex >= AUTOTUNE_SAMPLE_COUNT) {
        return;  // Buffer full
    }
    
    runtime.gyroHistory[runtime.sampleIndex] = gyro;
    runtime.setpointHistory[runtime.sampleIndex] = setpoint;
    runtime.dtermHistory[runtime.sampleIndex] = dterm;
    runtime.sampleIndex++;
    runtime.sampleCount = runtime.sampleIndex;
}

// Generate bidirectional 3-2-1-1 doublet excitation (zero net displacement)
// Returns additional setpoint to add to pilot's command
float autotuneGetExcitation(timeUs_t currentTimeUs)
{
    if (runtime.state != AUTOTUNE_STATE_EXCITE) {
        return 0.0f;
    }
    
    const timeUs_t elapsed = cmpTimeUs(currentTimeUs, runtime.excitationStartTime);
    const uint32_t stepDurationUs = autotuneConfig()->step_duration_ms * 1000;
    const float amplitude = autotuneConfig()->step_amplitude;
    
    // Bidirectional doublet (6 phases): +A, -2A, +2A, -2A, +2A, -A
    // Net displacement: A - 2A + 2A - 2A + 2A - A = 0
    // Creates two strong symmetric swings
    if (elapsed < stepDurationUs) {
        return amplitude;                    // Phase 1: +A
    } else if (elapsed < stepDurationUs * 2) {
        return -2.0f * amplitude;            // Phase 2: -2A
    } else if (elapsed < stepDurationUs * 3) {
        return 2.0f * amplitude;             // Phase 3: +2A (strong transition)
    } else if (elapsed < stepDurationUs * 4) {
        return -2.0f * amplitude;            // Phase 4: -2A (strong opposite)
    } else if (elapsed < stepDurationUs * 5) {
        return 2.0f * amplitude;             // Phase 5: +2A
    } else if (elapsed < stepDurationUs * 6) {
        return -amplitude;                   // Phase 6: -A (settle to zero)
    }
    
    return 0.0f;  // Done
}

// Calculate RMS noise in stable regions (before or after doublet)
static float calculateGyroNoise(uint16_t startIdx, uint16_t endIdx)
{
    if (startIdx >= endIdx || endIdx > runtime.sampleCount) {
        return 0.0f;
    }
    
    float sumSquares = 0.0f;
    uint16_t count = 0;
    
    for (uint16_t i = startIdx; i < endIdx; i++) {
        const float sample = runtime.gyroHistory[i];
        sumSquares += sample * sample;
        count++;
    }
    
    if (count == 0) {
        return 0.0f;
    }
    
    return sqrtf(sumSquares / count);  // RMS in deg/s
}

// TODO: The following functions are OLD and will be replaced with new metrics:
// - calculateSettlingTime() - too strict, never converges with flight noise
// - calculateDtermAmplitude() - keep this one, shows good correlation

// Calculate rise time to setpoint (time to first reach within 5% of target)
// This is simpler and more robust than 10-90% rise time
static float calculateRiseTimeToSetpoint(uint16_t stepIdx, float target)
{
    if (stepIdx == 0 || stepIdx >= runtime.sampleCount - 10) {
        return 999999.0f;  // Invalid
    }
    
    // Get baseline (average of 5 samples before step)
    float baseline = 0.0f;
    const uint16_t baselineStart = (stepIdx >= 5) ? (stepIdx - 5) : 0;
    for (uint16_t i = baselineStart; i < stepIdx; i++) {
        baseline += runtime.gyroHistory[i];
    }
    baseline /= (stepIdx - baselineStart);
    
    // Define 5% band around target
    const float tolerance = fabsf(target) * 0.05f;
    const float upperBound = target + tolerance;
    const float lowerBound = target - tolerance;
    
    // Search for first entry into the band
    const uint16_t searchLimit = MIN(stepIdx + 150, runtime.sampleCount);  // 75ms max search
    for (uint16_t i = stepIdx; i < searchLimit; i++) {
        const float gyro = runtime.gyroHistory[i];
        if (gyro >= lowerBound && gyro <= upperBound) {
            // Found first entry - convert samples to microseconds
            return (i - stepIdx) * (1000000.0f / AUTOTUNE_TARGET_SAMPLE_RATE_HZ);
        }
    }
    
    return 999999.0f;  // Never reached setpoint
}

// Calculate sum of absolute tracking error after reaching setpoint
// This measures how well the system tracks after initial response
static float calculateTrackingError(uint16_t stepIdx, float target, float riseTimeUs)
{
    if (stepIdx == 0 || riseTimeUs >= 999999.0f) {
        return 999999.0f;  // Invalid
    }
    
    // Start measuring error after rise time
    const uint16_t riseSamples = (uint16_t)(riseTimeUs * AUTOTUNE_TARGET_SAMPLE_RATE_HZ / 1000000.0f);
    const uint16_t trackingStart = stepIdx + riseSamples;
    
    // Measure error for duration of the step (step_duration_ms)
    const uint16_t stepDurationSamples = (autotuneConfig()->step_duration_ms * AUTOTUNE_TARGET_SAMPLE_RATE_HZ) / 1000;
    const uint16_t trackingEnd = MIN(trackingStart + stepDurationSamples, runtime.sampleCount);
    
    if (trackingStart >= trackingEnd) {
        return 999999.0f;
    }
    
    // Sum absolute tracking error
    float errorSum = 0.0f;
    uint16_t count = 0;
    
    for (uint16_t i = trackingStart; i < trackingEnd; i++) {
        const float error = fabsf(runtime.gyroHistory[i] - target);
        errorSum += error;
        count++;
    }
    
    // Return average error per sample (deg/s)
    return (count > 0) ? (errorSum / count) : 999999.0f;
}

// Calculate rise time (10% to 90% of target)
static float calculateRiseTime(uint16_t stepIdx, float target)
{
    if (stepIdx == 0 || stepIdx >= runtime.sampleCount - 10) {
        return 0.0f;
    }
    
    // Get baseline (average of 5 samples before step)
    float baseline = 0.0f;
    const uint16_t baselineStart = (stepIdx >= 5) ? (stepIdx - 5) : 0;
    for (uint16_t i = baselineStart; i < stepIdx; i++) {
        baseline += runtime.gyroHistory[i];
    }
    baseline /= (stepIdx - baselineStart);
    
    // Calculate travel distance and direction
    const float travelDistance = fabsf(target - baseline);
    const float travelDirection = (target > baseline) ? 1.0f : -1.0f;
    
    const float threshold10 = baseline + (travelDirection * travelDistance * 0.1f);
    const float threshold90 = baseline + (travelDirection * travelDistance * 0.9f);
    
    uint16_t idx10 = 0;
    uint16_t idx90 = 0;
    
    // Find 10% crossing (check if gyro has crossed threshold in correct direction)
    for (uint16_t i = stepIdx; i < runtime.sampleCount; i++) {
        const float gyro = runtime.gyroHistory[i];
        if (travelDirection > 0) {
            if (gyro >= threshold10) {
                idx10 = i;
                break;
            }
        } else {
            if (gyro <= threshold10) {
                idx10 = i;
                break;
            }
        }
    }
    
    // Find 90% crossing
    for (uint16_t i = idx10; i < runtime.sampleCount; i++) {
        const float gyro = runtime.gyroHistory[i];
        if (travelDirection > 0) {
            if (gyro >= threshold90) {
                idx90 = i;
                break;
            }
        } else {
            if (gyro <= threshold90) {
                idx90 = i;
                break;
            }
        }
    }
    
    if (idx10 == 0 || idx90 == 0 || idx90 <= idx10) {
        return 0.0f;
    }
    
    // Convert sample difference to time (microseconds)
    const float samplePeriod = 1000000.0f / AUTOTUNE_TARGET_SAMPLE_RATE_HZ;  // us
    return (idx90 - idx10) * samplePeriod;
}

// Detect D-term oscillations by analyzing D-term output directly
// Returns oscillation frequency in Hz, or 0 if no significant oscillation
static float detectOscillation(uint16_t stepIdx, float target)
{
    UNUSED(target);
    
    if (stepIdx == 0 || stepIdx >= runtime.sampleCount - 20) {
        return 0.0f;
    }
    
    // Analyze D-term output after the step response starts
    const uint16_t analyzeStart = stepIdx + 5;  // Skip initial transient
    const uint16_t searchSamples = MIN(30, runtime.sampleCount - analyzeStart);
    const uint16_t searchEnd = analyzeStart + searchSamples;
    
    // Count zero crossings in D-term output
    int zeroCrossings = 0;
    bool wasPositive = (runtime.dtermHistory[analyzeStart] > 0);
    
    for (uint16_t i = analyzeStart + 1; i < searchEnd; i++) {
        const bool isPositive = (runtime.dtermHistory[i] > 0);
        if (isPositive != wasPositive) {
            zeroCrossings++;
            wasPositive = isPositive;
        }
    }
    
    // High D causes high-frequency oscillations in D-term
    // Need at least 4 crossings (2 full cycles) to be significant
    if (zeroCrossings < 4) {
        return 0.0f;
    }
    
    // Calculate frequency
    const float timeWindowSec = searchSamples / (float)AUTOTUNE_TARGET_SAMPLE_RATE_HZ;
    const float frequency = (zeroCrossings / 2.0f) / timeWindowSec;
    
    // D-term oscillations typically >10Hz
    return (frequency > 10.0f) ? frequency : 0.0f;
}

// Detect gyro oscillations by analyzing gyro response
// Returns oscillation frequency in Hz, or 0 if no significant oscillation
static float detectGyroOscillation(uint16_t stepIdx, float target)
{
    if (stepIdx == 0 || stepIdx >= runtime.sampleCount - 20) {
        return 0.0f;
    }
    
    // Analyze gyro after step starts
    const uint16_t analyzeStart = stepIdx + 5;
    const uint16_t searchSamples = MIN(30, runtime.sampleCount - analyzeStart);
    const uint16_t searchEnd = analyzeStart + searchSamples;
    
    // Count oscillations around the target
    const float targetAbs = fabsf(target);
    int crossings = 0;
    bool wasAbove = (fabsf(runtime.gyroHistory[analyzeStart]) > targetAbs);
    
    for (uint16_t i = analyzeStart + 1; i < searchEnd; i++) {
        const bool isAbove = (fabsf(runtime.gyroHistory[i]) > targetAbs);
        if (isAbove != wasAbove) {
            crossings++;
            wasAbove = isAbove;
        }
    }
    
    // Need at least 2 crossings (underdamped oscillation)
    if (crossings < 2) {
        return 0.0f;
    }
    
    const float timeWindowSec = searchSamples / (float)AUTOTUNE_TARGET_SAMPLE_RATE_HZ;
    const float frequency = (crossings / 2.0f) / timeWindowSec;
    
    // Gyro oscillations typically 3-15Hz (underdamped system)
    return (frequency > 2.0f && frequency < 30.0f) ? frequency : 0.0f;
}

// Calculate overshoot percentage
static float calculateOvershoot(uint16_t stepIdx, float newSetpoint)
{
    if (stepIdx < 5 || stepIdx >= runtime.sampleCount - 10) {
        return 0.0f;
    }
    
    // Get previous setpoint to calculate step magnitude and direction
    const float prevSetpoint = runtime.setpointHistory[stepIdx - 1];
    const float stepMagnitude = fabsf(newSetpoint - prevSetpoint);
    const float stepDirection = (newSetpoint > prevSetpoint) ? 1.0f : -1.0f;
    
    if (stepMagnitude < 10.0f) {  // Ignore small changes
        return 0.0f;
    }
    
    // Find stable baseline BEFORE the step (look back 5 samples)
    float baselineGyro = 0.0f;
    const uint16_t baselineStart = stepIdx - 5;
    for (uint16_t i = baselineStart; i < stepIdx; i++) {
        baselineGyro += runtime.gyroHistory[i];
    }
    baselineGyro /= 5.0f;  // Average of 5 samples before step
    
    // Search for peak response in step direction
    const uint16_t searchSamples = (autotuneConfig()->step_duration_ms * AUTOTUNE_TARGET_SAMPLE_RATE_HZ) / 1000;
    const uint16_t searchEnd = MIN(stepIdx + searchSamples, runtime.sampleCount);
    
    float peakGyro = baselineGyro;
    float extremeGyro = baselineGyro;  // Track opposite direction too
    
    for (uint16_t i = stepIdx; i < searchEnd; i++) {
        const float gyro = runtime.gyroHistory[i];
        
        // Track peak in step direction (signed comparison)
        if (stepDirection > 0) {
            if (gyro > peakGyro) peakGyro = gyro;
            if (gyro < extremeGyro) extremeGyro = gyro;  // Opposite direction
        } else {
            if (gyro < peakGyro) peakGyro = gyro;
            if (gyro > extremeGyro) extremeGyro = gyro;  // Opposite direction
        }
    }
    
    // Calculate signed travel distances
    const float signedTravel = (peakGyro - baselineGyro) * stepDirection;  // Positive if in correct direction
    const float expectedTravel = stepMagnitude;
    
    // Overshoot is excess travel beyond the step magnitude
    if (signedTravel > expectedTravel) {
        const float overshootAmount = signedTravel - expectedTravel;
        return (overshootAmount / expectedTravel) * 100.0f;
    }
    
    // Check for reverse overshoot (oscillation bouncing back past baseline)
    const float reverseTravel = fabsf(extremeGyro - baselineGyro);
    if (reverseTravel > stepMagnitude * 0.1f) {  // More than 10% in wrong direction
        return (reverseTravel / stepMagnitude) * 100.0f;
    }
    
    return 0.0f;
}

// Calculate D-term oscillation amplitude (RMS during response window)
static float calculateDtermAmplitude(uint16_t stepIdx)
{
    if (stepIdx < 5 || stepIdx >= runtime.sampleCount - 20) {
        return 0.0f;
    }
    
    // Analyze D-term for 50ms after the step (100 samples @ 2kHz)
    const uint16_t analyzeStart = stepIdx + 5;  // Skip initial spike
    const uint16_t analyzeSamples = MIN(100, runtime.sampleCount - analyzeStart);
    const uint16_t analyzeEnd = analyzeStart + analyzeSamples;
    
    // Calculate RMS of D-term output
    float sumSquares = 0.0f;
    int count = 0;
    
    for (uint16_t i = analyzeStart; i < analyzeEnd; i++) {
        const float dterm = runtime.dtermHistory[i];
        sumSquares += dterm * dterm;
        count++;
    }
    
    if (count == 0) {
        return 0.0f;
    }
    
    return sqrtf(sumSquares / count);  // RMS amplitude
}

// Validate analysis results
static bool isValidAnalysis(const autotuneAnalysis_t *a)
{
    // TEMPORARILY DISABLED VALIDATION FOR DEBUGGING
    // Allow all data through so we can see what's actually happening
    
    // Still set status markers to show what WOULD have failed
    if (a->riseTime < 0 || a->riseTime > 200000.0f) {
        autotuneSetStatus(110);  // Invalid rise time (but allowing through)
    }
    if (a->overshoot < -10.0f || a->overshoot > 500.0f) {  // Increased threshold
        autotuneSetStatus(111);  // Invalid overshoot (but allowing through)
    }
    if (a->gyroNoiseRms > 150.0f) {  // Increased from 50 - was too aggressive
        autotuneSetStatus(112);  // Excessive noise (but allowing through)
    }
    
    // Always return true for now
    return true;
}

// Detect stick interference during excitation
static bool detectStickInterference(void)
{
    const uint8_t axis = runtime.currentAxis;
    
    // Get RC deflection (normalized)
    float rcDeflection = 0.0f;
    
    switch (axis) {
        case FD_ROLL:
            rcDeflection = fabsf(rcCommand[ROLL]) / 500.0f;
            break;
        case FD_PITCH:
            rcDeflection = fabsf(rcCommand[PITCH]) / 500.0f;
            break;
        case FD_YAW:
            rcDeflection = fabsf(rcCommand[YAW]) / 500.0f;
            break;
    }
    
    // Threshold: 10% stick deflection
    if (rcDeflection > 0.10f) {
        autotuneSetStatus(113);  // Stick interference
        return true;
    }
    
    return false;
}

// Check for convergence
static bool checkConvergence(void)
{
    if (runtime.iteration < 3) return false;
    
    const uint8_t axis = runtime.currentAxis;
    const pidProfile_t *pidProfile = currentPidProfile;
    
    float currentP = pidProfile->pid[axis].P;
    float currentD = pidProfile->pid[axis].D;
    
    // Look back 2 iterations
    if (runtime.iteration >= 2) {
        float prevP = runtime.history[runtime.iteration - 2].P;
        float prevD = runtime.history[runtime.iteration - 2].D;
        
        float pChange = fabsf((currentP - prevP) / prevP);
        float dChange = fabsf((currentD - prevD) / prevD);
        
        // Converged if changes < 5% for 2 consecutive iterations
        if (pChange < 0.05f && dChange < 0.05f) {
            return true;
        }
    }
    
    return false;
}

// Perform complete step response analysis
static void autotuneAnalyzeResponse(void)
{
    autotuneAnalysis_t *a = &runtime.analysis;
    memset(a, 0, sizeof(*a));
    
    // Validate we have enough samples
    if (runtime.sampleCount < 50) {
        a->valid = false;
        return;
    }
    
    // Calculate gyro noise from first 20 samples (stable hover before doublet)
    a->gyroNoiseRms = calculateGyroNoise(0, MIN(20, runtime.sampleCount));
    
    // Check if we're noise-limited
    a->noiseLimited = (a->gyroNoiseRms > AUTOTUNE_NOISE_THRESHOLD);
    
    // Bidirectional doublet has 6 step transitions - analyze all of them
    const uint16_t stepDurationSamples = (autotuneConfig()->step_duration_ms * AUTOTUNE_TARGET_SAMPLE_RATE_HZ) / 1000;
    const float stepAmplitude = autotuneConfig()->step_amplitude;
    const float stepThreshold = stepAmplitude * 0.5f;  // Detect transitions > half amplitude
    float totalRiseTime = 0.0f;
    float totalRiseTimeToSetpoint = 0.0f;
    float totalTrackingError = 0.0f;
    float totalOvershoot = 0.0f;
    float totalDtermAmp = 0.0f;
    float totalOscFreq = 0.0f;
    float totalGyroOsc = 0.0f;
    int validSteps = 0;
    int riseToSetpointSteps = 0;
    int oscillatingSteps = 0;
    int gyroOscSteps = 0;
    
    // Find all step edges and analyze each response
    for (uint16_t i = 20; i < runtime.sampleCount - stepDurationSamples; i++) {
        // Detect step by comparing setpoint change
        const float prevSetpoint = runtime.setpointHistory[i > 0 ? i - 1 : 0];
        const float currSetpoint = runtime.setpointHistory[i];
        const float setpointChange = fabsf(currSetpoint - prevSetpoint);
        
        // If setpoint changed significantly, this is a step edge
        if (setpointChange > stepThreshold) {
            const float riseTime = calculateRiseTime(i, currSetpoint);
            const float riseTimeToSp = calculateRiseTimeToSetpoint(i, currSetpoint);
            const float trackingErr = calculateTrackingError(i, currSetpoint, riseTimeToSp);
            const float overshoot = calculateOvershoot(i, currSetpoint);
            const float dtermAmp = calculateDtermAmplitude(i);
            const float dtermOsc = detectOscillation(i, currSetpoint);
            const float gyroOsc = detectGyroOscillation(i, currSetpoint);
            
            if (riseTime > 0) {
                totalRiseTime += riseTime;
                totalOvershoot += overshoot;
                totalDtermAmp += dtermAmp;
                
                if (riseTimeToSp < 999999.0f && trackingErr < 999999.0f) {
                    totalRiseTimeToSetpoint += riseTimeToSp;
                    totalTrackingError += trackingErr;
                    riseToSetpointSteps++;
                }
                
                if (dtermOsc > 0) {
                    totalOscFreq += dtermOsc;
                    oscillatingSteps++;
                }
                if (gyroOsc > 0) {
                    totalGyroOsc += gyroOsc;
                    gyroOscSteps++;
                }
                validSteps++;
            }
            
            // Skip ahead to avoid detecting same step multiple times
            i += stepDurationSamples / 2;
        }
    }
    
    // Average the metrics across all valid steps
    if (validSteps > 0) {
        a->riseTime = totalRiseTime / validSteps;
        a->riseTimeToSetpoint = (riseToSetpointSteps > 0) ? (totalRiseTimeToSetpoint / riseToSetpointSteps) : 999999.0f;
        a->trackingError = (riseToSetpointSteps > 0) ? (totalTrackingError / riseToSetpointSteps) : 999999.0f;
        a->overshoot = totalOvershoot / validSteps;
        a->settlingTime = 0;  // Removed - old metric was too strict
        a->dtermOscillationAmplitude = totalDtermAmp / validSteps;
        a->dtermOscillationFreq = (oscillatingSteps > 0) ? (totalOscFreq / oscillatingSteps) : 0.0f;
        a->gyroOscillationFreq = (gyroOscSteps > 0) ? (totalGyroOsc / gyroOscSteps) : 0.0f;
        
        a->valid = true;
    } else {
        a->valid = false;
    }
    
    // Simplified metrics for now (full implementation in later iteration)
    a->steadyStateError = 0;  // TODO: Calculate final error
    
    // Note: Debug output now handled uniformly by autotuneUpdateDebug()
}

// Phase 1: Establish P/D Ratio via sweep from D=0 to overdamped
// Strategy: Start with no damping, progressively add D until overdamped,
//           then back off to find critical damping region
static void phase1_establishRatio(void)
{
    tuneProgress_t *progress = &runtime.tuneProgress;
    pidProfile_t *pidProfile = currentPidProfile;
    const uint8_t axis = runtime.currentAxis;
    
    // First iteration: Set starting point with minimal D (not zero - too unstable!)
    if (progress->phaseIteration == 0) {
        // Store absolute original gains
        runtime.originalP = pidProfile->pid[axis].P;
        runtime.originalD = pidProfile->pid[axis].D;
        runtime.originalI = pidProfile->pid[axis].I;
        
        // Start with conservative P and MINIMAL D (30% of P)
        // D=0 is too aggressive and makes quad unflyable
        float startP = runtime.originalP * 0.70f;
        float startD = startP * 0.30f;  // Start with 30% ratio (underdamped but stable)
        float startI = runtime.originalI * 0.70f;
        
        // Apply starting gains
        pidProfile->pid[axis].P = lrintf(startP);
        pidProfile->pid[axis].D = lrintf(startD);
        pidProfile->pid[axis].I = lrintf(startI);
        
        // Initialize tracking
        progress->establishedRatio = startD / startP;
        progress->phase1StartP = startP;
        
        pidInitConfig(currentPidProfile);
        
        progress->phaseIteration++;
        autotuneSetStatus(1);  // Starting sweep
        return;
    }
    
    // Sweep strategy: Progressively increase D from 30% to 2.5× P
    float currentP = pidProfile->pid[axis].P;
    float currentD = pidProfile->pid[axis].D;
    
    // Increase D by 15% of P each iteration (slower sweep to see more detail)
    float nextD = currentD + (currentP * 0.15f);
    
    // Safety limit: don't exceed 2.5x P (should hit noise/oscillation limit before this)
    if (nextD > currentP * 2.5f) {
        // ERROR: Reached maximum D without finding overdamped response
        // This indicates the sweep failed - either analysis is broken or gains are way off
        // Abort autotune rather than continuing to waste iterations
        autotuneSetStatus(100);  // Phase 1 sweep failed
        runtime.state = AUTOTUNE_STATE_ABORTED;
        beeper(BEEPER_AUTOTUNE_FAIL);
        return;
    }
    
    // Apply increased D
    pidProfile->pid[axis].D = lrintf(nextD);
    progress->establishedRatio = nextD / currentP;
    
    pidInitConfig(currentPidProfile);
    
    progress->phaseIteration++;
    
    // Status: just show sweep progress (classification is unreliable)
    autotuneSetStatus(progress->phaseIteration);  // Show iteration number as status
    
    // Extended iteration limit for full metric collection
    if (progress->phaseIteration >= 15) {
        // Sweep complete - collected metrics for 15 D/P ratios (0.30 to ~2.7)
        // STOP HERE for data analysis - don't continue to Phase 2 automatically
        // User will review metrics and manually select best iteration
        autotuneSetStatus(102);  // Sweep complete, awaiting manual scoring
        progress->phase = TUNE_PHASE_COMPLETE;
        progress->phaseIteration = 0;
        beeper(BEEPER_AUTOTUNE_DONE);
        return;
    }
}

// Phase 2: Scale Up Gains
static void phase2_scaleUp(void)
{
    tuneProgress_t *progress = &runtime.tuneProgress;
    const autotuneAnalysis_t *a = &runtime.analysis;
    pidProfile_t *pidProfile = currentPidProfile;
    const uint8_t axis = runtime.currentAxis;
    
    float P = pidProfile->pid[axis].P;
    float D = pidProfile->pid[axis].D;
    float I = pidProfile->pid[axis].I;
    
    // Check if we've hit a limit
    bool hitNoiseLimit = (a->dtermOscillationFreq > 0);
    bool approachingOvershoot = (a->overshoot > autotuneConfig()->target_overshoot + 5.0f);
    bool noiseLimited = a->noiseLimited;
    
    if (hitNoiseLimit) {
        // Hit D-term noise - back off and complete
        P *= 0.85f;
        D *= 0.85f;
        I *= 0.85f;
        
        progress->phase = TUNE_PHASE_FINE_TUNE;
        progress->phaseIteration = 0;
        progress->noiseHitIteration = runtime.iteration;
        progress->noiseHit = true;
        autotuneSetStatus(10);  // Noise limit hit
        beeper(BEEPER_READY_BEEP);
    }
    else if (approachingOvershoot || noiseLimited) {
        // Approaching limits - increase cautiously
        P *= 1.10f;
        D *= 1.10f;
        I *= 1.10f;
        
        progress->maxCleanP = P;
        progress->maxCleanD = D;
        autotuneSetStatus(11);  // Cautious increase
    }
    else {
        // Still clean - push hard!
        P *= 1.25f;  // Aggressive 25% increase
        D *= 1.25f;
        I *= 1.25f;
        
        progress->maxCleanP = P;
        progress->maxCleanD = D;
        autotuneSetStatus(12);  // Aggressive increase
    }
    
    // Apply new gains
    pidProfile->pid[axis].P = lrintf(constrainf(P, 10, 250));
    pidProfile->pid[axis].D = lrintf(constrainf(D, 5, 100));
    pidProfile->pid[axis].I = lrintf(constrainf(I, 10, 250));
    
    pidInitConfig(currentPidProfile);
    
    // Maximum iterations safety limit
    progress->phaseIteration++;
    if (progress->phaseIteration >= 8) {
        progress->phase = TUNE_PHASE_FINE_TUNE;
        progress->phaseIteration = 0;
        autotuneSetStatus(13);  // Max iterations
    }
}

// Phase 3: Fine-Tune
static void phase3_fineTune(void)
{
    tuneProgress_t *progress = &runtime.tuneProgress;
    const autotuneAnalysis_t *a = &runtime.analysis;
    pidProfile_t *pidProfile = currentPidProfile;
    const uint8_t axis = runtime.currentAxis;
    
    float P = pidProfile->pid[axis].P;
    float D = pidProfile->pid[axis].D;
    float I = pidProfile->pid[axis].I;
    
    if (a->dtermOscillationFreq > 15.0f) {
        // High-frequency noise - reduce D more aggressively
        D *= 0.85f;
        autotuneSetStatus(20);  // D-term noise (high)
    }
    else if (a->dtermOscillationFreq > 0) {
        // Moderate noise - small D reduction
        D *= 0.95f;
        autotuneSetStatus(21);  // D-term noise (moderate)
    }
    else if (progress->noiseHit && a->dtermOscillationFreq == 0) {
        // Was noisy, now clean - done!
        progress->phase = TUNE_PHASE_COMPLETE;
        progress->phaseIteration = 0;
        autotuneSetStatus(22);  // Clean after noise hit
        beeper(BEEPER_AUTOTUNE_DONE);
        return;
    }
    else {
        // Never hit noise - try one more push
        P *= 1.05f;
        D *= 1.05f;
        I *= 1.05f;
        autotuneSetStatus(23);  // Final push
    }
    
    // Apply gains
    pidProfile->pid[axis].P = lrintf(constrainf(P, 10, 250));
    pidProfile->pid[axis].D = lrintf(constrainf(D, 5, 100));
    pidProfile->pid[axis].I = lrintf(constrainf(I, 10, 250));
    
    pidInitConfig(currentPidProfile);
    
    // Phase 3 is short - max 3 iterations
    progress->phaseIteration++;
    if (progress->phaseIteration >= 3) {
        progress->phase = TUNE_PHASE_COMPLETE;
        progress->phaseIteration = 0;
        beeper(BEEPER_AUTOTUNE_DONE);
    }
}

// Calculate and apply new PID gains based on analysis
static void autotuneAdjustGains(void)
{
    const autotuneAnalysis_t *a = &runtime.analysis;
    tuneProgress_t *progress = &runtime.tuneProgress;
    
    if (!a->valid) {
        return;  // Can't tune with invalid analysis
    }
    
    // Dispatch to phase-specific handler
    switch (progress->phase) {
        case TUNE_PHASE_ESTABLISH_RATIO:
            phase1_establishRatio();
            break;
        case TUNE_PHASE_SCALE_UP:
            phase2_scaleUp();
            break;
        case TUNE_PHASE_FINE_TUNE:
            phase3_fineTune();
            break;
        case TUNE_PHASE_COMPLETE:
            // Axis complete
            runtime.axisComplete[runtime.currentAxis] = true;
            break;
    }
    
    // Increment iteration counter
    runtime.iteration++;
    
    // Store history for convergence detection
    if (runtime.iteration < 20) {
        const pidProfile_t *pidProfile = currentPidProfile;
        const uint8_t axis = runtime.currentAxis;
        runtime.history[runtime.iteration].P = pidProfile->pid[axis].P;
        runtime.history[runtime.iteration].D = pidProfile->pid[axis].D;
        runtime.history[runtime.iteration].phase = progress->phase;
    }
}

// Change state with logging
static void changeState(autotuneState_e newState, timeUs_t currentTimeUs)
{
    if (runtime.state != newState) {
        runtime.prevState = runtime.state;
        runtime.state = newState;
        runtime.stateEnteredAt = currentTimeUs;
        runtime.stateJustEntered = true;  // Set flag for state initialization
        // Note: State now logged by autotuneUpdateDebug() every cycle
    }
}

// Update autotune state machine
void autotuneUpdate(timeUs_t currentTimeUs)
{
    // Check if autotune mode is active
    const bool shouldBeActive = IS_RC_MODE_ACTIVE(BOXAUTOTUNE) && 
                                autotuneConfig()->autotune_enabled &&
                                ARMING_FLAG(ARMED);
    
    // Activation/deactivation logic
    if (shouldBeActive && !runtime.active) {
        // Just activated - initialize everything
        autotuneInit();
        runtime.active = true;
        changeState(AUTOTUNE_STATE_SETUP, currentTimeUs);
        beeper(BEEPER_AUTOTUNE_START);
    } else if (!shouldBeActive && runtime.active) {
        // Just deactivated - reinitialize everything for next run
        autotuneInit();  // Properly reset all state
    }
    
    // Always update debug output (consistent view)
    autotuneUpdateDebug();
    
    // Update metrics debug if that mode is enabled
    if (debugMode == DEBUG_AUTOTUNE_METRICS) {
        autotuneUpdateMetricsDebug();
    }
    
    if (!runtime.active) {
        return;
    }
    
    // Safety check first - abort if unsafe
    if (runtime.state != AUTOTUNE_STATE_IDLE && 
        runtime.state != AUTOTUNE_STATE_COMPLETE &&
        runtime.state != AUTOTUNE_STATE_ABORTED) {
        
        if (!autotuneSafetyCheck()) {
            changeState(AUTOTUNE_STATE_ABORTED, currentTimeUs);
            beeper(BEEPER_AUTOTUNE_FAIL);
            autotuneSetStatus(100);  // Safety check failed
            return;
        }
    }
    
    // State machine
    switch (runtime.state) {
        case AUTOTUNE_STATE_IDLE:
            // Should not be here if active
            changeState(AUTOTUNE_STATE_SETUP, currentTimeUs);
            break;
            
        case AUTOTUNE_STATE_SETUP:
            // Wait 2 seconds before starting to allow stabilization
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 2000000) {
                changeState(AUTOTUNE_STATE_WAIT_STABLE, currentTimeUs);
            }
            break;
            
        case AUTOTUNE_STATE_WAIT_STABLE:
            // Wait for stable conditions for 1 second
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 1000000) {
                // Start excitation
                changeState(AUTOTUNE_STATE_EXCITE, currentTimeUs);
            }
            break;
            
        case AUTOTUNE_STATE_EXCITE:
            // Check for pilot interference
            if (detectStickInterference()) {
                // Abort this iteration
                beeper(BEEPER_DISARM_REPEAT);
                changeState(AUTOTUNE_STATE_WAIT_STABLE, currentTimeUs);
                break;
            }
            
            // Initialize on entry
            if (runtime.stateJustEntered) {
                runtime.stateJustEntered = false;
                runtime.excitationStartTime = currentTimeUs;
                runtime.sampleIndex = 0;
                runtime.sampleCount = 0;
                runtime.sampleCounter = 0;
            }
            
            // Collect samples (subsampled to avoid filling buffer too fast)
            runtime.sampleCounter++;
            if (runtime.sampleCounter >= runtime.sampleIntervalLoops) {
                runtime.sampleCounter = 0;
                const float excitation = autotuneGetExcitation(currentTimeUs);
                const float dterm = pidData[runtime.currentAxis].D;
                autotuneSampleData(gyro.gyroADCf[runtime.currentAxis], excitation, dterm);
            }
            
            // Bidirectional doublet duration is 6x step duration
            const timeDelta_t doubletDurationUs = autotuneConfig()->step_duration_ms * 6000;
            if (cmpTimeUs(currentTimeUs, runtime.excitationStartTime) >= doubletDurationUs) {
                changeState(AUTOTUNE_STATE_MEASURE, currentTimeUs);
            }
            break;
            
        case AUTOTUNE_STATE_MEASURE:
            // Continue collecting response data after doublet
            runtime.sampleCounter++;
            if (runtime.sampleCounter >= runtime.sampleIntervalLoops) {
                runtime.sampleCounter = 0;
                const float dterm = pidData[runtime.currentAxis].D;
                autotuneSampleData(gyro.gyroADCf[runtime.currentAxis], 0.0f, dterm);
            }
            
            // Hold for 50ms minimum to collect settling data
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) >= 50000 || 
                runtime.sampleIndex >= AUTOTUNE_SAMPLE_COUNT) {
                changeState(AUTOTUNE_STATE_ANALYZE, currentTimeUs);
            }
            break;
            
        case AUTOTUNE_STATE_ANALYZE:
            // Analyze the collected response data
            if (runtime.stateJustEntered) {
                runtime.stateJustEntered = false;
                
                // Perform step response analysis
                autotuneAnalyzeResponse();
                
                // Check if analysis was successful
                if (!runtime.analysis.valid) {
                    // Analysis failed - abort the process
                    changeState(AUTOTUNE_STATE_ABORTED, currentTimeUs);
                    beeper(BEEPER_ACC_CALIBRATION_FAIL);
                    autotuneSetStatus(200);  // Analysis invalid
                    break;
                }
                
                // Validate analysis before using
                if (!isValidAnalysis(&runtime.analysis)) {
                    // Bad data - retry this iteration
                    beeper(BEEPER_DISARM_REPEAT);
                    autotuneSetStatus(201);  // Analysis rejected
                    changeState(AUTOTUNE_STATE_IDLE, currentTimeUs);
                    break;
                }
            }
            
            // Hold in ANALYZE state for 200ms so debug values are visible in logs
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) >= 200000) {
                changeState(AUTOTUNE_STATE_ADJUST, currentTimeUs);
            }
            break;
            
        case AUTOTUNE_STATE_ADJUST: {
            // Adjust PID gains based on analysis results
            if (runtime.stateJustEntered) {
                runtime.stateJustEntered = false;
                
                // Increment iteration counter
                runtime.iteration++;
                
                // Calculate and apply new PID gains
                autotuneAdjustGains();
                
                // Check for convergence
                if (checkConvergence()) {
                    runtime.tuneProgress.phase = TUNE_PHASE_COMPLETE;
                    beeper(BEEPER_AUTOTUNE_DONE);
                }
            }
            
            // Hold in ADJUST state for 200ms so debug values are visible in logs
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) >= 200000) {
                const tuneProgress_t *adjProgress = &runtime.tuneProgress;
                // Check if tuning is complete
                if (adjProgress->phase == TUNE_PHASE_COMPLETE) {
                    changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
                    beeper(BEEPER_AUTOTUNE_DONE);
                } else {
                    // Continue to next iteration
                    changeState(AUTOTUNE_STATE_WAIT_STABLE, currentTimeUs);
                    beeper(BEEPER_AUTOTUNE_START);  // Beep to indicate new iteration
                }
            }
            break;
        }
            
        case AUTOTUNE_STATE_COMPLETE:
            // Hold in complete state briefly, then restart for next iteration
            autotuneSetStatus(50);  // Completion marker
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) >= 1000000) {  // 1 second delay
                // Reset sample buffers but keep original gains for comparison
                runtime.sampleIndex = 0;
                runtime.sampleCount = 0;
                runtime.sampleCounter = 0;
                // Don't clear originalP/I/D - we want to compare all iterations to baseline
                
                // Restart the tuning cycle to test the new gains
                changeState(AUTOTUNE_STATE_WAIT_STABLE, currentTimeUs);
                beeper(BEEPER_AUTOTUNE_START);  // Beep to indicate new iteration
            }
            break;
            
        case AUTOTUNE_STATE_ABORTED:
            // Stay in aborted state until switch turned off
            autotuneSetStatus(99);  // Abort marker
            break;
    }
}

// Query functions
bool autotuneIsActive(void)
{
    return runtime.active;
}

autotuneState_e autotuneGetState(void)
{
    return runtime.state;
}

const char* autotuneGetStateName(void)
{
    return stateNames[runtime.state];
}

// Modify setpoint to add excitation signal
// This is called from PID loop to inject the doublet
float autotuneModifySetpoint(uint8_t axis, float setpoint, timeUs_t currentTimeUs)
{
    if (!runtime.active || runtime.state != AUTOTUNE_STATE_EXCITE) {
        return setpoint;
    }
    
    // Only modify the axis being tested
    if (axis != runtime.currentAxis) {
        return setpoint;
    }
    
    // Add excitation signal to pilot's setpoint
    const float excitation = autotuneGetExcitation(currentTimeUs);
    return setpoint + excitation;
}

#endif // USE_AUTOTUNE
