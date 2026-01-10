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

/*
 * PILOT-DRIVEN AUTOTUNER
 * ======================
 * 
 * This autotuner works by detecting pilot maneuvers and analyzing the response:
 * 
 * 1. Pilot enables autotune mode via switch
 * 2. Pilot performs maneuvers:
 *    - Rolls -> Roll axis tuning
 *    - Flips -> Pitch axis tuning  
 *    - Throttle punches -> Filter tuning
 * 3. After maneuver, drone settles to hover
 * 4. Analysis runs and gains are adjusted
 * 5. Drone wiggles to signal ready for next maneuver
 * 6. Repeat until converged or max iterations
 * 
 * The analysis attributes poor performance to specific gains:
 * - Overshoot/oscillation -> P too high or D too low
 * - Slow response -> P too low or D too high
 * - Noise -> D too high, filters too weak
 * - Steady-state error -> I too low
 * - Tracking error -> F (feedforward) issue
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

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"

#include "io/beeper.h"

#include "pg/autotune.h"

#include "sensors/gyro.h"

#include "autotune.h"
#include "autotune_types.h"
#include "autotune_analysis.h"
#include "autotune_gains.h"

// ============================================================================
// RUNTIME STATE
// ============================================================================

static autotuneRuntime_t runtime;

// State names for debugging
static const char* const stateNames[] = {
    "IDLE",
    "ARMED",
    "DETECTING",
    "COLLECTING",
    "SETTLING",
    "ANALYZING",
    "ADJUSTING",
    "SIGNALING",
    "COMPLETE",
    "ABORTED"
};

static const char* const modeNames[] = {
    "NONE",
    "ROLL",
    "PITCH",
    "FILTER"
};

// ============================================================================
// INITIALIZATION
// ============================================================================

void autotuneInit(void)
{
    memset(&runtime, 0, sizeof(runtime));
    runtime.state = AUTOTUNE_STATE_IDLE;
    runtime.prevState = AUTOTUNE_STATE_IDLE;
    runtime.tuneMode = TUNE_MODE_NONE;
    runtime.bestScore = 1000.0f;  // Start with worst score
    
    // Calculate sample interval based on PID loop frequency
    const uint16_t pidFreq = pidGetPidFrequency();
    runtime.sampleIntervalLoops = pidFreq / AUTOTUNE_TARGET_SAMPLE_RATE_HZ;
    if (runtime.sampleIntervalLoops < 1) {
        runtime.sampleIntervalLoops = 1;
    }
}

// ============================================================================
// STATE MANAGEMENT
// ============================================================================

static void changeState(autotuneState_e newState, timeUs_t currentTimeUs)
{
    if (runtime.state != newState) {
        runtime.prevState = runtime.state;
        runtime.state = newState;
        runtime.stateEnteredAt = currentTimeUs;
        runtime.stateJustEntered = true;
        
        // Update debug output
        DEBUG_SET(DEBUG_AUTOTUNE, 0, newState);
    }
}

// ============================================================================
// SAFETY CHECKS
// ============================================================================

// Check if conditions are suitable for transitioning to hover/analysis states
// Returns true if attitude and rates are calm enough
static bool isReadyForSettle(void)
{
    // Check attitude is level-ish (within safety margin)
    const int safetyMarginDeci = autotuneConfig()->safety_margin * 10;
    
    if (ABS(attitude.raw[FD_ROLL]) > safetyMarginDeci ||
        ABS(attitude.raw[FD_PITCH]) > safetyMarginDeci) {
        return false;
    }
    
    // Check gyro rates are calm (not still spinning)
    for (int axis = 0; axis < 3; axis++) {
        if (fabsf(gyro.gyroADCf[axis]) > 100.0f) {  // Much lower than maneuver threshold
            return false;
        }
    }
    
    return true;
}

// Check if throttle is in a reasonable hover range (for state transitions, not abort)
static bool isThrottleInRange(void)
{
    const int8_t throttlePercent = calculateThrottlePercent();
    return (throttlePercent >= 10 && throttlePercent <= 90);
}

// ============================================================================
// MANEUVER DETECTION
// ============================================================================

// Filter mode tracking - uses hover throttle reference
static struct {
    bool inFilterMode;              // Currently in filter mode collection
    timeUs_t lowThrottleStartTime;  // When throttle dropped below threshold
    bool lowThrottleTimerActive;    // Tracking low throttle duration
} filterTracker;

// Update filter mode exit timer - must be called from DETECTING state to track 2s exit
static void updateFilterModeExitTimer(timeUs_t currentTimeUs)
{
    if (runtime.tuneMode != TUNE_MODE_FILTER) {
        return;
    }
    
    const int8_t throttlePercent = calculateThrottlePercent();
    const int8_t exitThreshold = runtime.hoverThrottle + 10;  // 10% above hover to stay in filter mode
    
    if (throttlePercent <= exitThreshold) {
        if (!filterTracker.lowThrottleTimerActive) {
            // Start timer
            filterTracker.lowThrottleTimerActive = true;
            filterTracker.lowThrottleStartTime = currentTimeUs;
        } else if (cmpTimeUs(currentTimeUs, filterTracker.lowThrottleStartTime) > 2000000) {
            // Been low for 2 seconds - exit filter mode
            filterTracker.inFilterMode = false;
            filterTracker.lowThrottleTimerActive = false;
        }
    } else {
        // Throttle back up - reset timer, stay in filter mode
        filterTracker.lowThrottleTimerActive = false;
    }
}

static autotuneTuneMode_e detectManeuverType(void)
{
    const float rollRate = fabsf(gyro.gyroADCf[FD_ROLL]);
    const float pitchRate = fabsf(gyro.gyroADCf[FD_PITCH]);
    const int8_t throttlePercent = calculateThrottlePercent();
    const timeUs_t now = micros();
    
    // Check for roll maneuver (high roll rate with roll stick input)
    if (rollRate > MANEUVER_ROLL_RATE_THRESHOLD && 
        fabsf(getRcDeflection(FD_ROLL)) > 0.5f) {
        filterTracker.inFilterMode = false;
        return TUNE_MODE_ROLL;
    }
    
    // Check for flip maneuver (high pitch rate with pitch stick input)
    if (pitchRate > MANEUVER_PITCH_RATE_THRESHOLD && 
        fabsf(getRcDeflection(FD_PITCH)) > 0.5f) {
        filterTracker.inFilterMode = false;
        return TUNE_MODE_PITCH;
    }
    
    // =========================================================================
    // FILTER MODE: Throttle-based detection using hover reference
    // =========================================================================
    // Entry: sticks centered + throttle > (hover + 15%)
    // Exit: throttle < (hover + 10%) for 2+ seconds (in ARMED state)
    // NOTE: Only check entry when we're not already committed to filter mode
    
    // Need hover to be calibrated first
    if (!runtime.hoverCalibrated) {
        return TUNE_MODE_NONE;
    }
    
    // If tuneMode is already FILTER, we're committed - only check for exit
    // This prevents re-triggering filter mode on every return to ARMED
    if (runtime.tuneMode == TUNE_MODE_FILTER) {
        // Already in filter mode session - just continue
        return TUNE_MODE_FILTER;
    }
    
    const float maxStickDeflection = MAX(fabsf(getRcDeflection(FD_ROLL)),
                                         MAX(fabsf(getRcDeflection(FD_PITCH)),
                                             fabsf(getRcDeflection(FD_YAW))));
    const bool sticksAreCentered = (maxStickDeflection < 0.5f);
    const int8_t throttleThreshold = runtime.hoverThrottle + 15;  // 15% above hover to enter
    
    if (!filterTracker.inFilterMode) {
        // Not in filter mode - check for entry condition
        // Sticks centered AND throttle above threshold
        if (sticksAreCentered && throttlePercent > throttleThreshold) {
            filterTracker.inFilterMode = true;
            filterTracker.lowThrottleTimerActive = false;
            return TUNE_MODE_FILTER;
        }
    } else {
        // Already in filter mode - check for exit condition
        // Throttle below threshold for 2+ seconds
        
        if (throttlePercent <= throttleThreshold) {
            if (!filterTracker.lowThrottleTimerActive) {
                // Start timer
                filterTracker.lowThrottleTimerActive = true;
                filterTracker.lowThrottleStartTime = now;
            } else if (cmpTimeUs(now, filterTracker.lowThrottleStartTime) > 2000000) {
                // Been low for 2 seconds - exit filter mode, trigger analysis
                filterTracker.inFilterMode = false;
                filterTracker.lowThrottleTimerActive = false;
                // Return NONE to signal maneuver complete (will be handled by state machine)
                return TUNE_MODE_NONE;
            }
        } else {
            // Throttle back up - reset timer
            filterTracker.lowThrottleTimerActive = false;
        }
        
        // Still in filter mode
        return TUNE_MODE_FILTER;
    }
    
    return TUNE_MODE_NONE;
}

static bool isManeuverComplete(void)
{
    // For filter mode: exit is handled by detectManeuverType returning NONE
    // after throttle has been low for 2 seconds
    if (runtime.tuneMode == TUNE_MODE_FILTER) {
        // Filter mode completion is detected when detectManeuverType stops returning FILTER
        // This happens when throttle stays below threshold for 2s
        return !filterTracker.inFilterMode;
    }
    
    // For roll/pitch: complete when rates drop and sticks centered
    const float maxRate = MAX(fabsf(gyro.gyroADCf[FD_ROLL]), 
                              fabsf(gyro.gyroADCf[FD_PITCH]));
    const float maxStick = MAX(fabsf(getRcDeflection(FD_ROLL)),
                               fabsf(getRcDeflection(FD_PITCH)));
    
    return (maxRate < 100.0f && maxStick < 0.2f);
}

static bool isHoverStable(void)
{
    // Check if in stable hover - used for initial hover calibration
    const float maxRate = MAX(fabsf(gyro.gyroADCf[FD_ROLL]), 
                              MAX(fabsf(gyro.gyroADCf[FD_PITCH]),
                                  fabsf(gyro.gyroADCf[FD_YAW])));
    const float maxStick = MAX(fabsf(getRcDeflection(FD_ROLL)),
                               MAX(fabsf(getRcDeflection(FD_PITCH)),
                                   fabsf(getRcDeflection(FD_YAW))));
    const int8_t throttle = calculateThrottlePercent();
    
    // Must have some throttle (not on ground) and low rates/sticks
    bool isStable = (maxRate < HOVER_GYRO_THRESHOLD && 
                     maxStick < HOVER_STICK_THRESHOLD &&
                     throttle > 15 && throttle < 80);  // Reasonable hover range
    
    return isStable;
}

static bool calibrateHover(timeUs_t currentTimeUs)
{
    // Calibrate hover throttle by waiting for stable hover for 1 second
    if (runtime.hoverCalibrated) {
        return true;  // Already calibrated
    }
    
    if (isHoverStable()) {
        if (runtime.hoverStableStartTime == 0) {
            runtime.hoverStableStartTime = currentTimeUs;
        } else if (cmpTimeUs(currentTimeUs, runtime.hoverStableStartTime) > 1000000) {
            // Stable for 1 second - calibrate!
            runtime.hoverThrottle = calculateThrottlePercent();
            runtime.hoverCalibrated = true;
            return true;
        }
    } else {
        runtime.hoverStableStartTime = 0;  // Reset timer
    }
    
    return false;
}

// ============================================================================
// SAMPLE COLLECTION
// ============================================================================

static void collectSample(float gyroRate, float setpoint, float dterm, float throttle)
{
    if (runtime.sampleIndex >= AUTOTUNE_SAMPLE_COUNT) {
        return;
    }
    
    runtime.gyroHistory[runtime.sampleIndex] = gyroRate;
    runtime.setpointHistory[runtime.sampleIndex] = setpoint;
    runtime.dtermHistory[runtime.sampleIndex] = dterm;
    runtime.throttleHistory[runtime.sampleIndex] = throttle;
    runtime.sampleIndex++;
    runtime.sampleCount = runtime.sampleIndex;
}

static void resetSamples(void)
{
    runtime.sampleIndex = 0;
    runtime.sampleCount = 0;
    runtime.sampleCounter = 0;
}

// ============================================================================
// WIGGLE SIGNAL
// ============================================================================

// Track hover calibration wiggle separately from iteration wiggle
static timeUs_t hoverCalibratedWiggleStart = 0;
static bool hoverWiggleActive = false;

static float getWiggleSignal(timeUs_t currentTimeUs)
{
    // Check for hover calibration wiggle (works in ARMED state)
    if (hoverWiggleActive && runtime.state == AUTOTUNE_STATE_ARMED) {
        const timeUs_t elapsed = cmpTimeUs(currentTimeUs, hoverCalibratedWiggleStart);
        const float wigglePeriodUs = 80000;   // 80ms period
        const float wiggleAmplitude = 100.0f;
        
        // 6 wiggle cycles for ~500ms total
        if (elapsed > wigglePeriodUs * 6) {
            hoverWiggleActive = false;  // Done with hover wiggle
            return 0.0f;
        }
        
        float phase = (float)(elapsed % (uint32_t)wigglePeriodUs) / wigglePeriodUs * 2.0f * M_PIf;
        return wiggleAmplitude * sinf(phase);
    }
    
    // Regular iteration wiggle - only in SIGNALING state
    if (runtime.state != AUTOTUNE_STATE_SIGNALING) {
        return 0.0f;
    }
    
    const timeUs_t elapsed = cmpTimeUs(currentTimeUs, runtime.wiggleStartTime);
    const float wigglePeriodUs = 80000;   // 80ms period = 12.5Hz (faster, more noticeable)
    const float wiggleAmplitude = 100.0f; // 100 deg/s (much more noticeable)
    
    // 6 wiggle cycles for ~500ms total
    if (elapsed > wigglePeriodUs * 6) {
        return 0.0f;
    }
    
    // Sine wave wiggle
    float phase = (float)(elapsed % (uint32_t)wigglePeriodUs) / wigglePeriodUs * 2.0f * M_PIf;
    return wiggleAmplitude * sinf(phase);
}

// External access to current PID profile
extern pidProfile_t *currentPidProfile;

// ============================================================================
// GAIN CACHING
// ============================================================================

static void cacheCurrentGains(void)
{
    runtime.currentP = currentPidProfile->pid[runtime.currentAxis].P;
    runtime.currentI = currentPidProfile->pid[runtime.currentAxis].I;
    runtime.currentD = currentPidProfile->pid[runtime.currentAxis].D;
    runtime.currentF = currentPidProfile->pid[runtime.currentAxis].F;
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

static void updateDebugOutput(void)
{
    // [0] = state
    DEBUG_SET(DEBUG_AUTOTUNE, 0, runtime.state);
    
    // [1] = tuneMode | (iteration << 4)
    DEBUG_SET(DEBUG_AUTOTUNE, 1, runtime.tuneMode | (runtime.iteration << 4));
    
    // For filter mode, show filter frequencies instead of PID gains
    if (runtime.tuneMode == TUNE_MODE_FILTER) {
        // [2] = dterm_lpf1 Hz
        DEBUG_SET(DEBUG_AUTOTUNE, 2, currentPidProfile->dterm_lpf1_static_hz);
        // [3] = gyro_lpf1 Hz
        DEBUG_SET(DEBUG_AUTOTUNE, 3, gyroConfig()->gyro_lpf1_static_hz);
        // [5] = noise ratio * 10 (>12 = too noisy, <8 = can relax filters)
        float targetNoise = runtime.noiseFloor > 0 ? runtime.noiseFloor : 1.0f;
        float noiseRatio = runtime.metrics.noiseRms / targetNoise;
        DEBUG_SET(DEBUG_AUTOTUNE, 5, (int16_t)(noiseRatio * 10));
        // [7] = filter noise score * 100 (0 = perfect at target, positive = too noisy, negative = too filtered)
        float filterScore = (noiseRatio - 1.0f) * 100.0f;  // 0 at target, +100 = 2x noise, -50 = half noise
        DEBUG_SET(DEBUG_AUTOTUNE, 7, (int16_t)constrainf(filterScore, -999, 999));
    } else {
        // [2] = P gain (or hover throttle before calibrated)
        if (!runtime.hoverCalibrated) {
            DEBUG_SET(DEBUG_AUTOTUNE, 2, calculateThrottlePercent());  // Show current throttle
        } else {
            DEBUG_SET(DEBUG_AUTOTUNE, 2, runtime.currentP);
        }
        
        // [3] = D gain (or hover reference after calibrated)
        if (runtime.hoverCalibrated && runtime.state == AUTOTUNE_STATE_ARMED) {
            DEBUG_SET(DEBUG_AUTOTUNE, 3, runtime.hoverThrottle);  // Show calibrated hover
        } else {
            DEBUG_SET(DEBUG_AUTOTUNE, 3, runtime.currentD);
        }
        
        // [5] = overshoot * 10
        DEBUG_SET(DEBUG_AUTOTUNE, 5, (int16_t)(runtime.metrics.overshootPercent * 10));
        
        // [7] = score * 100
        float score = autotuneCalculateScore(&runtime.metrics);
        DEBUG_SET(DEBUG_AUTOTUNE, 7, (int16_t)(score * 100));
    }
    
    // [4] = status code
    DEBUG_SET(DEBUG_AUTOTUNE, 4, runtime.status);
    
    // [6] = noise * 10 (or filterTracker state during filter detection)
    if (runtime.tuneMode == TUNE_MODE_FILTER || filterTracker.inFilterMode) {
        // Show filter mode state: 100=in filter mode, 0=not
        DEBUG_SET(DEBUG_AUTOTUNE, 6, filterTracker.inFilterMode ? 100 : 0);
    } else {
        DEBUG_SET(DEBUG_AUTOTUNE, 6, (int16_t)(runtime.metrics.noiseRms * 10));
    }
}

// ============================================================================
// STATE MACHINE
// ============================================================================

void autotuneUpdate(timeUs_t currentTimeUs)
{
    // Check activation
    const bool shouldBeActive = IS_RC_MODE_ACTIVE(BOXAUTOTUNE) && 
                                autotuneConfig()->autotune_enabled &&
                                ARMING_FLAG(ARMED);
    
    // Activation logic
    if (shouldBeActive && runtime.state == AUTOTUNE_STATE_IDLE) {
        changeState(AUTOTUNE_STATE_ARMED, currentTimeUs);
        runtime.tuneMode = TUNE_MODE_NONE;
        runtime.iteration = 0;
        runtime.historyCount = 0;
        runtime.bestScore = 1000.0f;
        runtime.status = STATUS_WAITING_MANEUVER;
        beeper(BEEPER_AUTOTUNE_START);
    } else if (!shouldBeActive && runtime.state != AUTOTUNE_STATE_IDLE) {
        changeState(AUTOTUNE_STATE_IDLE, currentTimeUs);
        runtime.status = STATUS_OK;
    }
    
    // No automatic abort - safety checks used for state transition gating instead
    // User can abort by disabling autotune mode switch
    
    // State machine
    switch (runtime.state) {
        case AUTOTUNE_STATE_IDLE:
            // Nothing to do
            break;
            
        case AUTOTUNE_STATE_ARMED:
            {
                // First, calibrate hover if not done yet
                if (!runtime.hoverCalibrated) {
                    runtime.status = STATUS_WAITING_MANEUVER;  // "Waiting" = calibrating hover
                    if (calibrateHover(currentTimeUs)) {
                        // Hover calibrated! Give wiggle to indicate ready
                        beeper(BEEPER_READY_BEEP);
                        hoverCalibratedWiggleStart = currentTimeUs;
                        hoverWiggleActive = true;
                    }
                    break;
                }
                
                runtime.status = STATUS_WAITING_MANEUVER;
                
                // Check if we should exit filter mode session (throttle low for 2s)
                // This happens when returning to ARMED after a filter iteration
                if (runtime.tuneMode == TUNE_MODE_FILTER) {
                    const int8_t throttlePercent = calculateThrottlePercent();
                    const int8_t exitThreshold = runtime.hoverThrottle + 10;  // 10% above hover to stay in filter mode
                    
                    if (throttlePercent <= exitThreshold) {
                        if (!filterTracker.lowThrottleTimerActive) {
                            filterTracker.lowThrottleTimerActive = true;
                            filterTracker.lowThrottleStartTime = currentTimeUs;
                        } else if (cmpTimeUs(currentTimeUs, filterTracker.lowThrottleStartTime) > 2000000) {
                            // Been low for 2 seconds - exit filter mode session
                            runtime.tuneMode = TUNE_MODE_NONE;
                            filterTracker.inFilterMode = false;
                            filterTracker.lowThrottleTimerActive = false;
                            beeper(BEEPER_READY_BEEP);  // Signal filter mode ended
                        }
                    } else {
                        // Throttle back up - reset timer, continue filter mode
                        filterTracker.lowThrottleTimerActive = false;
                    }
                }
                
                // Detect maneuver start
                autotuneTuneMode_e detected = detectManeuverType();
                if (detected != TUNE_MODE_NONE) {
                    runtime.tuneMode = detected;
                    runtime.currentAxis = (detected == TUNE_MODE_ROLL) ? FD_ROLL : 
                                          (detected == TUNE_MODE_PITCH) ? FD_PITCH : FD_ROLL;
                    runtime.maneuverStartTime = currentTimeUs;
                    runtime.maneuverActive = true;
                    runtime.peakRate = 0.0f;
                    
                    // Cache current gains
                    cacheCurrentGains();
                    
                    changeState(AUTOTUNE_STATE_DETECTING, currentTimeUs);
                    resetSamples();
                    runtime.dataValid = true;  // Assume data is valid until proven otherwise
                    runtime.status = STATUS_MANEUVER_DETECTED;
                }
            }
            break;
            
        case AUTOTUNE_STATE_DETECTING:
            runtime.status = STATUS_MANEUVER_DETECTED;
            {
                // For filter mode, update the exit timer (tracks 2s low throttle)
                if (runtime.tuneMode == TUNE_MODE_FILTER) {
                    updateFilterModeExitTimer(currentTimeUs);
                }
                
                // Track peak rate during maneuver
                float currentRate = fabsf(gyro.gyroADCf[runtime.currentAxis]);
                if (currentRate > runtime.peakRate) {
                    runtime.peakRate = currentRate;
                }
                
                // Collect samples during maneuver
                runtime.sampleCounter++;
                if (runtime.sampleCounter >= runtime.sampleIntervalLoops) {
                    runtime.sampleCounter = 0;
                    
                    float setpoint = (runtime.currentAxis == FD_ROLL) ? 
                                     getRcDeflection(FD_ROLL) * 500.0f :  // Approximate rate
                                     getRcDeflection(FD_PITCH) * 500.0f;
                    
                    collectSample(
                        gyro.gyroADCf[runtime.currentAxis],
                        setpoint,
                        0.0f,  // D-term would need PID internals
                        (float)calculateThrottlePercent() / 100.0f
                    );
                }
                
                // Check if maneuver complete
                if (isManeuverComplete()) {
                    runtime.maneuverEndTime = currentTimeUs;
                    runtime.maneuverActive = false;
                    changeState(AUTOTUNE_STATE_COLLECTING, currentTimeUs);
                    runtime.status = STATUS_COLLECTING_DATA;
                }
                
                // Timeout - maneuver too long (skip for filter mode - uses 2s low-throttle exit timer instead)
                if (runtime.tuneMode != TUNE_MODE_FILTER && cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 3000000) {
                    changeState(AUTOTUNE_STATE_SETTLING, currentTimeUs);
                }
            }
            break;
            
        case AUTOTUNE_STATE_COLLECTING:
            runtime.status = STATUS_COLLECTING_DATA;
            {
                // For filter mode, continue updating the exit timer
                if (runtime.tuneMode == TUNE_MODE_FILTER) {
                    updateFilterModeExitTimer(currentTimeUs);
                }
                
                // Only collect settling data when attitude is reasonable
                // (pilot may still be recovering from roll/flip)
                if (isReadyForSettle() && isThrottleInRange()) {
                    // Continue collecting settling data
                    runtime.sampleCounter++;
                    if (runtime.sampleCounter >= runtime.sampleIntervalLoops) {
                        runtime.sampleCounter = 0;
                        collectSample(
                            gyro.gyroADCf[runtime.currentAxis],
                            0.0f,  // No setpoint during settling
                            0.0f,
                            (float)calculateThrottlePercent() / 100.0f
                        );
                    }
                    
                    // Collect for 500ms of good data or until buffer full
                    if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 500000 ||
                        runtime.sampleIndex >= AUTOTUNE_SAMPLE_COUNT) {
                        changeState(AUTOTUNE_STATE_SETTLING, currentTimeUs);
                    }
                }
                
                // Long timeout - pilot taking too long to recover
                if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 10000000) {
                    // Go to settling anyway with whatever data we have
                    changeState(AUTOTUNE_STATE_SETTLING, currentTimeUs);
                }
            }
            break;
            
        case AUTOTUNE_STATE_SETTLING:
            runtime.status = STATUS_WAITING_SETTLE;
            {
                // Wait for stable hover
                if (isHoverStable()) {
                    changeState(AUTOTUNE_STATE_ANALYZING, currentTimeUs);
                }
                
                // Timeout
                if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 5000000) {
                    // Proceed anyway
                    changeState(AUTOTUNE_STATE_ANALYZING, currentTimeUs);
                }
            }
            break;
            
        case AUTOTUNE_STATE_ANALYZING:
            runtime.status = STATUS_ANALYZING;
            {
                if (runtime.stateJustEntered) {
                    runtime.stateJustEntered = false;
                    
                    if (runtime.tuneMode == TUNE_MODE_FILTER) {
                        // Filter tuning - analyze noise
                        autotuneAnalyzeNoise(
                            runtime.gyroHistory,
                            runtime.throttleHistory,
                            runtime.sampleCount,
                            AUTOTUNE_TARGET_SAMPLE_RATE_HZ,
                            &runtime.filterAnalysis
                        );
                    } else {
                        // PID tuning - analyze response
                        autotuneAnalyzeResponse(
                            runtime.gyroHistory,
                            runtime.setpointHistory,
                            runtime.dtermHistory,
                            runtime.sampleCount,
                            AUTOTUNE_TARGET_SAMPLE_RATE_HZ,
                            &runtime.metrics
                        );
                        
                        // Classify response
                        runtime.responseClass = autotuneClassifyResponse(&runtime.metrics);
                        
                        // For now, trust the data - crash detection was too aggressive
                        // TODO: Detect actual crashes using D-term spikes, not gyro RMS
                        runtime.dataValid = true;
                        
                        if (runtime.dataValid) {
                            // Attribute gains only if data is good
                            autotuneAttributeGains(
                                &runtime.metrics,
                                runtime.responseClass,
                                &runtime.attribution
                            );
                            
                            // Save to history
                            float score = autotuneCalculateScore(&runtime.metrics);
                            autotuneSaveToHistory(&runtime, score);
                        }
                    }
                }
                
                // Analysis complete after 100ms
                if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 100000) {
                    changeState(AUTOTUNE_STATE_ADJUSTING, currentTimeUs);
                }
            }
            break;
            
        case AUTOTUNE_STATE_ADJUSTING:
            {
                if (runtime.stateJustEntered) {
                    runtime.stateJustEntered = false;
                    
                    // Skip gain adjustment if data was bad (crash/invalid data detected)
                    if (!runtime.dataValid) {
                        runtime.status = STATUS_ABORT_GYRO;  // Reuse this status to indicate bad data
                        // Don't apply gains, don't increment iteration - just signal and try again
                    } else if (runtime.tuneMode == TUNE_MODE_FILTER) {
                        // Apply filter adjustments
                        runtime.status = STATUS_ADJUSTING_FILTER;
                        autotuneApplyFilterAdjustment(
                            &runtime.filterAnalysis,
                            runtime.filterAnalysis.noiseFloor,
                            NOISE_TARGET_RMS
                        );
                        runtime.iteration++;
                    } else {
                        // Apply gain adjustments
                        switch (runtime.attribution.primary) {
                            case GAIN_ATTRIBUTION_P:
                                runtime.status = STATUS_ADJUSTING_P;
                                break;
                            case GAIN_ATTRIBUTION_D:
                                runtime.status = STATUS_ADJUSTING_D;
                                break;
                            case GAIN_ATTRIBUTION_I:
                                runtime.status = STATUS_ADJUSTING_I;
                                break;
                            case GAIN_ATTRIBUTION_F:
                                runtime.status = STATUS_ADJUSTING_F;
                                break;
                            default:
                                runtime.status = STATUS_ADJUSTING_P;
                                break;
                        }
                        
                        autotuneApplyGainAdjustment(
                            &runtime,
                            &runtime.attribution,
                            runtime.responseClass
                        );
                        runtime.iteration++;
                    }
                }
                
                // Check for convergence (only if we have valid data this round)
                if (runtime.dataValid && autotuneCheckConvergence(&runtime)) {
                    autotuneRestoreBestGains(&runtime);
                    changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
                    beeper(BEEPER_AUTOTUNE_DONE);
                    runtime.status = STATUS_COMPLETE;
                } else if (runtime.iteration >= autotuneConfig()->max_iterations) {
                    autotuneRestoreBestGains(&runtime);
                    changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
                    beeper(BEEPER_AUTOTUNE_DONE);
                    runtime.status = STATUS_COMPLETE;
                } else if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 100000) {
                    // Signal ready for next maneuver
                    changeState(AUTOTUNE_STATE_SIGNALING, currentTimeUs);
                    runtime.wiggleStartTime = currentTimeUs;
                    runtime.wigglePhase = 0;
                }
            }
            break;
            
        case AUTOTUNE_STATE_SIGNALING:
            runtime.status = STATUS_SIGNAL_READY;
            {
                // Wiggle for 500ms then return to armed
                if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 500000) {
                    changeState(AUTOTUNE_STATE_ARMED, currentTimeUs);
                    beeper(BEEPER_READY_BEEP);
                }
            }
            break;
            
        case AUTOTUNE_STATE_COMPLETE:
            runtime.status = STATUS_COMPLETE;
            // Stay here until switch turned off
            break;
            
        case AUTOTUNE_STATE_ABORTED:
            // Stay here until switch turned off
            break;
            
        default:
            break;
    }
    
    updateDebugOutput();
}

// ============================================================================
// PUBLIC API
// ============================================================================

bool autotuneIsActive(void)
{
    return (runtime.state != AUTOTUNE_STATE_IDLE);
}

autotuneState_e autotuneGetState(void)
{
    return runtime.state;
}

const char* autotuneGetStateName(void)
{
    if (runtime.state < AUTOTUNE_STATE_COUNT) {
        return stateNames[runtime.state];
    }
    return "UNKNOWN";
}

const char* autotuneGetModeName(void)
{
    if (runtime.tuneMode <= TUNE_MODE_FILTER) {
        return modeNames[runtime.tuneMode];
    }
    return "UNKNOWN";
}

// Modify setpoint to add wiggle signal during signaling state
float autotuneModifySetpoint(uint8_t axis, float setpoint, timeUs_t currentTimeUs)
{
    // Check for hover calibration wiggle (ARMED state, roll axis)
    if (hoverWiggleActive && runtime.state == AUTOTUNE_STATE_ARMED && axis == FD_ROLL) {
        return setpoint + getWiggleSignal(currentTimeUs);
    }
    
    // Check for iteration complete wiggle (SIGNALING state)
    if (runtime.state != AUTOTUNE_STATE_SIGNALING) {
        return setpoint;
    }
    
    // Only wiggle the current axis being tuned
    if (axis != runtime.currentAxis) {
        return setpoint;
    }
    
    return setpoint + getWiggleSignal(currentTimeUs);
}

#endif // USE_AUTOTUNE
