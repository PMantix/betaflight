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
#include "flight/pid_init.h"

#include "pg/autotune.h"

#include "config/config.h"  // For writeEEPROM()

#include "sensors/gyro.h"
#include "sensors/gyro_init.h"  // For gyroInitFilters()

#include "flight/mixer.h"  // For motor[] array

#include "autotune.h"
#include "autotune_types.h"
#include "autotune_analysis.h"
#include "autotune_gains.h"

// External access to current PID profile
extern pidProfile_t *currentPidProfile;

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
    runtime.lastReasonCode = REASON_IDLE;  // Initialize to IDLE reason
    
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
// HOVER-BASED MOTOR RMS TUNING (Phase 0)
// ============================================================================

// Motor RMS target - lower is better tuned filters/PIDs
// These values are motor command standard deviation (0-1000 scale)
#define MOTOR_RMS_TARGET        15.0f   // Target: below 15 is good
#define MOTOR_RMS_EXCELLENT      8.0f   // Excellent: can try relaxing filters
#define MOTOR_RMS_WINDOW_US     500000  // 500ms measurement window
#define MOTOR_RMS_MAX_ITER      10      // Max hover tune iterations

// Motor command tracking for time-based RMS calculation
// We track sum and sum-of-squares to compute variance: var = E[X^2] - E[X]^2
static struct {
    float motorSum[MAX_SUPPORTED_MOTORS];      // Sum of motor values
    float motorSumSq[MAX_SUPPORTED_MOTORS];    // Sum of squared motor values
    uint32_t sampleCount;
} motorStats;

// Per-component noise tracking for intelligent attribution
// Tracks gyro, P-term, and D-term RMS to identify noise source
static struct {
    float gyroSumSq[3];    // Sum of squared gyro values (roll, pitch, yaw)
    float pTermSumSq[3];   // Sum of squared P-term values (roll, pitch, yaw)
    float dTermSumSq[2];   // Sum of squared D-term values (roll, pitch only)
    // Computed RMS values after window
    float gyroRms;         // Average gyro RMS across axes
    float pTermRms;        // Average P-term RMS across axes
    float dTermRms;        // Average D-term RMS (roll+pitch)
} noiseStats;

// Accumulate motor samples for variance calculation
// Also accumulates gyro, P-term, D-term for noise attribution
static void accumulateMotorRms(void)
{
    // Accumulate motor commands
    uint8_t motorCount = getMotorCount();
    for (int i = 0; i < motorCount; i++) {
        float val = motor[i];
        motorStats.motorSum[i] += val;
        motorStats.motorSumSq[i] += val * val;
    }
    motorStats.sampleCount++;
    
    // Accumulate gyro, P-term, D-term for noise attribution
    for (int axis = 0; axis <= FD_YAW; axis++) {
        float gyroVal = gyro.gyroADCf[axis];
        noiseStats.gyroSumSq[axis] += gyroVal * gyroVal;
        
        float pVal = pidData[axis].P;
        noiseStats.pTermSumSq[axis] += pVal * pVal;
        
        // D-term only for roll and pitch
        if (axis <= FD_PITCH) {
            float dVal = pidData[axis].D;
            noiseStats.dTermSumSq[axis] += dVal * dVal;
        }
    }
}

// Calculate motor command RMS (average standard deviation across motors)
// Also computes gyro, P-term, D-term RMS for noise attribution
static float computeWindowedMotorRms(void)
{
    if (motorStats.sampleCount < 2) return 0.0f;
    
    // Motor command std deviation
    uint8_t motorCount = getMotorCount();
    float totalStd = 0.0f;
    
    for (int i = 0; i < motorCount; i++) {
        float mean = motorStats.motorSum[i] / motorStats.sampleCount;
        float meanSq = motorStats.motorSumSq[i] / motorStats.sampleCount;
        float variance = meanSq - mean * mean;
        if (variance > 0) {
            totalStd += sqrtf(variance);
        }
    }
    
    // Compute per-component RMS values for noise attribution
    // These are root-mean-square (sqrt of mean of squared values)
    float gyroRmsSum = 0.0f;
    float pRmsSum = 0.0f;
    float dRmsSum = 0.0f;
    
    for (int axis = 0; axis <= FD_YAW; axis++) {
        float gyroMeanSq = noiseStats.gyroSumSq[axis] / motorStats.sampleCount;
        gyroRmsSum += sqrtf(gyroMeanSq);
        
        float pMeanSq = noiseStats.pTermSumSq[axis] / motorStats.sampleCount;
        pRmsSum += sqrtf(pMeanSq);
        
        if (axis <= FD_PITCH) {
            float dMeanSq = noiseStats.dTermSumSq[axis] / motorStats.sampleCount;
            dRmsSum += sqrtf(dMeanSq);
        }
    }
    
    noiseStats.gyroRms = gyroRmsSum / 3.0f;   // Average across 3 axes
    noiseStats.pTermRms = pRmsSum / 3.0f;     // Average across 3 axes
    noiseStats.dTermRms = dRmsSum / 2.0f;     // Average across roll+pitch
    
    return totalStd / motorCount;  // Average std across all motors
}

// Reset motor RMS accumulator and noise stats
static void resetMotorRmsAccum(void)
{
    uint8_t motorCount = getMotorCount();
    for (int i = 0; i < motorCount; i++) {
        motorStats.motorSum[i] = 0.0f;
        motorStats.motorSumSq[i] = 0.0f;
    }
    motorStats.sampleCount = 0;
    
    // Reset noise attribution stats
    for (int axis = 0; axis <= FD_YAW; axis++) {
        noiseStats.gyroSumSq[axis] = 0.0f;
        noiseStats.pTermSumSq[axis] = 0.0f;
        if (axis <= FD_PITCH) {
            noiseStats.dTermSumSq[axis] = 0.0f;
        }
    }
}

// ============================================================================
// PROCEDURAL DIAGNOSTIC HOVER TUNE
// ============================================================================
// Replaces heuristic noise attribution with controlled A/B testing.
// Each diagnostic cycle:
//   1. Measure baseline (500ms)
//   2. Halve Roll gains, measure (500ms), restore
//   3. Halve Pitch gains, measure (500ms), restore  
//   4. Lower Gyro LPF1 by 50Hz, measure (500ms), restore
//   5. Lower Dterm LPF1 by 50Hz, measure (500ms), restore
//   6. Analyze: identify dominant contributor (highest improvement %)
//   7. Apply permanent fix to dominant contributor only
//   8. Repeat until motor RMS <= target OR no improvement OR max iterations

// Save current settings before diagnostic tests
static void saveDiagnosticSettings(void)
{
    runtime.savedSettings.rollP = currentPidProfile->pid[FD_ROLL].P;
    runtime.savedSettings.rollI = currentPidProfile->pid[FD_ROLL].I;
    runtime.savedSettings.rollD = currentPidProfile->pid[FD_ROLL].D;
    runtime.savedSettings.rollF = currentPidProfile->pid[FD_ROLL].F;
    
    runtime.savedSettings.pitchP = currentPidProfile->pid[FD_PITCH].P;
    runtime.savedSettings.pitchI = currentPidProfile->pid[FD_PITCH].I;
    runtime.savedSettings.pitchD = currentPidProfile->pid[FD_PITCH].D;
    runtime.savedSettings.pitchF = currentPidProfile->pid[FD_PITCH].F;
    
    // Check if LPF1 filters are in dynamic mode
    runtime.savedSettings.dtermLpf1IsDynamic = (currentPidProfile->dterm_lpf1_static_hz == 0) && 
                                                (currentPidProfile->dterm_lpf1_dyn_min_hz > 0);
    runtime.savedSettings.gyroLpf1IsDynamic = (gyroConfig()->gyro_lpf1_static_hz == 0) && 
                                               (gyroConfig()->gyro_lpf1_dyn_min_hz > 0);
    
    // Save current LPF1 values (dynamic min or static)
    runtime.savedSettings.dtermLpf1Hz = runtime.savedSettings.dtermLpf1IsDynamic ? 
        currentPidProfile->dterm_lpf1_dyn_min_hz : currentPidProfile->dterm_lpf1_static_hz;
    runtime.savedSettings.gyroLpf1Hz = runtime.savedSettings.gyroLpf1IsDynamic ? 
        gyroConfig()->gyro_lpf1_dyn_min_hz : gyroConfig()->gyro_lpf1_static_hz;
}

// Restore settings after a diagnostic test
static void restoreDiagnosticSettings(void)
{
    // Restore Roll gains
    currentPidProfile->pid[FD_ROLL].P = runtime.savedSettings.rollP;
    currentPidProfile->pid[FD_ROLL].I = runtime.savedSettings.rollI;
    currentPidProfile->pid[FD_ROLL].D = runtime.savedSettings.rollD;
    currentPidProfile->pid[FD_ROLL].F = runtime.savedSettings.rollF;
    
    // Restore Pitch gains
    currentPidProfile->pid[FD_PITCH].P = runtime.savedSettings.pitchP;
    currentPidProfile->pid[FD_PITCH].I = runtime.savedSettings.pitchI;
    currentPidProfile->pid[FD_PITCH].D = runtime.savedSettings.pitchD;
    currentPidProfile->pid[FD_PITCH].F = runtime.savedSettings.pitchF;
    
    // Restore Dterm LPF1
    if (runtime.savedSettings.dtermLpf1IsDynamic) {
        currentPidProfile->dterm_lpf1_dyn_min_hz = runtime.savedSettings.dtermLpf1Hz;
        pidRuntime.dynLpfMin = runtime.savedSettings.dtermLpf1Hz;
    } else {
        currentPidProfile->dterm_lpf1_static_hz = runtime.savedSettings.dtermLpf1Hz;
    }
    
    // Restore Gyro LPF1
    if (runtime.savedSettings.gyroLpf1IsDynamic) {
        gyroConfigMutable()->gyro_lpf1_dyn_min_hz = runtime.savedSettings.gyroLpf1Hz;
        gyro.dynLpfMin = runtime.savedSettings.gyroLpf1Hz;
    } else {
        gyroConfigMutable()->gyro_lpf1_static_hz = runtime.savedSettings.gyroLpf1Hz;
    }
    
    // Reinitialize with restored settings
    pidInitConfig(currentPidProfile);
}

// Apply the test modification for a given diagnostic phase
static void applyDiagnosticTest(hoverDiagPhase_e phase)
{
    switch (phase) {
        case HOVER_DIAG_ROLL_TEST:
            // Halve Roll gains (P, I, D, F)
            currentPidProfile->pid[FD_ROLL].P = runtime.savedSettings.rollP / 2;
            currentPidProfile->pid[FD_ROLL].I = runtime.savedSettings.rollI / 2;
            currentPidProfile->pid[FD_ROLL].D = runtime.savedSettings.rollD / 2;
            currentPidProfile->pid[FD_ROLL].F = runtime.savedSettings.rollF / 2;
            pidInitConfig(currentPidProfile);
            break;
            
        case HOVER_DIAG_PITCH_TEST:
            // Halve Pitch gains (P, I, D, F)
            currentPidProfile->pid[FD_PITCH].P = runtime.savedSettings.pitchP / 2;
            currentPidProfile->pid[FD_PITCH].I = runtime.savedSettings.pitchI / 2;
            currentPidProfile->pid[FD_PITCH].D = runtime.savedSettings.pitchD / 2;
            currentPidProfile->pid[FD_PITCH].F = runtime.savedSettings.pitchF / 2;
            pidInitConfig(currentPidProfile);
            break;
            
        case HOVER_DIAG_GYRO_LPF1_TEST:
            // Lower Gyro LPF1 by 50Hz (but not below minimum)
            {
                uint16_t newHz = (runtime.savedSettings.gyroLpf1Hz > DIAG_FILTER_STEP_HZ + GYRO_LPF1_MIN_HZ) ?
                    runtime.savedSettings.gyroLpf1Hz - DIAG_FILTER_STEP_HZ : GYRO_LPF1_MIN_HZ;
                if (runtime.savedSettings.gyroLpf1IsDynamic) {
                    gyroConfigMutable()->gyro_lpf1_dyn_min_hz = newHz;
                    gyro.dynLpfMin = newHz;
                } else {
                    gyroConfigMutable()->gyro_lpf1_static_hz = newHz;
                    gyroInitFilters();
                }
            }
            break;
            
        case HOVER_DIAG_DTERM_LPF1_TEST:
            // Lower Dterm LPF1 by 50Hz (but not below minimum)
            {
                uint16_t newHz = (runtime.savedSettings.dtermLpf1Hz > DIAG_FILTER_STEP_HZ + DTERM_LPF1_MIN_HZ) ?
                    runtime.savedSettings.dtermLpf1Hz - DIAG_FILTER_STEP_HZ : DTERM_LPF1_MIN_HZ;
                if (runtime.savedSettings.dtermLpf1IsDynamic) {
                    currentPidProfile->dterm_lpf1_dyn_min_hz = newHz;
                    pidRuntime.dynLpfMin = newHz;
                } else {
                    currentPidProfile->dterm_lpf1_static_hz = newHz;
                    pidInitFilters(currentPidProfile);
                }
            }
            break;
            
        default:
            // BASELINE and others: no modification needed
            break;
    }
}

// Apply permanent fix for the dominant contributor
static void applyDiagnosticFix(hoverDiagPhase_e dominantPhase)
{
    switch (dominantPhase) {
        case HOVER_DIAG_ROLL_TEST:
            // Permanently halve Roll gains
            runtime.lastReasonCode = REASON_DIAG_FIX_ROLL;
            currentPidProfile->pid[FD_ROLL].P = runtime.savedSettings.rollP / 2;
            currentPidProfile->pid[FD_ROLL].I = runtime.savedSettings.rollI / 2;
            currentPidProfile->pid[FD_ROLL].D = runtime.savedSettings.rollD / 2;
            currentPidProfile->pid[FD_ROLL].F = runtime.savedSettings.rollF / 2;
            pidInitConfig(currentPidProfile);
            break;
            
        case HOVER_DIAG_PITCH_TEST:
            // Permanently halve Pitch gains
            runtime.lastReasonCode = REASON_DIAG_FIX_PITCH;
            currentPidProfile->pid[FD_PITCH].P = runtime.savedSettings.pitchP / 2;
            currentPidProfile->pid[FD_PITCH].I = runtime.savedSettings.pitchI / 2;
            currentPidProfile->pid[FD_PITCH].D = runtime.savedSettings.pitchD / 2;
            currentPidProfile->pid[FD_PITCH].F = runtime.savedSettings.pitchF / 2;
            pidInitConfig(currentPidProfile);
            break;
            
        case HOVER_DIAG_GYRO_LPF1_TEST:
            // Permanently lower Gyro LPF1 by 50Hz
            runtime.lastReasonCode = REASON_DIAG_FIX_GYRO_LPF1;
            {
                uint16_t newHz = (runtime.savedSettings.gyroLpf1Hz > DIAG_FILTER_STEP_HZ + GYRO_LPF1_MIN_HZ) ?
                    runtime.savedSettings.gyroLpf1Hz - DIAG_FILTER_STEP_HZ : GYRO_LPF1_MIN_HZ;
                if (runtime.savedSettings.gyroLpf1IsDynamic) {
                    gyroConfigMutable()->gyro_lpf1_dyn_min_hz = newHz;
                    gyro.dynLpfMin = newHz;
                } else {
                    gyroConfigMutable()->gyro_lpf1_static_hz = newHz;
                    gyroInitFilters();
                }
            }
            break;
            
        case HOVER_DIAG_DTERM_LPF1_TEST:
            // Permanently lower Dterm LPF1 by 50Hz
            runtime.lastReasonCode = REASON_DIAG_FIX_DTERM_LPF1;
            {
                uint16_t newHz = (runtime.savedSettings.dtermLpf1Hz > DIAG_FILTER_STEP_HZ + DTERM_LPF1_MIN_HZ) ?
                    runtime.savedSettings.dtermLpf1Hz - DIAG_FILTER_STEP_HZ : DTERM_LPF1_MIN_HZ;
                if (runtime.savedSettings.dtermLpf1IsDynamic) {
                    currentPidProfile->dterm_lpf1_dyn_min_hz = newHz;
                    pidRuntime.dynLpfMin = newHz;
                } else {
                    currentPidProfile->dterm_lpf1_static_hz = newHz;
                    pidInitFilters(currentPidProfile);
                }
            }
            break;
            
        default:
            break;
    }
}

// Procedural diagnostic hover tune
// Returns true when tuning is complete (target reached, no improvement, or max iterations)
static bool updateHoverDiagnostic(timeUs_t currentTimeUs)
{
    // Accumulate motor samples
    accumulateMotorRms();
    
    // Check if 500ms window complete
    if (cmpTimeUs(currentTimeUs, runtime.lastMotorRmsTime) < MOTOR_RMS_WINDOW_US) {
        // Still measuring - show current phase in reason code
        switch (runtime.diagPhase) {
            case HOVER_DIAG_BASELINE:     runtime.lastReasonCode = REASON_DIAG_BASELINE; break;
            case HOVER_DIAG_ROLL_TEST:    runtime.lastReasonCode = REASON_DIAG_ROLL_TEST; break;
            case HOVER_DIAG_PITCH_TEST:   runtime.lastReasonCode = REASON_DIAG_PITCH_TEST; break;
            case HOVER_DIAG_GYRO_LPF1_TEST: runtime.lastReasonCode = REASON_DIAG_GYRO_LPF1_TEST; break;
            case HOVER_DIAG_DTERM_LPF1_TEST: runtime.lastReasonCode = REASON_DIAG_DTERM_LPF1_TEST; break;
            default: break;
        }
        return false;
    }
    
    // Window complete - compute RMS and record for current phase
    float currentRms = computeWindowedMotorRms();
    resetMotorRmsAccum();
    runtime.lastMotorRmsTime = currentTimeUs;
    runtime.lastMotorRms = currentRms;
    
    // Record RMS for current phase
    runtime.diagRms[runtime.diagPhase] = currentRms;
    
    // State machine for diagnostic phases
    switch (runtime.diagPhase) {
        case HOVER_DIAG_BASELINE:
            // Baseline complete - check if already at target
            if (currentRms <= MOTOR_RMS_TARGET) {
                runtime.lastReasonCode = REASON_DIAG_TARGET_REACHED;
                runtime.diagPhase = HOVER_DIAG_COMPLETE;
                return true;
            }
            // Save settings and start Roll test
            saveDiagnosticSettings();
            applyDiagnosticTest(HOVER_DIAG_ROLL_TEST);
            runtime.diagPhase = HOVER_DIAG_ROLL_TEST;
            break;
            
        case HOVER_DIAG_ROLL_TEST:
            // Roll test complete - restore and start Pitch test
            restoreDiagnosticSettings();
            applyDiagnosticTest(HOVER_DIAG_PITCH_TEST);
            runtime.diagPhase = HOVER_DIAG_PITCH_TEST;
            break;
            
        case HOVER_DIAG_PITCH_TEST:
            // Pitch test complete - restore and start Gyro LPF1 test
            restoreDiagnosticSettings();
            applyDiagnosticTest(HOVER_DIAG_GYRO_LPF1_TEST);
            runtime.diagPhase = HOVER_DIAG_GYRO_LPF1_TEST;
            break;
            
        case HOVER_DIAG_GYRO_LPF1_TEST:
            // Gyro LPF1 test complete - restore and start Dterm LPF1 test
            restoreDiagnosticSettings();
            applyDiagnosticTest(HOVER_DIAG_DTERM_LPF1_TEST);
            runtime.diagPhase = HOVER_DIAG_DTERM_LPF1_TEST;
            break;
            
        case HOVER_DIAG_DTERM_LPF1_TEST:
            // All tests complete - restore and analyze
            restoreDiagnosticSettings();
            runtime.diagPhase = HOVER_DIAG_ANALYZING;
            // Fall through to analysis immediately
            // fallthrough
            
        case HOVER_DIAG_ANALYZING:
        {
            runtime.lastReasonCode = REASON_DIAG_ANALYZING;
            
            // Calculate improvement percentages for each test
            float baselineRms = runtime.diagRms[HOVER_DIAG_BASELINE];
            float safeBase = (baselineRms > 0.1f) ? baselineRms : 0.1f;
            
            // Improvement = (baseline - test) / baseline * 100
            // Positive = test was better (lower RMS)
            runtime.diagImprovement[HOVER_DIAG_ROLL_TEST] = 
                (safeBase - runtime.diagRms[HOVER_DIAG_ROLL_TEST]) / safeBase * 100.0f;
            runtime.diagImprovement[HOVER_DIAG_PITCH_TEST] = 
                (safeBase - runtime.diagRms[HOVER_DIAG_PITCH_TEST]) / safeBase * 100.0f;
            runtime.diagImprovement[HOVER_DIAG_GYRO_LPF1_TEST] = 
                (safeBase - runtime.diagRms[HOVER_DIAG_GYRO_LPF1_TEST]) / safeBase * 100.0f;
            runtime.diagImprovement[HOVER_DIAG_DTERM_LPF1_TEST] = 
                (safeBase - runtime.diagRms[HOVER_DIAG_DTERM_LPF1_TEST]) / safeBase * 100.0f;
            
            // Find dominant contributor (highest improvement %)
            float maxImprovement = 0.0f;
            hoverDiagPhase_e dominant = HOVER_DIAG_IDLE;
            
            for (int i = HOVER_DIAG_ROLL_TEST; i <= HOVER_DIAG_DTERM_LPF1_TEST; i++) {
                if (runtime.diagImprovement[i] > maxImprovement) {
                    maxImprovement = runtime.diagImprovement[i];
                    dominant = (hoverDiagPhase_e)i;
                }
            }
            
            // Apply fix if dominant contributor showed >10% improvement
            if (maxImprovement >= DIAG_IMPROVEMENT_THRESHOLD && dominant != HOVER_DIAG_IDLE) {
                applyDiagnosticFix(dominant);
                runtime.diagIteration++;
                
                // Check max iterations
                if (runtime.diagIteration >= DIAG_MAX_ITERATIONS) {
                    runtime.lastReasonCode = REASON_DIAG_MAX_ITERATIONS;
                    runtime.diagPhase = HOVER_DIAG_COMPLETE;
                    return true;
                }
                
                // Restart diagnostic cycle to verify fix
                runtime.diagPhase = HOVER_DIAG_BASELINE;
                // Clear RMS array for next cycle
                for (int i = 0; i < HOVER_DIAG_PHASE_COUNT; i++) {
                    runtime.diagRms[i] = 0.0f;
                    runtime.diagImprovement[i] = 0.0f;
                }
            } else {
                // No significant improvement found
                runtime.lastReasonCode = REASON_DIAG_NO_IMPROVEMENT;
                runtime.diagPhase = HOVER_DIAG_COMPLETE;
                return true;
            }
            break;
        }
            
        case HOVER_DIAG_COMPLETE:
            return true;
            
        default:
            // Start with baseline if in unknown state
            runtime.diagPhase = HOVER_DIAG_BASELINE;
            break;
    }
    
    return false;
}

// ============================================================================
// MANEUVER DETECTION
// ============================================================================

// Filter mode tracking - uses hover throttle reference
static struct {
    bool inFilterMode;              // Currently in filter mode collection
    timeUs_t lowThrottleStartTime;  // When throttle dropped below threshold
    bool lowThrottleTimerActive;    // Tracking low throttle duration
    timeUs_t filterModeEntryTime;   // When filter mode was first entered (for grace period)
} filterTracker;

// Grace period: during first 500ms of filter mode, if sticks move, switch to roll/pitch
#define FILTER_MODE_GRACE_PERIOD_US  500000

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
    
    const float maxStickDeflection = MAX(fabsf(getRcDeflection(FD_ROLL)),
                                         MAX(fabsf(getRcDeflection(FD_PITCH)),
                                             fabsf(getRcDeflection(FD_YAW))));
    const bool sticksAreCentered = (maxStickDeflection < 0.5f);
    const int8_t throttleThreshold = runtime.hoverThrottle + 15;  // 15% above hover to enter
    
    // If in filter mode, check grace period FIRST (before any early returns)
    // During first 500ms, if sticks move, this was a throttle blip before a roll/flip
    if (filterTracker.inFilterMode) {
        if (cmpTimeUs(now, filterTracker.filterModeEntryTime) < FILTER_MODE_GRACE_PERIOD_US) {
            if (!sticksAreCentered) {
                // Sticks moved during grace period - this is a roll/flip, not filter mode
                filterTracker.inFilterMode = false;
                filterTracker.lowThrottleTimerActive = false;
                // Fall through to roll/pitch detection below
            }
        }
    }
    
    // If tuneMode is already FILTER, check if we're still in an active filter session
    // filterTracker.inFilterMode gets set false when 2s low throttle timer expires (or grace period cancel)
    if (runtime.tuneMode == TUNE_MODE_FILTER) {
        if (filterTracker.inFilterMode) {
            // Still in active filter session - check for exit condition
            if (throttlePercent <= throttleThreshold) {
                if (!filterTracker.lowThrottleTimerActive) {
                    filterTracker.lowThrottleTimerActive = true;
                    filterTracker.lowThrottleStartTime = now;
                } else if (cmpTimeUs(now, filterTracker.lowThrottleStartTime) > 2000000) {
                    filterTracker.inFilterMode = false;
                    filterTracker.lowThrottleTimerActive = false;
                    return TUNE_MODE_NONE;
                }
            } else {
                filterTracker.lowThrottleTimerActive = false;
            }
            return TUNE_MODE_FILTER;
        }
        // Filter session ended - fall through to normal detection for potential new session
    }
    
    if (!filterTracker.inFilterMode) {
        // Not in filter mode - check for entry condition
        // Sticks centered AND throttle above threshold
        if (sticksAreCentered && throttlePercent > throttleThreshold) {
            filterTracker.inFilterMode = true;
            filterTracker.lowThrottleTimerActive = false;
            filterTracker.filterModeEntryTime = now;  // Record entry time for grace period
            return TUNE_MODE_FILTER;
        }
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

// ============================================================================
// GAIN CACHING
// ============================================================================

static void cacheCurrentGains(void)
{
    runtime.currentP = currentPidProfile->pid[runtime.currentAxis].P;
    runtime.currentI = currentPidProfile->pid[runtime.currentAxis].I;
    runtime.currentD = currentPidProfile->pid[runtime.currentAxis].D;
    runtime.currentF = currentPidProfile->pid[runtime.currentAxis].F;
    
    // Also initialize best gains to current values
    // This ensures we have valid values if convergence triggers before first save
    runtime.bestP = runtime.currentP;
    runtime.bestI = runtime.currentI;
    runtime.bestD = runtime.currentD;
    runtime.bestF = runtime.currentF;
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

static void updateDebugOutput(void)
{
    // [0] = state
    DEBUG_SET(DEBUG_AUTOTUNE, 0, runtime.state);
    
    // [1] = iteration * 10 + mode (e.g., 13 = iteration 1, mode 3/filter)
    // During hover tune, show hover tune iteration in high bits
    if (runtime.hoverTuneActive) {
        DEBUG_SET(DEBUG_AUTOTUNE, 1, runtime.hoverTuneIteration * 10 + 4);  // mode 4 = hover tune
    } else {
        DEBUG_SET(DEBUG_AUTOTUNE, 1, runtime.iteration * 10 + runtime.tuneMode);
    }
    
    // During hover tune, show motor RMS and noise attribution data
    if (runtime.hoverTuneActive) {
        // [1] = diagPhase * 10 + diagIteration (e.g., 32 = phase 3, iteration 2)
        DEBUG_SET(DEBUG_AUTOTUNE, 1, runtime.diagPhase * 10 + runtime.diagIteration);
        
        // [2] = baseline motor RMS * 10 (reference for improvement comparison)
        DEBUG_SET(DEBUG_AUTOTUNE, 2, (int16_t)(runtime.diagRms[HOVER_DIAG_BASELINE] * 10));
        
        // [3] = current/last motor RMS * 10 (the metric we're measuring)
        DEBUG_SET(DEBUG_AUTOTUNE, 3, (int16_t)(runtime.lastMotorRms * 10));
        
        // [4] = Roll improvement % (positive = lower RMS = better)
        DEBUG_SET(DEBUG_AUTOTUNE, 4, (int16_t)(runtime.diagImprovement[HOVER_DIAG_ROLL_TEST]));
        
        // [5] = Pitch improvement %
        DEBUG_SET(DEBUG_AUTOTUNE, 5, (int16_t)(runtime.diagImprovement[HOVER_DIAG_PITCH_TEST]));
        
        // [6] = Max of Gyro/Dterm improvement % (shows dominant filter contributor)
        {
            float gyroImp = runtime.diagImprovement[HOVER_DIAG_GYRO_LPF1_TEST];
            float dtermImp = runtime.diagImprovement[HOVER_DIAG_DTERM_LPF1_TEST];
            // Encode: positive = gyro dominant, negative = dterm dominant (for visualization)
            if (gyroImp >= dtermImp) {
                DEBUG_SET(DEBUG_AUTOTUNE, 6, (int16_t)(gyroImp));
            } else {
                DEBUG_SET(DEBUG_AUTOTUNE, 6, (int16_t)(-dtermImp));
            }
        }
    }
    // For filter mode, show filter frequencies instead of PID gains
    else if (runtime.tuneMode == TUNE_MODE_FILTER) {
        // Check if LPF1 filters are in dynamic mode
        bool dtermLpf1IsDynamic = (currentPidProfile->dterm_lpf1_static_hz == 0) && 
                                   (currentPidProfile->dterm_lpf1_dyn_min_hz > 0);
        bool gyroLpf1IsDynamic = (gyroConfig()->gyro_lpf1_static_hz == 0) && 
                                  (gyroConfig()->gyro_lpf1_dyn_min_hz > 0);
        
        // [2] = dterm_lpf1 Hz (dynamic min or static)
        DEBUG_SET(DEBUG_AUTOTUNE, 2, dtermLpf1IsDynamic ? 
            currentPidProfile->dterm_lpf1_dyn_min_hz : currentPidProfile->dterm_lpf1_static_hz);
        // [3] = dterm_lpf2 Hz
        DEBUG_SET(DEBUG_AUTOTUNE, 3, currentPidProfile->dterm_lpf2_static_hz);
        // [4] = gyro_lpf1 Hz (dynamic min or static)
        DEBUG_SET(DEBUG_AUTOTUNE, 4, gyroLpf1IsDynamic ? 
            gyroConfig()->gyro_lpf1_dyn_min_hz : gyroConfig()->gyro_lpf1_static_hz);
        // [5] = gyro_lpf2 Hz
        DEBUG_SET(DEBUG_AUTOTUNE, 5, gyroConfig()->gyro_lpf2_static_hz);
        // [6] = noise floor from filter analysis * 10
        DEBUG_SET(DEBUG_AUTOTUNE, 6, (int16_t)(runtime.filterAnalysis.noiseFloor * 10));
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
        
        // [4] = I gain
        DEBUG_SET(DEBUG_AUTOTUNE, 4, runtime.currentI);
        
        // [5] = F gain
        DEBUG_SET(DEBUG_AUTOTUNE, 5, runtime.currentF);
        
        // [6] = overshoot * 10
        DEBUG_SET(DEBUG_AUTOTUNE, 6, (int16_t)(runtime.metrics.overshootPercent * 10));
    }
    
    // [7] = ALWAYS reason code (explains what autotune is doing)
    DEBUG_SET(DEBUG_AUTOTUNE, 7, runtime.lastReasonCode);
}

// ============================================================================
// QUICK-TOGGLE SAVE FEATURE
// ============================================================================
// Toggle autotune off-on-off quickly (within 500ms each) to save changes to EEPROM

static struct {
    timeUs_t lastToggleTime;      // Time of last switch state change
    bool lastSwitchState;         // Previous switch state
    uint8_t quickToggleCount;     // Number of quick toggles detected
    bool saveTriggered;           // Prevent multiple saves
} saveTracker;

#define QUICK_TOGGLE_WINDOW_US   500000   // 500ms window for quick toggle
#define QUICK_TOGGLE_RESET_US   1000000   // Reset counter after 1s of no activity
#define QUICK_TOGGLES_TO_SAVE         2   // Need 2 complete off-on cycles

static void checkQuickToggleSave(bool currentSwitchState, timeUs_t currentTimeUs)
{
    // Detect switch state change
    if (currentSwitchState != saveTracker.lastSwitchState) {
        timeUs_t timeSinceLastToggle = cmpTimeUs(currentTimeUs, saveTracker.lastToggleTime);
        
        // On switch ON after being OFF
        if (currentSwitchState && timeSinceLastToggle < QUICK_TOGGLE_WINDOW_US) {
            // This was a quick off-to-on transition
            saveTracker.quickToggleCount++;
            
            // Check if we've reached the save threshold
            if (saveTracker.quickToggleCount >= QUICK_TOGGLES_TO_SAVE && !saveTracker.saveTriggered) {
                // Save to EEPROM!
                writeEEPROM();
                saveTracker.saveTriggered = true;
                saveTracker.quickToggleCount = 0;
            }
        }
        
        saveTracker.lastToggleTime = currentTimeUs;
        saveTracker.lastSwitchState = currentSwitchState;
    }
    
    // Reset counter if too much time has passed (only when not armed to prevent crashes)
    if (!ARMING_FLAG(ARMED) && cmpTimeUs(currentTimeUs, saveTracker.lastToggleTime) > QUICK_TOGGLE_RESET_US) {
        saveTracker.quickToggleCount = 0;
        saveTracker.saveTriggered = false;
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
    
    // Check for quick-toggle save gesture
    checkQuickToggleSave(IS_RC_MODE_ACTIVE(BOXAUTOTUNE), currentTimeUs);
    
    // Activation logic
    if (shouldBeActive && runtime.state == AUTOTUNE_STATE_IDLE) {
        changeState(AUTOTUNE_STATE_ARMED, currentTimeUs);
        runtime.tuneMode = TUNE_MODE_NONE;
        runtime.iteration = 0;
        runtime.historyCount = 0;
        runtime.bestScore = 1000.0f;
        runtime.status = STATUS_WAITING_MANEUVER;
        
        // Reset hover tune state
        runtime.hoverTuneActive = false;
        runtime.hoverTuneIteration = 0;
        runtime.bestMotorRms = 1000.0f;
        resetMotorRmsAccum();
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
            runtime.lastReasonCode = REASON_IDLE;
            break;
            
        case AUTOTUNE_STATE_ARMED:
            {
                // First, calibrate hover if not done yet
                if (!runtime.hoverCalibrated) {
                    runtime.status = STATUS_WAITING_MANEUVER;  // "Waiting" = calibrating hover
                    runtime.lastReasonCode = REASON_HOVER_WAITING;
                    if (calibrateHover(currentTimeUs)) {
                        // Hover calibrated! Give wiggle to indicate ready
                        hoverCalibratedWiggleStart = currentTimeUs;
                        hoverWiggleActive = true;
                        
                        // Start hover-based diagnostic tuning (Phase 0)
                        runtime.hoverTuneActive = true;
                        runtime.lastMotorRmsTime = currentTimeUs;
                        runtime.diagPhase = HOVER_DIAG_BASELINE;
                        runtime.diagIteration = 0;
                        resetMotorRmsAccum();
                        // Clear diagnostic arrays
                        for (int i = 0; i < HOVER_DIAG_PHASE_COUNT; i++) {
                            runtime.diagRms[i] = 0.0f;
                            runtime.diagImprovement[i] = 0.0f;
                        }
                    }
                    break;
                }
                
                // Phase 0: Procedural diagnostic hover tune
                // While hovering (sticks centered, stable throttle), run A/B tests to identify noise source
                if (runtime.hoverTuneActive && isHoverStable()) {
                    runtime.status = STATUS_COLLECTING_DATA;  // Show we're doing something
                    if (updateHoverDiagnostic(currentTimeUs)) {
                        // Diagnostic tuning complete
                        runtime.hoverTuneActive = false;
                        // Reason code already set by updateHoverDiagnostic
                    }
                    // Don't detect maneuvers while hover tuning - wait for stable baseline
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
                    
                    // Set reason code based on detected mode
                    if (detected == TUNE_MODE_FILTER) {
                        runtime.lastReasonCode = REASON_FILTER_COLLECTING;
                    } else {
                        runtime.lastReasonCode = REASON_PID_COLLECTING;
                    }
                    
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
                // Set appropriate reason code based on tune mode
                if (runtime.tuneMode == TUNE_MODE_FILTER) {
                    runtime.lastReasonCode = REASON_FILTER_COLLECTING;
                } else {
                    runtime.lastReasonCode = REASON_PID_COLLECTING;
                }
                
                // For filter mode, update the exit timer (tracks 2s low throttle)
                // Also check grace period - if sticks move, switch to roll/pitch
                if (runtime.tuneMode == TUNE_MODE_FILTER) {
                    updateFilterModeExitTimer(currentTimeUs);
                    
                    // Grace period: during first 500ms, if we detect roll/pitch, go back to ARMED
                    // This gives a clean restart for the new mode (resets history, iteration tracking)
                    if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) < FILTER_MODE_GRACE_PERIOD_US) {
                        runtime.lastReasonCode = REASON_GRACE_PERIOD;
                        const float rollRate = fabsf(gyro.gyroADCf[FD_ROLL]);
                        const float pitchRate = fabsf(gyro.gyroADCf[FD_PITCH]);
                        
                        // Check for roll - lower threshold during grace period since maneuver is starting
                        if (rollRate > 150.0f && fabsf(getRcDeflection(FD_ROLL)) > 0.4f) {
                            // Cancel filter mode and return to ARMED for clean roll mode start
                            runtime.tuneMode = TUNE_MODE_NONE;
                            filterTracker.inFilterMode = false;
                            changeState(AUTOTUNE_STATE_ARMED, currentTimeUs);
                            break;
                        }
                        // Check for pitch/flip
                        else if (pitchRate > 150.0f && fabsf(getRcDeflection(FD_PITCH)) > 0.4f) {
                            // Cancel filter mode and return to ARMED for clean pitch mode start
                            runtime.tuneMode = TUNE_MODE_NONE;
                            filterTracker.inFilterMode = false;
                            changeState(AUTOTUNE_STATE_ARMED, currentTimeUs);
                            break;
                        }
                    }
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
                    
                    // Use actual setpoint from PID controller, not approximation
                    float setpoint = getSetpointRate(runtime.currentAxis);
                    
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
                        // Use actual setpoint - should be near zero during settling
                        float setpoint = getSetpointRate(runtime.currentAxis);
                        collectSample(
                            gyro.gyroADCf[runtime.currentAxis],
                            setpoint,
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
                        runtime.lastReasonCode = REASON_DATA_INVALID;
                        // Don't apply gains, don't increment iteration - just signal and try again
                    } else if (runtime.tuneMode == TUNE_MODE_FILTER) {
                        // Apply filter adjustments
                        runtime.status = STATUS_ADJUSTING_FILTER;
                        autotuneApplyFilterAdjustment(
                            &runtime.filterAnalysis,
                            runtime.filterAnalysis.noiseFloor,
                            NOISE_TARGET_RMS,
                            &runtime.lastReasonCode
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
                            runtime.responseClass,
                            &runtime.lastReasonCode
                        );
                        runtime.iteration++;
                    }
                }
                
                // Check for convergence (only if we have valid data this round)
                if (runtime.dataValid && autotuneCheckConvergence(&runtime)) {
                    autotuneRestoreBestGains(&runtime);
                    changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
                    runtime.status = STATUS_COMPLETE;
                    runtime.lastReasonCode = (runtime.tuneMode == TUNE_MODE_FILTER) ? 
                                             REASON_FILTER_COMPLETE : REASON_PID_COMPLETE;
                } else if (runtime.iteration >= autotuneConfig()->max_iterations) {
                    autotuneRestoreBestGains(&runtime);
                    changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
                    runtime.status = STATUS_COMPLETE;
                    runtime.lastReasonCode = (runtime.tuneMode == TUNE_MODE_FILTER) ? 
                                             REASON_FILTER_COMPLETE : REASON_PID_COMPLETE;
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
