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

#include "common/maths.h"
#include "config/simplified_tuning.h"
#include "flight/pid.h"
#include "flight/pid_init.h"
#include "flight/autotune_types.h"
#include "flight/autotune_gains.h"
#include "sensors/gyro_init.h"

// External access to current PID profile
extern pidProfile_t *currentPidProfile;

// ============================================================================
// SLIDER MODE HELPER
// ============================================================================

// Disable simplified PID slider mode so our direct changes aren't overwritten
static void disableSliderMode(void)
{
    if (currentPidProfile->simplified_pids_mode != PID_SIMPLIFIED_TUNING_OFF) {
        currentPidProfile->simplified_pids_mode = PID_SIMPLIFIED_TUNING_OFF;
    }
}

// ============================================================================
// GAIN ACCESS HELPERS (inline macros for direct access)
// ============================================================================

#define GET_GAIN_P(axis) (currentPidProfile->pid[axis].P)
#define GET_GAIN_I(axis) (currentPidProfile->pid[axis].I)
#define GET_GAIN_D(axis) (currentPidProfile->pid[axis].D)
#define GET_GAIN_F(axis) (currentPidProfile->pid[axis].F)

#define SET_GAIN_P(axis, value) (currentPidProfile->pid[axis].P = (value))
#define SET_GAIN_I(axis, value) (currentPidProfile->pid[axis].I = (value))
#define SET_GAIN_D(axis, value) (currentPidProfile->pid[axis].D = (value))
#define SET_GAIN_F(axis, value) (currentPidProfile->pid[axis].F = (value))

// ============================================================================
// GAIN ADJUSTMENT
// ============================================================================

float autotuneCalculateAdjustmentStep(
    const autotuneRuntime_t *runtime,
    autotuneGainAttribution_e gainType,
    autotuneAdjustDir_e direction
)
{
    UNUSED(gainType);
    UNUSED(direction);
    
    // Base step size as percentage
    float stepPercent = GAIN_ADJUST_STEP_PERCENT;
    
    // Scale by iteration - start aggressive, become more conservative
    if (runtime->iteration > 5) {
        stepPercent *= 0.7f;
    }
    if (runtime->iteration > 10) {
        stepPercent *= 0.7f;
    }
    
    // Look at history for oscillation in adjustments
    if (runtime->historyCount >= 3) {
        // Check if we've been oscillating (up-down-up or down-up-down)
        const autotuneHistoryEntry_t *h0 = &runtime->history[runtime->historyCount - 1];
        const autotuneHistoryEntry_t *h1 = &runtime->history[runtime->historyCount - 2];
        const autotuneHistoryEntry_t *h2 = &runtime->history[runtime->historyCount - 3];
        
        // Check P oscillation
        if ((h0->pGain > h1->pGain && h1->pGain < h2->pGain) ||
            (h0->pGain < h1->pGain && h1->pGain > h2->pGain)) {
            stepPercent *= 0.5f;  // Reduce step if oscillating
        }
    }
    
    return stepPercent;
}

bool autotuneApplyGainAdjustment(
    autotuneRuntime_t *runtime,
    const autotuneAttribution_t *attribution,
    autotuneResponseClass_e responseClass,
    uint16_t *reasonCode
)
{
    bool gainsChanged = false;
    uint8_t axis = runtime->currentAxis;
    
    // Initialize reason code
    if (reasonCode) {
        *reasonCode = REASON_PID_RESPONSE_GOOD;
    }
    
    // Get adjustment step size
    float stepPercent = autotuneCalculateAdjustmentStep(
        runtime,
        attribution->primary,
        attribution->pDirection
    );
    
    // Scale step by confidence
    stepPercent *= attribution->confidence;
    
    // Apply primary adjustment
    switch (attribution->primary) {
        case GAIN_ATTRIBUTION_P:
            {
                int16_t pDelta = (int16_t)(runtime->currentP * stepPercent / 100.0f);
                if (pDelta < 1) pDelta = 1;
                
                if (attribution->pDirection == ADJUST_INCREASE) {
                    runtime->currentP = MIN(GAIN_MAX_VALUE, runtime->currentP + pDelta);
                    // Set reason based on why we're increasing P
                    if (reasonCode) {
                        *reasonCode = REASON_PID_SLUGGISH_P_UP;  // Overdamped/sluggish -> raise P
                    }
                } else if (attribution->pDirection == ADJUST_DECREASE) {
                    runtime->currentP = MAX(GAIN_MIN_VALUE, runtime->currentP - pDelta);
                    // Set reason based on why we're decreasing P
                    if (reasonCode) {
                        if (responseClass == RESPONSE_UNDERDAMPED) {
                            *reasonCode = REASON_PID_OVERSHOOT_P_DOWN;  // Overshoot/oscillation
                        } else {
                            *reasonCode = REASON_PID_OSCILLATION_P_DOWN;  // Default for P down
                        }
                    }
                }
                
                SET_GAIN_P(axis, runtime->currentP);
                gainsChanged = true;
            }
            break;
            
        case GAIN_ATTRIBUTION_D:
            {
                int16_t dDelta = (int16_t)(runtime->currentD * stepPercent / 100.0f);
                if (dDelta < 1) dDelta = 1;
                
                if (attribution->dDirection == ADJUST_INCREASE) {
                    runtime->currentD = MIN(GAIN_MAX_VALUE, runtime->currentD + dDelta);
                    // Set reason based on why we're increasing D
                    if (reasonCode) {
                        if (responseClass == RESPONSE_UNDERDAMPED) {
                            *reasonCode = REASON_PID_OSCILLATION_D_UP;  // Oscillation -> more damping
                        } else {
                            *reasonCode = REASON_PID_OVERSHOOT_D_UP;  // Overshoot -> more damping
                        }
                    }
                } else if (attribution->dDirection == ADJUST_DECREASE) {
                    runtime->currentD = MAX(GAIN_MIN_VALUE, runtime->currentD - dDelta);
                    // Set reason based on why we're decreasing D
                    if (reasonCode) {
                        if (responseClass == RESPONSE_NOISY) {
                            *reasonCode = REASON_PID_NOISE_D_DOWN;
                        } else if (responseClass == RESPONSE_OVERDAMPED) {
                            *reasonCode = REASON_PID_SLUGGISH_D_DOWN;
                        } else {
                            *reasonCode = REASON_PID_NOISE_D_DOWN;  // Default for D down
                        }
                    }
                }
                
                SET_GAIN_D(axis, runtime->currentD);
                gainsChanged = true;
            }
            break;
            
        case GAIN_ATTRIBUTION_I:
            {
                int16_t iDelta = (int16_t)(runtime->currentI * stepPercent / 100.0f);
                if (iDelta < 1) iDelta = 1;
                
                if (attribution->iDirection == ADJUST_INCREASE) {
                    runtime->currentI = MIN(GAIN_MAX_VALUE, runtime->currentI + iDelta);
                    if (reasonCode) {
                        *reasonCode = REASON_PID_DRIFT_I_UP;  // Drift detected, raising I
                    }
                } else if (attribution->iDirection == ADJUST_DECREASE) {
                    runtime->currentI = MAX(GAIN_MIN_VALUE, runtime->currentI - iDelta);
                    if (reasonCode) {
                        *reasonCode = REASON_PID_BOUNCEBACK_I_DOWN;  // Bounceback or slow osc, lowering I
                    }
                }
                
                SET_GAIN_I(axis, runtime->currentI);
                gainsChanged = true;
            }
            break;
            
        case GAIN_ATTRIBUTION_F:
            {
                int16_t fDelta = (int16_t)(runtime->currentF * stepPercent / 100.0f);
                if (fDelta < 5) fDelta = 5;
                
                if (attribution->fDirection == ADJUST_INCREASE) {
                    runtime->currentF = MIN(2000, runtime->currentF + fDelta);
                    if (reasonCode) {
                        *reasonCode = REASON_PID_LAG_F_UP;  // Stick lag, raising F
                    }
                } else if (attribution->fDirection == ADJUST_DECREASE) {
                    runtime->currentF = MAX(0, runtime->currentF - fDelta);
                    if (reasonCode) {
                        *reasonCode = REASON_PID_LEAD_F_DOWN;  // Gyro leading stick, lowering F
                    }
                }
                
                SET_GAIN_F(axis, runtime->currentF);
                gainsChanged = true;
            }
            break;
            
        default:
            break;
    }
    
    // Apply secondary adjustment if present (at reduced step)
    if (attribution->secondary != GAIN_ATTRIBUTION_NONE && 
        attribution->secondary != attribution->primary) {
        
        float secondaryStep = stepPercent * 0.5f;  // Half step for secondary
        
        switch (attribution->secondary) {
            case GAIN_ATTRIBUTION_D:
                if (attribution->dDirection != ADJUST_NONE) {
                    int16_t dDelta = (int16_t)(runtime->currentD * secondaryStep / 100.0f);
                    if (dDelta < 1) dDelta = 1;
                    
                    if (attribution->dDirection == ADJUST_INCREASE) {
                        runtime->currentD = MIN(GAIN_MAX_VALUE, runtime->currentD + dDelta);
                    } else {
                        runtime->currentD = MAX(GAIN_MIN_VALUE, runtime->currentD - dDelta);
                    }
                    SET_GAIN_D(axis, runtime->currentD);
                    gainsChanged = true;
                }
                break;
                
            case GAIN_ATTRIBUTION_P:
                if (attribution->pDirection != ADJUST_NONE) {
                    int16_t pDelta = (int16_t)(runtime->currentP * secondaryStep / 100.0f);
                    if (pDelta < 1) pDelta = 1;
                    
                    if (attribution->pDirection == ADJUST_INCREASE) {
                        runtime->currentP = MIN(GAIN_MAX_VALUE, runtime->currentP + pDelta);
                    } else {
                        runtime->currentP = MAX(GAIN_MIN_VALUE, runtime->currentP - pDelta);
                    }
                    SET_GAIN_P(axis, runtime->currentP);
                    gainsChanged = true;
                }
                break;
                
            default:
                break;
        }
    }
    
    // If we changed any gains, disable slider mode so changes persist
    if (gainsChanged) {
        disableSliderMode();
    }
    
    return gainsChanged;
}

void autotuneSaveToHistory(autotuneRuntime_t *runtime, float score)
{
    if (runtime->historyCount >= AUTOTUNE_HISTORY_SIZE) {
        // Shift history down
        for (int i = 0; i < AUTOTUNE_HISTORY_SIZE - 1; i++) {
            runtime->history[i] = runtime->history[i + 1];
        }
        runtime->historyCount = AUTOTUNE_HISTORY_SIZE - 1;
    }
    
    autotuneHistoryEntry_t *entry = &runtime->history[runtime->historyCount];
    entry->pGain = runtime->currentP;
    entry->iGain = runtime->currentI;
    entry->dGain = runtime->currentD;
    entry->fGain = runtime->currentF;
    entry->overshoot = runtime->metrics.overshootPercent;
    entry->riseTime = runtime->metrics.riseTimeMs;
    entry->noise = runtime->metrics.noiseRms;
    entry->score = score;
    entry->responseClass = runtime->responseClass;
    
    runtime->historyCount++;
    
    // Track best gains
    if (score < runtime->bestScore || runtime->iteration == 0) {
        runtime->bestScore = score;
        runtime->bestP = runtime->currentP;
        runtime->bestI = runtime->currentI;
        runtime->bestD = runtime->currentD;
        runtime->bestF = runtime->currentF;
    }
}

bool autotuneCheckConvergence(const autotuneRuntime_t *runtime)
{
    // Need at least 3 iterations to check convergence
    if (runtime->historyCount < 3) {
        return false;
    }
    
    // Check if response is excellent
    if (runtime->responseClass == RESPONSE_EXCELLENT) {
        return true;
    }
    
    // Check if scores have stabilized (last 3 within 10% of each other)
    float score0 = runtime->history[runtime->historyCount - 1].score;
    float score1 = runtime->history[runtime->historyCount - 2].score;
    float score2 = runtime->history[runtime->historyCount - 3].score;
    
    float avgScore = (score0 + score1 + score2) / 3.0f;
    float maxDev = MAX(fabsf(score0 - avgScore), MAX(fabsf(score1 - avgScore), fabsf(score2 - avgScore)));
    
    if (avgScore > 0.1f && (maxDev / avgScore) < 0.1f) {
        // Scores have stabilized
        return true;
    }
    
    return false;
}

void autotuneRestoreBestGains(autotuneRuntime_t *runtime)
{
    uint8_t axis = runtime->currentAxis;
    
    runtime->currentP = runtime->bestP;
    runtime->currentI = runtime->bestI;
    runtime->currentD = runtime->bestD;
    runtime->currentF = runtime->bestF;
    
    SET_GAIN_P(axis, runtime->bestP);
    SET_GAIN_I(axis, runtime->bestI);
    SET_GAIN_D(axis, runtime->bestD);
    SET_GAIN_F(axis, runtime->bestF);
}

// ============================================================================
// FILTER ADJUSTMENT
// ============================================================================

#include "sensors/gyro.h"

// Filter frequency limits are now defined in autotune_types.h:
// GYRO_LPF1: [125-250-375], GYRO_LPF2: [250-500-750]
// DTERM_LPF1: [37-75-112], DTERM_LPF2: [75-150-225]

// Resonance-based tuning: frequency threshold for problematic peaks
#define RESONANCE_PROBLEM_FREQ_HZ  30.0f

bool autotuneApplyFilterAdjustment(
    const autotuneFilterAnalysis_t *filterAnalysis,
    float currentNoise,
    float targetNoise,
    uint16_t *reasonCode
)
{
    bool changed = false;
    uint16_t reason = REASON_FILTER_NO_CHANGE;
    
    // Check if LPF1 filters are in dynamic mode
    // Dynamic mode: static_hz == 0 && dyn_min_hz > 0
    bool gyroLpf1IsDynamic = (gyroConfig()->gyro_lpf1_static_hz == 0) && 
                              (gyroConfig()->gyro_lpf1_dyn_min_hz > 0);
    bool dtermLpf1IsDynamic = (currentPidProfile->dterm_lpf1_static_hz == 0) && 
                               (currentPidProfile->dterm_lpf1_dyn_min_hz > 0);
    
    // Get current filter settings (dynamic min or static)
    uint16_t gyroLpf1Hz = gyroLpf1IsDynamic ? 
        gyroConfig()->gyro_lpf1_dyn_min_hz : gyroConfig()->gyro_lpf1_static_hz;
    uint16_t gyroLpf2Hz = gyroConfig()->gyro_lpf2_static_hz;
    uint16_t dtermLpf1Hz = dtermLpf1IsDynamic ? 
        currentPidProfile->dterm_lpf1_dyn_min_hz : currentPidProfile->dterm_lpf1_static_hz;
    uint16_t dtermLpf2Hz = currentPidProfile->dterm_lpf2_static_hz;
    
    // Track original values for change detection
    uint16_t origGyroLpf1Hz = gyroLpf1Hz;
    uint16_t origGyroLpf2Hz = gyroLpf2Hz;
    uint16_t origDtermLpf1Hz = dtermLpf1Hz;
    uint16_t origDtermLpf2Hz = dtermLpf2Hz;
    
    // PRIMARY DECISION: Resonance-based tuning
    // If resonance peaks detected above 30Hz, tighten filters to eliminate them
    // If no resonance, relax filters for better response
    
    if (filterAnalysis->resonanceDetected && filterAnalysis->peakFrequency > RESONANCE_PROBLEM_FREQ_HZ) {
        // Resonance detected - need to tighten filters
        // Lower LPF cutoffs to be below the peak frequency
        float targetCutoff = filterAnalysis->peakFrequency * 0.7f;  // 30% below peak
        
        // Check if ANY filter is still above targetCutoff and can be lowered
        // Priority: dterm LPF1 > dterm LPF2 > gyro LPF1 (most to least impact on motor output)
        if (dtermLpf1Hz > DTERM_LPF1_MIN_HZ && dtermLpf1Hz > targetCutoff) {
            dtermLpf1Hz = MAX(dtermLpf1Hz - FILTER_STEP_HZ, DTERM_LPF1_MIN_HZ);
            reason = REASON_FILTER_RESONANCE_LPF;
            changed = true;
        } else if (dtermLpf2Hz > DTERM_LPF2_MIN_HZ && dtermLpf2Hz > 0 && dtermLpf2Hz > targetCutoff) {
            dtermLpf2Hz = MAX(dtermLpf2Hz - FILTER_STEP_HZ, DTERM_LPF2_MIN_HZ);
            reason = REASON_FILTER_RESONANCE_LPF;
            changed = true;
        } else if (gyroLpf1Hz > 0 && gyroLpf1Hz > GYRO_LPF1_MIN_HZ && gyroLpf1Hz > targetCutoff) {
            // Lower gyro LPF1 if it's above targetCutoff (don't require dterm at min first)
            gyroLpf1Hz = MAX(gyroLpf1Hz - FILTER_STEP_HZ, GYRO_LPF1_MIN_HZ);
            reason = REASON_FILTER_RESONANCE_LPF;
            changed = true;
        }
        // If all filters are already below targetCutoff, resonance is adequately filtered
        // Fall through to noise-based adjustment below
    }
    
    // Noise-based filter adjustment (also reached if resonance is already adequately filtered)
    if (!changed) {
        // NO resonance detected - adjust filters based on noise level
        
        // Calculate noise ratio (how far are we from target?)
        float noiseRatio = (targetNoise > 0.1f) ? (currentNoise / targetNoise) : 1.0f;
        
        // If noise is BELOW target, can try relaxing filters for better response
        if (noiseRatio < 0.8f) {
            // Noise well below target - try to raise dterm filters first (more response)
            if (dtermLpf1Hz > 0 && dtermLpf1Hz < DTERM_LPF1_MAX_HZ) {
                dtermLpf1Hz = MIN(dtermLpf1Hz + FILTER_STEP_HZ, DTERM_LPF1_MAX_HZ);
                reason = REASON_FILTER_NOISE_LOW_LPF;
                changed = true;
            } else if (dtermLpf2Hz < DTERM_LPF2_MAX_HZ && dtermLpf2Hz > 0) {
                dtermLpf2Hz = MIN(dtermLpf2Hz + FILTER_STEP_HZ, DTERM_LPF2_MAX_HZ);
                reason = REASON_FILTER_NOISE_LOW_LPF;
                changed = true;
            } else if (gyroLpf1Hz > 0 && gyroLpf1Hz < GYRO_LPF1_MAX_HZ) {
                gyroLpf1Hz = MIN(gyroLpf1Hz + FILTER_STEP_HZ, GYRO_LPF1_MAX_HZ);
                reason = REASON_FILTER_NOISE_LOW_LPF;
                changed = true;
            } else if (gyroLpf2Hz > 0 && gyroLpf2Hz < GYRO_LPF2_MAX_HZ) {
                gyroLpf2Hz = MIN(gyroLpf2Hz + FILTER_STEP_HZ, GYRO_LPF2_MAX_HZ);
                reason = REASON_FILTER_NOISE_LOW_LPF;
                changed = true;
            } else {
                reason = REASON_AT_LIMIT;
            }
        }
        // If noise is ABOVE target, tighten filters
        else if (noiseRatio > 1.2f) {
            // Noise above target - lower dterm filters first (most impact on motor noise)
            if (dtermLpf1Hz > DTERM_LPF1_MIN_HZ) {
                dtermLpf1Hz = MAX(dtermLpf1Hz - FILTER_STEP_HZ, DTERM_LPF1_MIN_HZ);
                reason = REASON_FILTER_NOISE_HIGH_LPF;
                changed = true;
            } else if (dtermLpf2Hz > DTERM_LPF2_MIN_HZ && dtermLpf2Hz > 0) {
                dtermLpf2Hz = MAX(dtermLpf2Hz - FILTER_STEP_HZ, DTERM_LPF2_MIN_HZ);
                reason = REASON_FILTER_NOISE_HIGH_LPF;
                changed = true;
            } else if (gyroLpf1Hz > 0 && gyroLpf1Hz > GYRO_LPF1_MIN_HZ) {
                gyroLpf1Hz = MAX(gyroLpf1Hz - FILTER_STEP_HZ, GYRO_LPF1_MIN_HZ);
                reason = REASON_FILTER_NOISE_HIGH_LPF;
                changed = true;
            } else {
                reason = REASON_AT_LIMIT;
            }
        } else {
            // Noise in acceptable range (0.8 - 1.2) - no change needed
            reason = REASON_FILTER_NOISE_OK;
        }
    }
    
    // Apply changes if any
    if (changed) {
        // Apply gyro LPF1 (dynamic or static)
        if (gyroLpf1Hz != origGyroLpf1Hz) {
            if (gyroLpf1IsDynamic) {
                gyroConfigMutable()->gyro_lpf1_dyn_min_hz = gyroLpf1Hz;
            } else {
                gyroConfigMutable()->gyro_lpf1_static_hz = gyroLpf1Hz;
            }
        }
        
        // Apply gyro LPF2 (always static)
        if (gyroLpf2Hz != origGyroLpf2Hz) {
            gyroConfigMutable()->gyro_lpf2_static_hz = gyroLpf2Hz;
        }
        
        // Apply dterm LPF1 (dynamic or static)
        if (dtermLpf1Hz != origDtermLpf1Hz) {
            if (dtermLpf1IsDynamic) {
                currentPidProfile->dterm_lpf1_dyn_min_hz = dtermLpf1Hz;
            } else {
                currentPidProfile->dterm_lpf1_static_hz = dtermLpf1Hz;
            }
        }
        
        // Apply dterm LPF2 (always static)
        if (dtermLpf2Hz != origDtermLpf2Hz) {
            currentPidProfile->dterm_lpf2_static_hz = dtermLpf2Hz;
        }
        
        // Reinitialize filters so changes take effect immediately
        gyroInitFilters();
        pidInitFilters(currentPidProfile);
    }
    
    if (reasonCode) {
        *reasonCode = reason;
    }
    
    return changed;
}

#endif // USE_AUTOTUNE
