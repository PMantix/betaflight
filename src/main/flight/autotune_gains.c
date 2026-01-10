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
#include "flight/pid.h"
#include "flight/pid_init.h"
#include "flight/autotune_types.h"
#include "flight/autotune_gains.h"

// External access to current PID profile
extern pidProfile_t *currentPidProfile;

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
    autotuneResponseClass_e responseClass
)
{
    bool gainsChanged = false;
    uint8_t axis = runtime->currentAxis;
    
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
                } else if (attribution->pDirection == ADJUST_DECREASE) {
                    runtime->currentP = MAX(GAIN_MIN_VALUE, runtime->currentP - pDelta);
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
                } else if (attribution->dDirection == ADJUST_DECREASE) {
                    runtime->currentD = MAX(GAIN_MIN_VALUE, runtime->currentD - dDelta);
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
                } else if (attribution->iDirection == ADJUST_DECREASE) {
                    runtime->currentI = MAX(GAIN_MIN_VALUE, runtime->currentI - iDelta);
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
                } else if (attribution->fDirection == ADJUST_DECREASE) {
                    runtime->currentF = MAX(0, runtime->currentF - fDelta);
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
    
    UNUSED(responseClass);
    
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

// Filter frequency limits
#define GYRO_LPF1_MIN_HZ     100
#define GYRO_LPF1_MAX_HZ     400
#define GYRO_LPF2_MIN_HZ     150
#define GYRO_LPF2_MAX_HZ     500
#define DTERM_LPF1_MIN_HZ     50
#define DTERM_LPF1_MAX_HZ    200
#define DTERM_LPF2_MIN_HZ    100
#define DTERM_LPF2_MAX_HZ    300

// Adjustment step size (Hz)
#define FILTER_STEP_HZ       10

bool autotuneApplyFilterAdjustment(
    const autotuneFilterAnalysis_t *filterAnalysis,
    float currentNoise,
    float targetNoise
)
{
    UNUSED(filterAnalysis);
    
    bool changed = false;
    
    // Get current filter settings
    uint16_t gyroLpf1Hz = gyroConfig()->gyro_lpf1_static_hz;
    uint16_t gyroLpf2Hz = gyroConfig()->gyro_lpf2_static_hz;
    uint16_t dtermLpf1Hz = currentPidProfile->dterm_lpf1_static_hz;
    uint16_t dtermLpf2Hz = currentPidProfile->dterm_lpf2_static_hz;
    
    // Calculate noise ratio - how far are we from target?
    // ratio > 1 means too much noise, need more filtering (lower freqs)
    // ratio < 1 means noise is low, can use less filtering (higher freqs)
    float noiseRatio = (targetNoise > 0.1f) ? (currentNoise / targetNoise) : 1.0f;
    
    if (noiseRatio > 1.2f) {
        // Too much noise - lower filter frequencies (more aggressive filtering)
        // Lower dterm first (most sensitive to noise)
        if (dtermLpf1Hz > DTERM_LPF1_MIN_HZ) {
            dtermLpf1Hz = MAX(dtermLpf1Hz - FILTER_STEP_HZ, DTERM_LPF1_MIN_HZ);
            changed = true;
        }
        if (dtermLpf2Hz > DTERM_LPF2_MIN_HZ && dtermLpf2Hz > 0) {
            dtermLpf2Hz = MAX(dtermLpf2Hz - FILTER_STEP_HZ, DTERM_LPF2_MIN_HZ);
            changed = true;
        }
        // If dterm is already at minimum, lower gyro filters
        if (dtermLpf1Hz <= DTERM_LPF1_MIN_HZ && gyroLpf1Hz > GYRO_LPF1_MIN_HZ) {
            gyroLpf1Hz = MAX(gyroLpf1Hz - FILTER_STEP_HZ, GYRO_LPF1_MIN_HZ);
            changed = true;
        }
    } else if (noiseRatio < 0.8f) {
        // Low noise - raise filter frequencies (less filtering, better response)
        // Raise gyro first (less impact on noise)
        if (gyroLpf1Hz < GYRO_LPF1_MAX_HZ && gyroLpf1Hz > 0) {
            gyroLpf1Hz = MIN(gyroLpf1Hz + FILTER_STEP_HZ, GYRO_LPF1_MAX_HZ);
            changed = true;
        }
        if (gyroLpf2Hz < GYRO_LPF2_MAX_HZ && gyroLpf2Hz > 0) {
            gyroLpf2Hz = MIN(gyroLpf2Hz + FILTER_STEP_HZ, GYRO_LPF2_MAX_HZ);
            changed = true;
        }
        // If gyro is already at maximum, raise dterm filters
        if (gyroLpf1Hz >= GYRO_LPF1_MAX_HZ && dtermLpf1Hz < DTERM_LPF1_MAX_HZ) {
            dtermLpf1Hz = MIN(dtermLpf1Hz + FILTER_STEP_HZ, DTERM_LPF1_MAX_HZ);
            changed = true;
        }
    }
    // else noise is in acceptable range - no change needed
    
    // Apply changes if any
    if (changed) {
        gyroConfigMutable()->gyro_lpf1_static_hz = gyroLpf1Hz;
        gyroConfigMutable()->gyro_lpf2_static_hz = gyroLpf2Hz;
        currentPidProfile->dterm_lpf1_static_hz = dtermLpf1Hz;
        currentPidProfile->dterm_lpf2_static_hz = dtermLpf2Hz;
        
        // Note: Filter reinitialization would typically require pidInitFilters()
        // but that's expensive - for now we just update the config
        // The changes will take effect on next reboot or can be reinit'd
    }
    
    return changed;
}

#endif // USE_AUTOTUNE
