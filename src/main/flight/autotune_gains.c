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
#include "flight/autotune_analysis.h"
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

// Check if a specific gain is oscillating (up-down-up or down-up-down pattern)
static bool isGainOscillating(const autotuneRuntime_t *runtime, autotuneGainAttribution_e gainType)
{
    if (runtime->historyCount < 3) {
        return false;
    }
    
    const autotuneHistoryEntry_t *h0 = &runtime->history[runtime->historyCount - 1];
    const autotuneHistoryEntry_t *h1 = &runtime->history[runtime->historyCount - 2];
    const autotuneHistoryEntry_t *h2 = &runtime->history[runtime->historyCount - 3];
    
    int v0, v1, v2;
    switch (gainType) {
        case GAIN_ATTRIBUTION_P:
            v0 = h0->pGain; v1 = h1->pGain; v2 = h2->pGain;
            break;
        case GAIN_ATTRIBUTION_D:
            v0 = h0->dGain; v1 = h1->dGain; v2 = h2->dGain;
            break;
        case GAIN_ATTRIBUTION_F:
            v0 = h0->fGain; v1 = h1->fGain; v2 = h2->fGain;
            break;
        case GAIN_ATTRIBUTION_I:
            v0 = h0->iGain; v1 = h1->iGain; v2 = h2->iGain;
            break;
        default:
            return false;
    }
    
    // Check for up-down-up or down-up-down pattern
    return (v0 > v1 && v1 < v2) || (v0 < v1 && v1 > v2);
}

// Check if score got worse after last adjustment
static bool didScoreGetWorse(const autotuneRuntime_t *runtime)
{
    if (runtime->historyCount < 2) {
        return false;
    }
    
    float currentScore = runtime->history[runtime->historyCount - 1].score;
    float previousScore = runtime->history[runtime->historyCount - 2].score;
    
    // Score got worse if it increased by more than 5%
    return currentScore > previousScore * 1.05f;
}

float autotuneCalculateAdjustmentStep(
    const autotuneRuntime_t *runtime,
    autotuneGainAttribution_e gainType,
    autotuneAdjustDir_e direction
)
{
    UNUSED(direction);
    
    float stepPercent;
    
    // Use larger probe steps initially for faster convergence
    if (runtime->historyCount < 2) {
        // First iterations: big steps to quickly find the right ballpark
        stepPercent = GAIN_PROBE_STEP_PERCENT;  // 25%
    } else if (runtime->historyCount < 4) {
        // Intermediate: medium steps
        stepPercent = GAIN_PROBE_STEP_PERCENT * 0.6f;  // 15%
    } else {
        // Later: smaller refinement steps
        stepPercent = GAIN_ADJUST_STEP_PERCENT;  // 8%
        
        // Further reduce if converging
        if (runtime->iteration > 8) {
            stepPercent *= 0.7f;  // ~5.6%
        }
    }
    
    // Check for oscillation in THIS specific gain - strong damping if bouncing
    if (isGainOscillating(runtime, gainType)) {
        stepPercent *= 0.25f;  // Reduce to 25% if oscillating (was 50%)
    }
    
    // If score got worse, be more conservative
    if (didScoreGetWorse(runtime)) {
        stepPercent *= 0.5f;
    }
    
    return stepPercent;
}

// Check if we should reverse direction for a gain based on history
// Returns the potentially reversed direction
static autotuneAdjustDir_e getSmartDirection(
    const autotuneRuntime_t *runtime,
    autotuneGainAttribution_e gainType,
    autotuneAdjustDir_e proposedDirection
)
{
    if (runtime->historyCount < 2 || proposedDirection == ADJUST_NONE) {
        return proposedDirection;
    }
    
    // Get the last two history entries
    const autotuneHistoryEntry_t *h0 = &runtime->history[runtime->historyCount - 1];  // Most recent
    const autotuneHistoryEntry_t *h1 = &runtime->history[runtime->historyCount - 2];  // Previous
    
    // Check if score got worse
    bool scoreWorse = h0->score > h1->score * 1.05f;
    if (!scoreWorse) {
        return proposedDirection;  // Keep proposed direction if score improved
    }
    
    // Score got worse - check if this gain changed in the last iteration
    int currentVal, previousVal;
    switch (gainType) {
        case GAIN_ATTRIBUTION_P:
            currentVal = h0->pGain; previousVal = h1->pGain;
            break;
        case GAIN_ATTRIBUTION_D:
            currentVal = h0->dGain; previousVal = h1->dGain;
            break;
        case GAIN_ATTRIBUTION_F:
            currentVal = h0->fGain; previousVal = h1->fGain;
            break;
        case GAIN_ATTRIBUTION_I:
            currentVal = h0->iGain; previousVal = h1->iGain;
            break;
        default:
            return proposedDirection;
    }
    
    // If this gain changed and score got worse, reverse the direction
    if (currentVal != previousVal) {
        // We moved this gain and score got worse - try the opposite direction
        if (proposedDirection == ADJUST_INCREASE) {
            return ADJUST_DECREASE;
        } else if (proposedDirection == ADJUST_DECREASE) {
            return ADJUST_INCREASE;
        }
    }
    
    return proposedDirection;
}

// ============================================================================
// NEWTON'S METHOD - PREDICT OPTIMAL GAIN FROM HISTORY
// ============================================================================

// Calculate optimal F gain using linear regression on (fGain, velocityWeightedLag) history
// Returns the predicted optimal F value, or 0 if not enough history
static uint16_t calculateOptimalF_Newton(const autotuneRuntime_t *runtime, float targetLag)
{
    // Need at least 2 data points with DIFFERENT F values
    if (runtime->historyCount < F_TERM_MIN_HISTORY) {
        return 0;
    }
    
    // Collect (F, lag) pairs from history
    float sumF = 0, sumLag = 0, sumFF = 0, sumFLag = 0;
    int validPoints = 0;
    uint16_t lastF = 0;
    
    for (int i = 0; i < runtime->historyCount; i++) {
        const autotuneHistoryEntry_t *h = &runtime->history[i];
        float f = (float)h->fGain;
        float lag = h->velocityWeightedLag;
        
        // Skip invalid entries or duplicate F values (need variation)
        if (lag <= 0 || f <= 0) continue;
        if (validPoints > 0 && h->fGain == lastF) continue;  // Need different F values
        
        sumF += f;
        sumLag += lag;
        sumFF += f * f;
        sumFLag += f * lag;
        validPoints++;
        lastF = h->fGain;
    }
    
    // Need at least 2 points with different F values
    if (validPoints < 2) {
        return 0;
    }
    
    // Linear regression: lag = slope * F + intercept
    // slope = (n*sum(F*lag) - sum(F)*sum(lag)) / (n*sum(F^2) - sum(F)^2)
    float n = (float)validPoints;
    float denominator = n * sumFF - sumF * sumF;
    
    if (fabsf(denominator) < 0.001f) {
        return 0;  // F values too similar, can't calculate slope
    }
    
    float slope = (n * sumFLag - sumF * sumLag) / denominator;
    float intercept = (sumLag - slope * sumF) / n;
    
    // We expect slope < 0 (higher F = lower lag)
    // If slope is positive or near zero, the model doesn't fit well
    if (slope >= -0.001f) {
        return 0;  // Model doesn't make sense
    }
    
    // Solve for F where lag = targetLag:
    // targetLag = slope * F + intercept
    // F = (targetLag - intercept) / slope
    float optimalF = (targetLag - intercept) / slope;
    
    // Sanity check - keep within reasonable bounds (max 300 for most quads)
    optimalF = constrainf(optimalF, 20.0f, 300.0f);
    
    // Apply damping - only go 70% of the way to predicted optimal
    float currentF = (float)runtime->currentF;
    float dampedF = currentF + 0.70f * (optimalF - currentF);
    
    // CRITICAL: Limit max change per iteration to 50% of current value
    // This prevents wild jumps from bad regression fits
    float maxChange = currentF * 0.50f;
    if (maxChange < 20.0f) maxChange = 20.0f;  // At least 20 units
    
    float change = dampedF - currentF;
    if (fabsf(change) > maxChange) {
        dampedF = currentF + (change > 0 ? maxChange : -maxChange);
    }
    
    return (uint16_t)constrainf(dampedF, 20.0f, 300.0f);
}

// Calculate optimal P gain using regression on (pGain, overshoot) history
// For P: higher P = more overshoot (usually), target ~10% overshoot
static uint8_t calculateOptimalP_Newton(const autotuneRuntime_t *runtime, float targetOvershoot)
{
    if (runtime->historyCount < 2) {
        return 0;
    }
    
    float sumP = 0, sumOv = 0, sumPP = 0, sumPOv = 0;
    int validPoints = 0;
    uint8_t lastP = 0;
    
    for (int i = 0; i < runtime->historyCount; i++) {
        const autotuneHistoryEntry_t *h = &runtime->history[i];
        float p = (float)h->pGain;
        float ov = h->overshoot;
        
        if (p <= 0) continue;
        if (validPoints > 0 && h->pGain == lastP) continue;
        
        sumP += p;
        sumOv += ov;
        sumPP += p * p;
        sumPOv += p * ov;
        validPoints++;
        lastP = h->pGain;
    }
    
    if (validPoints < 2) {
        return 0;
    }
    
    float n = (float)validPoints;
    float denominator = n * sumPP - sumP * sumP;
    
    if (fabsf(denominator) < 0.001f) {
        return 0;
    }
    
    float slope = (n * sumPOv - sumP * sumOv) / denominator;
    float intercept = (sumOv - slope * sumP) / n;
    
    // We expect slope > 0 (higher P = more overshoot)
    if (slope <= 0.001f) {
        return 0;
    }
    
    float optimalP = (targetOvershoot - intercept) / slope;
    optimalP = constrainf(optimalP, 20.0f, 100.0f);
    
    float currentP = (float)runtime->currentP;
    float dampedP = currentP + 0.70f * (optimalP - currentP);
    
    // Limit max change per iteration to 30% of current value
    float maxChange = currentP * 0.30f;
    if (maxChange < 5.0f) maxChange = 5.0f;
    
    float change = dampedP - currentP;
    if (fabsf(change) > maxChange) {
        dampedP = currentP + (change > 0 ? maxChange : -maxChange);
    }
    
    return (uint8_t)constrainf(dampedP, 20.0f, 100.0f);
}

// Calculate optimal D gain using regression on (dGain, noise) history
// For D: higher D = more noise, target low noise
static uint8_t calculateOptimalD_Newton(const autotuneRuntime_t *runtime, float targetNoise)
{
    if (runtime->historyCount < 2) {
        return 0;
    }
    
    float sumD = 0, sumN = 0, sumDD = 0, sumDN = 0;
    int validPoints = 0;
    uint8_t lastD = 0;
    
    for (int i = 0; i < runtime->historyCount; i++) {
        const autotuneHistoryEntry_t *h = &runtime->history[i];
        float d = (float)h->dGain;
        float noise = h->noise;
        
        if (d <= 0) continue;
        if (validPoints > 0 && h->dGain == lastD) continue;
        
        sumD += d;
        sumN += noise;
        sumDD += d * d;
        sumDN += d * noise;
        validPoints++;
        lastD = h->dGain;
    }
    
    if (validPoints < 2) {
        return 0;
    }
    
    float n = (float)validPoints;
    float denominator = n * sumDD - sumD * sumD;
    
    if (fabsf(denominator) < 0.001f) {
        return 0;
    }
    
    float slope = (n * sumDN - sumD * sumN) / denominator;
    float intercept = (sumN - slope * sumD) / n;
    
    // We expect slope > 0 (higher D = more noise)
    if (slope <= 0.001f) {
        return 0;
    }
    
    float optimalD = (targetNoise - intercept) / slope;
    optimalD = constrainf(optimalD, 15.0f, 70.0f);
    
    float currentD = (float)runtime->currentD;
    float dampedD = currentD + 0.70f * (optimalD - currentD);
    
    // Limit max change per iteration to 30% of current value
    float maxChange = currentD * 0.30f;
    if (maxChange < 3.0f) maxChange = 3.0f;
    
    float change = dampedD - currentD;
    if (fabsf(change) > maxChange) {
        dampedD = currentD + (change > 0 ? maxChange : -maxChange);
    }
    
    return (uint8_t)constrainf(dampedD, 15.0f, 70.0f);
}

// Calculate optimal I gain using regression on (iGain, steadyStateError) history
// For I: lower I = more steady-state error (drift), higher I = less error but potential overshoot
// Target is minimal steady-state error (~2 deg/s)
static uint8_t calculateOptimalI_Newton(const autotuneRuntime_t *runtime, float targetError)
{
    if (runtime->historyCount < 2) {
        return 0;
    }
    
    float sumI = 0, sumErr = 0, sumII = 0, sumIErr = 0;
    int validPoints = 0;
    uint8_t lastI = 0;
    
    for (int i = 0; i < runtime->historyCount; i++) {
        const autotuneHistoryEntry_t *h = &runtime->history[i];
        float ig = (float)h->iGain;
        float err = h->steadyStateError;
        
        if (ig <= 0) continue;
        if (validPoints > 0 && h->iGain == lastI) continue;  // Skip duplicates
        
        sumI += ig;
        sumErr += err;
        sumII += ig * ig;
        sumIErr += ig * err;
        validPoints++;
        lastI = h->iGain;
    }
    
    if (validPoints < 2) {
        return 0;
    }
    
    float n = (float)validPoints;
    float denominator = n * sumII - sumI * sumI;
    
    if (fabsf(denominator) < 0.001f) {
        return 0;
    }
    
    float slope = (n * sumIErr - sumI * sumErr) / denominator;
    float intercept = (sumErr - slope * sumI) / n;
    
    // We expect slope < 0 (higher I = lower steady-state error)
    // If slope is positive or near zero, the model doesn't fit well
    if (slope >= -0.001f) {
        return 0;
    }
    
    // Solve for I where error = targetError
    float optimalI = (targetError - intercept) / slope;
    optimalI = constrainf(optimalI, 30.0f, 120.0f);
    
    float currentI = (float)runtime->currentI;
    float dampedI = currentI + 0.70f * (optimalI - currentI);
    
    // Limit max change per iteration to 30% of current value
    float maxChange = currentI * 0.30f;
    if (maxChange < 5.0f) maxChange = 5.0f;
    
    float change = dampedI - currentI;
    if (fabsf(change) > maxChange) {
        dampedI = currentI + (change > 0 ? maxChange : -maxChange);
    }
    
    return (uint8_t)constrainf(dampedI, 30.0f, 120.0f);
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
    
    // Get smart directions that consider history (may reverse if last change hurt)
    autotuneAdjustDir_e smartPDir = getSmartDirection(runtime, GAIN_ATTRIBUTION_P, attribution->pDirection);
    autotuneAdjustDir_e smartDDir = getSmartDirection(runtime, GAIN_ATTRIBUTION_D, attribution->dDirection);
    autotuneAdjustDir_e smartFDir = getSmartDirection(runtime, GAIN_ATTRIBUTION_F, attribution->fDirection);
    autotuneAdjustDir_e smartIDir = getSmartDirection(runtime, GAIN_ATTRIBUTION_I, attribution->iDirection);
    
    // Apply primary adjustment
    switch (attribution->primary) {
        case GAIN_ATTRIBUTION_P:
            {
                // Try Newton's method first - use term-specific target
                float targetOvershoot = getTargetForMetric(METRIC_OVERSHOOT);
                uint8_t newtonP = calculateOptimalP_Newton(runtime, targetOvershoot);
                
                if (newtonP > 0 && newtonP != runtime->currentP) {
                    // Newton's method gave us a prediction - use it!
                    bool increasing = newtonP > runtime->currentP;
                    runtime->currentP = newtonP;
                    if (reasonCode) {
                        *reasonCode = increasing ? REASON_PID_SLUGGISH_P_UP : REASON_PID_OVERSHOOT_P_DOWN;
                    }
                } else {
                    // Fall back to step-based adjustment
                    int16_t pDelta = (int16_t)(runtime->currentP * stepPercent / 100.0f);
                    if (pDelta < 1) pDelta = 1;
                    
                    if (smartPDir == ADJUST_INCREASE) {
                        runtime->currentP = MIN(GAIN_MAX_VALUE, runtime->currentP + pDelta);
                        if (reasonCode) {
                            *reasonCode = REASON_PID_SLUGGISH_P_UP;  // Overdamped/sluggish -> raise P
                        }
                    } else if (smartPDir == ADJUST_DECREASE) {
                        runtime->currentP = MAX(GAIN_MIN_VALUE, runtime->currentP - pDelta);
                        if (reasonCode) {
                            if (responseClass == RESPONSE_UNDERDAMPED) {
                                *reasonCode = REASON_PID_OVERSHOOT_P_DOWN;  // Overshoot/oscillation
                            } else {
                                *reasonCode = REASON_PID_OSCILLATION_P_DOWN;  // Default for P down
                            }
                        }
                    }
                }
                
                SET_GAIN_P(axis, runtime->currentP);
                gainsChanged = true;
            }
            break;
            
        case GAIN_ATTRIBUTION_D:
            {
                // Try Newton's method first - use term-specific target (oscillation)
                float targetOscillation = getTargetForMetric(METRIC_OSCILLATION);
                uint8_t newtonD = calculateOptimalD_Newton(runtime, targetOscillation);
                
                if (newtonD > 0 && newtonD != runtime->currentD) {
                    // Newton's method gave us a prediction
                    bool increasing = newtonD > runtime->currentD;
                    runtime->currentD = newtonD;
                    if (reasonCode) {
                        *reasonCode = increasing ? REASON_PID_OSCILLATION_D_UP : REASON_PID_NOISE_D_DOWN;
                    }
                } else {
                    // Fall back to step-based adjustment
                    int16_t dDelta = (int16_t)(runtime->currentD * stepPercent / 100.0f);
                    if (dDelta < 1) dDelta = 1;
                    
                    if (smartDDir == ADJUST_INCREASE) {
                        runtime->currentD = MIN(GAIN_MAX_VALUE, runtime->currentD + dDelta);
                        if (reasonCode) {
                            if (responseClass == RESPONSE_UNDERDAMPED) {
                                *reasonCode = REASON_PID_OSCILLATION_D_UP;  // Oscillation -> more damping
                            } else {
                                *reasonCode = REASON_PID_OVERSHOOT_D_UP;  // Overshoot -> more damping
                            }
                        }
                    } else if (smartDDir == ADJUST_DECREASE) {
                        runtime->currentD = MAX(GAIN_MIN_VALUE, runtime->currentD - dDelta);
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
                }
                
                SET_GAIN_D(axis, runtime->currentD);
                gainsChanged = true;
            }
            break;
            
        case GAIN_ATTRIBUTION_I:
            {
                // Try Newton's method first - use term-specific target (long-term error)
                float targetError = getTargetForMetric(METRIC_LONG_TERM_ERROR);
                uint8_t newtonI = calculateOptimalI_Newton(runtime, targetError);
                
                if (newtonI > 0 && newtonI != runtime->currentI) {
                    // Newton's method gave us a prediction - use it!
                    runtime->currentI = newtonI;
                    if (reasonCode) {
                        *reasonCode = (newtonI > runtime->history[runtime->historyCount - 1].iGain) 
                            ? REASON_PID_DRIFT_I_UP : REASON_PID_BOUNCEBACK_I_DOWN;
                    }
                } else {
                    // Fall back to step-based adjustment
                    int16_t iDelta = (int16_t)(runtime->currentI * stepPercent / 100.0f);
                    if (iDelta < 1) iDelta = 1;
                    
                    if (smartIDir == ADJUST_INCREASE) {
                        runtime->currentI = MIN(GAIN_MAX_VALUE, runtime->currentI + iDelta);
                        if (reasonCode) {
                            *reasonCode = REASON_PID_DRIFT_I_UP;  // Drift detected, raising I
                        }
                    } else if (smartIDir == ADJUST_DECREASE) {
                        runtime->currentI = MAX(GAIN_MIN_VALUE, runtime->currentI - iDelta);
                        if (reasonCode) {
                            *reasonCode = REASON_PID_BOUNCEBACK_I_DOWN;  // Bounceback or slow osc, lowering I
                        }
                    }
                }
                
                SET_GAIN_I(axis, runtime->currentI);
                gainsChanged = true;
            }
            break;
            
        case GAIN_ATTRIBUTION_F:
            {
                // Try Newton's method first - F-term uses velocityWeightedLag metric
                // The target is F_TERM_TARGET_LAG (near zero lag during rapid stick moves)
                uint16_t newtonF = calculateOptimalF_Newton(runtime, F_TERM_TARGET_LAG);
                
                if (newtonF > 0 && newtonF != runtime->currentF) {
                    // Newton's method gave us a prediction - use it!
                    runtime->currentF = newtonF;
                    if (reasonCode) {
                        *reasonCode = (newtonF > runtime->history[runtime->historyCount - 1].fGain) 
                            ? REASON_PID_LAG_F_UP : REASON_PID_LEAD_F_DOWN;
                    }
                } else {
                    // Fall back to step-based adjustment
                    int16_t fDelta = (int16_t)(runtime->currentF * stepPercent / 100.0f);
                    if (fDelta < 5) fDelta = 5;
                    
                    if (smartFDir == ADJUST_INCREASE) {
                        runtime->currentF = MIN(2000, runtime->currentF + fDelta);
                        if (reasonCode) {
                            *reasonCode = REASON_PID_LAG_F_UP;  // Stick lag, raising F
                        }
                    } else if (smartFDir == ADJUST_DECREASE) {
                        runtime->currentF = MAX(0, runtime->currentF - fDelta);
                        if (reasonCode) {
                            *reasonCode = REASON_PID_LEAD_F_DOWN;  // Gyro leading stick, lowering F
                        }
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
    // Use Newton's method when we have history, otherwise fall back to step-based
    if (attribution->secondary != GAIN_ATTRIBUTION_NONE && 
        attribution->secondary != attribution->primary) {
        
        float secondaryStep = stepPercent * 0.5f;  // Half step for secondary fallback
        
        switch (attribution->secondary) {
            case GAIN_ATTRIBUTION_D:
                if (smartDDir != ADJUST_NONE) {
                    // Try Newton's method first - use term-specific target
                    uint8_t newtonD = calculateOptimalD_Newton(runtime, getTargetForMetric(METRIC_OSCILLATION));
                    if (newtonD > 0 && newtonD != runtime->currentD) {
                        runtime->currentD = newtonD;
                    } else {
                        // Fall back to step-based
                        int16_t dDelta = (int16_t)(runtime->currentD * secondaryStep / 100.0f);
                        if (dDelta < 1) dDelta = 1;
                        
                        if (smartDDir == ADJUST_INCREASE) {
                            runtime->currentD = MIN(GAIN_MAX_VALUE, runtime->currentD + dDelta);
                        } else {
                            runtime->currentD = MAX(GAIN_MIN_VALUE, runtime->currentD - dDelta);
                        }
                    }
                    SET_GAIN_D(axis, runtime->currentD);
                    gainsChanged = true;
                }
                break;
                
            case GAIN_ATTRIBUTION_P:
                if (smartPDir != ADJUST_NONE) {
                    // Try Newton's method first - use term-specific target
                    uint8_t newtonP = calculateOptimalP_Newton(runtime, getTargetForMetric(METRIC_OVERSHOOT));
                    if (newtonP > 0 && newtonP != runtime->currentP) {
                        runtime->currentP = newtonP;
                    } else {
                        // Fall back to step-based
                        int16_t pDelta = (int16_t)(runtime->currentP * secondaryStep / 100.0f);
                        if (pDelta < 1) pDelta = 1;
                        
                        if (smartPDir == ADJUST_INCREASE) {
                            runtime->currentP = MIN(GAIN_MAX_VALUE, runtime->currentP + pDelta);
                        } else {
                            runtime->currentP = MAX(GAIN_MIN_VALUE, runtime->currentP - pDelta);
                        }
                    }
                    SET_GAIN_P(axis, runtime->currentP);
                    gainsChanged = true;
                }
                break;
                
            case GAIN_ATTRIBUTION_F:
                if (smartFDir != ADJUST_NONE) {
                    // Try Newton's method first
                    uint16_t newtonF = calculateOptimalF_Newton(runtime, F_TERM_TARGET_LAG);
                    if (newtonF > 0 && newtonF != runtime->currentF) {
                        runtime->currentF = newtonF;
                    } else {
                        // Fall back to step-based
                        int16_t fDelta = (int16_t)(runtime->currentF * secondaryStep / 100.0f);
                        if (fDelta < 3) fDelta = 3;
                        
                        if (smartFDir == ADJUST_INCREASE) {
                            runtime->currentF = MIN(2000, runtime->currentF + fDelta);
                        } else {
                            runtime->currentF = MAX(0, runtime->currentF - fDelta);
                        }
                    }
                    SET_GAIN_F(axis, runtime->currentF);
                    gainsChanged = true;
                }
                break;
                
            case GAIN_ATTRIBUTION_I:
                if (smartIDir != ADJUST_NONE) {
                    // Try Newton's method first - use term-specific target
                    uint8_t newtonI = calculateOptimalI_Newton(runtime, getTargetForMetric(METRIC_LONG_TERM_ERROR));
                    if (newtonI > 0 && newtonI != runtime->currentI) {
                        runtime->currentI = newtonI;
                    } else {
                        // Fall back to step-based
                        int16_t iDelta = (int16_t)(runtime->currentI * secondaryStep / 100.0f);
                        if (iDelta < 1) iDelta = 1;
                        
                        if (smartIDir == ADJUST_INCREASE) {
                            runtime->currentI = MIN(GAIN_MAX_VALUE, runtime->currentI + iDelta);
                        } else {
                            runtime->currentI = MAX(GAIN_MIN_VALUE, runtime->currentI - iDelta);
                        }
                    }
                    SET_GAIN_I(axis, runtime->currentI);
                    gainsChanged = true;
                }
                break;
                
            default:
                break;
        }
    }
    
    // Apply additional non-conflicting adjustments at tertiary priority
    // Use Newton's method when available, otherwise use 25% step fallback
    // This allows simultaneous P+D+F corrections when all are needed
    float tertiaryStep = stepPercent * 0.25f;
    
    // Check P if not already adjusted
    if (attribution->primary != GAIN_ATTRIBUTION_P && 
        attribution->secondary != GAIN_ATTRIBUTION_P &&
        smartPDir != ADJUST_NONE) {
        // Try Newton's method first - use term-specific target
        uint8_t newtonP = calculateOptimalP_Newton(runtime, getTargetForMetric(METRIC_OVERSHOOT));
        if (newtonP > 0 && newtonP != runtime->currentP) {
            runtime->currentP = newtonP;
            SET_GAIN_P(axis, runtime->currentP);
            gainsChanged = true;
        } else {
            int16_t pDelta = (int16_t)(runtime->currentP * tertiaryStep / 100.0f);
            if (pDelta >= 1) {
                if (smartPDir == ADJUST_INCREASE) {
                    runtime->currentP = MIN(GAIN_MAX_VALUE, runtime->currentP + pDelta);
                } else {
                    runtime->currentP = MAX(GAIN_MIN_VALUE, runtime->currentP - pDelta);
                }
                SET_GAIN_P(axis, runtime->currentP);
                gainsChanged = true;
            }
        }
    }
    
    // Check D if not already adjusted
    if (attribution->primary != GAIN_ATTRIBUTION_D && 
        attribution->secondary != GAIN_ATTRIBUTION_D &&
        smartDDir != ADJUST_NONE) {
        // Try Newton's method first - use term-specific target
        uint8_t newtonD = calculateOptimalD_Newton(runtime, getTargetForMetric(METRIC_OSCILLATION));
        if (newtonD > 0 && newtonD != runtime->currentD) {
            runtime->currentD = newtonD;
            SET_GAIN_D(axis, runtime->currentD);
            gainsChanged = true;
        } else {
            int16_t dDelta = (int16_t)(runtime->currentD * tertiaryStep / 100.0f);
            if (dDelta >= 1) {
                if (smartDDir == ADJUST_INCREASE) {
                    runtime->currentD = MIN(GAIN_MAX_VALUE, runtime->currentD + dDelta);
                } else {
                    runtime->currentD = MAX(GAIN_MIN_VALUE, runtime->currentD - dDelta);
                }
                SET_GAIN_D(axis, runtime->currentD);
                gainsChanged = true;
            }
        }
    }
    
    // Check F if not already adjusted
    if (attribution->primary != GAIN_ATTRIBUTION_F && 
        attribution->secondary != GAIN_ATTRIBUTION_F &&
        smartFDir != ADJUST_NONE) {
        // Try Newton's method first
        uint16_t newtonF = calculateOptimalF_Newton(runtime, F_TERM_TARGET_LAG);
        if (newtonF > 0 && newtonF != runtime->currentF) {
            runtime->currentF = newtonF;
            SET_GAIN_F(axis, runtime->currentF);
            gainsChanged = true;
        } else {
            int16_t fDelta = (int16_t)(runtime->currentF * tertiaryStep / 100.0f);
            if (fDelta >= 2) {
                if (smartFDir == ADJUST_INCREASE) {
                    runtime->currentF = MIN(2000, runtime->currentF + fDelta);
                } else {
                    runtime->currentF = MAX(0, runtime->currentF - fDelta);
                }
                SET_GAIN_F(axis, runtime->currentF);
                gainsChanged = true;
            }
        }
    }
    
    // If we changed any gains, disable slider mode so changes persist
    // and reinitialize PID controller to use the new coefficients
    if (gainsChanged) {
        disableSliderMode();
        pidInitConfig(currentPidProfile);  // Apply gains to PID runtime
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
    entry->velocityWeightedLag = runtime->metrics.velocityWeightedLag;  // Key F-term metric
    entry->steadyStateError = runtime->metrics.steadyStateError;        // Key I-term metric
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
    
    pidInitConfig(currentPidProfile);  // Apply gains to PID runtime
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
