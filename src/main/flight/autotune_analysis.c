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
#include "flight/autotune_types.h"
#include "flight/autotune_analysis.h"
#include "flight/dyn_notch_filter.h"

// Resonance detection thresholds
#define RESONANCE_MIN_FREQ_HZ       30.0f   // Below this is acceptable for motors
#define RESONANCE_MAX_FREQ_HZ      500.0f   // Above this is usually handled by LPF anyway

// ============================================================================
// DAMPING RATIO CALCULATION
// ============================================================================

// Calculate damping ratio from overshoot percentage
// Uses standard control theory formula: ζ = -ln(OS/100) / sqrt(π² + ln²(OS/100))
// Returns value > 1.0 for overdamped (no overshoot)
static float calculateDampingFromOvershoot(float overshootPercent, float riseTimeMs)
{
    if (overshootPercent <= 0.0f) {
        // No overshoot = overdamped
        // Estimate how overdamped based on rise time
        // A well-tuned quad should have rise time < 40ms
        // Rise time > 60ms is sluggish (overdamped)
        // Map rise time to damping ratio > 1.0
        if (riseTimeMs > 100.0f) {
            return 2.0f;  // Very overdamped
        } else if (riseTimeMs > 60.0f) {
            return 1.5f;  // Moderately overdamped
        } else if (riseTimeMs > 40.0f) {
            return 1.2f;  // Slightly overdamped  
        }
        return 1.0f;  // Critically damped (good rise time, no overshoot)
    }
    
    // Clamp to reasonable range to avoid math issues
    if (overshootPercent > 100.0f) {
        overshootPercent = 100.0f;
    }
    
    float os = overshootPercent / 100.0f;
    float lnOs = logf(os);
    
    // ζ = -ln(OS) / sqrt(π² + ln²(OS))
    float denominator = sqrtf(M_PIf * M_PIf + lnOs * lnOs);
    if (denominator < 0.001f) {
        return 1.0f;
    }
    
    return -lnOs / denominator;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

float autotuneCalculateRms(const float *samples, uint16_t start, uint16_t end)
{
    if (end <= start || samples == NULL) {
        return 0.0f;
    }
    
    float sumSquares = 0.0f;
    uint16_t count = 0;
    
    for (uint16_t i = start; i < end; i++) {
        sumSquares += samples[i] * samples[i];
        count++;
    }
    
    if (count == 0) {
        return 0.0f;
    }
    
    return sqrtf(sumSquares / count);
}

float autotuneFindPeak(const float *samples, uint16_t start, uint16_t end, uint16_t *peakIndex)
{
    if (end <= start || samples == NULL) {
        if (peakIndex) *peakIndex = start;
        return 0.0f;
    }
    
    float maxVal = fabsf(samples[start]);
    uint16_t maxIdx = start;
    
    for (uint16_t i = start + 1; i < end; i++) {
        float absVal = fabsf(samples[i]);
        if (absVal > maxVal) {
            maxVal = absVal;
            maxIdx = i;
        }
    }
    
    if (peakIndex) *peakIndex = maxIdx;
    return samples[maxIdx];  // Return signed value
}

float autotuneDetectOscillationFreq(const float *samples, uint16_t count, uint16_t sampleRateHz)
{
    if (count < 10 || samples == NULL) {
        return 0.0f;
    }
    
    // Count zero crossings
    uint16_t zeroCrossings = 0;
    float prevSample = samples[0];
    
    for (uint16_t i = 1; i < count; i++) {
        if ((prevSample > 0 && samples[i] <= 0) || (prevSample < 0 && samples[i] >= 0)) {
            zeroCrossings++;
        }
        prevSample = samples[i];
    }
    
    // Two zero crossings per cycle
    float durationSec = (float)count / sampleRateHz;
    float freqHz = (zeroCrossings / 2.0f) / durationSec;
    
    return freqHz;
}

// ============================================================================
// RESPONSE ANALYSIS
// ============================================================================

// Find the index where response reaches a threshold of the setpoint
static uint16_t findThresholdCrossing(
    const float *response,
    const float *setpoint,
    uint16_t count,
    float thresholdPercent,
    uint16_t startIdx
)
{
    if (count < 2) return 0;
    
    // Find target value (use max setpoint as target)
    float target = 0.0f;
    for (uint16_t i = startIdx; i < count; i++) {
        if (fabsf(setpoint[i]) > fabsf(target)) {
            target = setpoint[i];
        }
    }
    
    if (fabsf(target) < 1.0f) {
        return startIdx;  // No significant setpoint
    }
    
    float threshold = target * thresholdPercent / 100.0f;
    
    // Find first crossing
    for (uint16_t i = startIdx; i < count; i++) {
        if ((target > 0 && response[i] >= threshold) ||
            (target < 0 && response[i] <= threshold)) {
            return i;
        }
    }
    
    return count - 1;  // Never crossed
}

void autotuneAnalyzeResponse(
    const float *gyroSamples,
    const float *setpointSamples,
    const float *dtermSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz,
    autotuneMetrics_t *metricsOut
)
{
    memset(metricsOut, 0, sizeof(autotuneMetrics_t));
    
    if (sampleCount < 20 || gyroSamples == NULL || setpointSamples == NULL) {
        metricsOut->isValid = false;
        return;
    }
    
    const float samplePeriodMs = 1000.0f / sampleRateHz;
    
    // Find the start of the maneuver (first significant setpoint)
    uint16_t maneuverStart = 0;
    for (uint16_t i = 0; i < sampleCount; i++) {
        if (fabsf(setpointSamples[i]) > 50.0f) {
            maneuverStart = i;
            break;
        }
    }
    
    // Find peak setpoint value and direction
    float peakSetpoint = 0.0f;
    uint16_t peakSetpointIdx = maneuverStart;
    for (uint16_t i = maneuverStart; i < sampleCount; i++) {
        if (fabsf(setpointSamples[i]) > fabsf(peakSetpoint)) {
            peakSetpoint = setpointSamples[i];
            peakSetpointIdx = i;
        }
    }
    UNUSED(peakSetpointIdx);  // May be used in future analysis
    
    if (fabsf(peakSetpoint) < 10.0f) {
        metricsOut->isValid = false;
        return;
    }
    
    // Calculate rise time (10% to 90%)
    uint16_t idx10 = findThresholdCrossing(gyroSamples, setpointSamples, sampleCount, 10.0f, maneuverStart);
    uint16_t idx50 = findThresholdCrossing(gyroSamples, setpointSamples, sampleCount, 50.0f, maneuverStart);
    uint16_t idx90 = findThresholdCrossing(gyroSamples, setpointSamples, sampleCount, 90.0f, maneuverStart);
    
    if (idx90 > idx10) {
        metricsOut->riseTimeMs = (idx90 - idx10) * samplePeriodMs;
    }
    
    // Calculate overshoot
    // Find peak response after reaching setpoint
    float peakResponse = 0.0f;
    uint16_t peakResponseIdx = 0;
    for (uint16_t i = idx90; i < MIN(sampleCount, idx90 + 50); i++) {
        if (fabsf(gyroSamples[i]) > fabsf(peakResponse)) {
            peakResponse = gyroSamples[i];
            peakResponseIdx = i;
        }
    }
    
    // Traditional overshoot = (peak - setpoint) / setpoint * 100
    // This looks at peak gyro shortly after reaching target
    float traditionalOvershoot = 0.0f;
    if (fabsf(peakSetpoint) > 1.0f) {
        traditionalOvershoot = (fabsf(peakResponse) - fabsf(peakSetpoint)) / fabsf(peakSetpoint) * 100.0f;
        traditionalOvershoot = MAX(0.0f, traditionalOvershoot);
    }
    
    // Also calculate INSTANTANEOUS overshoot - max(|gyro| - |setpoint|) at any point
    // This catches oscillation around setpoint during sustained maneuvers
    float maxInstantOvershoot = 0.0f;
    for (uint16_t i = maneuverStart; i < sampleCount; i++) {
        // Only count when setpoint is significant (actual maneuver, not settling)
        if (fabsf(setpointSamples[i]) > 50.0f) {
            // Signed comparison: is gyro exceeding setpoint in the same direction?
            float instantOvershoot;
            if (setpointSamples[i] > 0) {
                instantOvershoot = gyroSamples[i] - setpointSamples[i];  // Positive overshoot
            } else {
                instantOvershoot = setpointSamples[i] - gyroSamples[i];  // Negative setpoint, check the other way
            }
            
            if (instantOvershoot > 0) {
                float overshootPercent = instantOvershoot / fabsf(setpointSamples[i]) * 100.0f;
                maxInstantOvershoot = MAX(maxInstantOvershoot, overshootPercent);
            }
        }
    }
    
    // Use the LARGER of traditional or instantaneous overshoot
    // This ensures we catch underdamped behavior whether it's at onset or during tracking
    metricsOut->overshootPercent = MAX(traditionalOvershoot, maxInstantOvershoot);
    
    // Calculate damping ratio from overshoot AND rise time
    // Rise time is critical for detecting overdamped responses (no overshoot but slow)
    metricsOut->dampingRatio = calculateDampingFromOvershoot(metricsOut->overshootPercent, metricsOut->riseTimeMs);
    
    // Calculate tracking error (after settling region)
    uint16_t settleStart = MIN(peakResponseIdx + 20, sampleCount);
    uint16_t settleEnd = MIN(settleStart + 30, sampleCount);
    
    float errorSum = 0.0f;
    uint16_t errorCount = 0;
    for (uint16_t i = settleStart; i < settleEnd; i++) {
        errorSum += fabsf(gyroSamples[i] - setpointSamples[i]);
        errorCount++;
    }
    
    if (errorCount > 0) {
        metricsOut->trackingError = errorSum / errorCount;
    }
    
    // Calculate settling time (time to stay within 5% band)
    float settlingBand = fabsf(peakSetpoint) * 0.05f;
    metricsOut->settlingTimeMs = 0;
    
    for (uint16_t i = idx90; i < sampleCount; i++) {
        float error = fabsf(gyroSamples[i] - setpointSamples[i]);
        if (error > settlingBand) {
            metricsOut->settlingTimeMs = (i - idx90) * samplePeriodMs;
        }
    }
    
    // Calculate oscillation amplitude (peak-to-peak of TRACKING ERROR, not raw gyro)
    // This is the true measure of underdamped behavior - gyro oscillating around setpoint
    if (peakResponseIdx + 10 < sampleCount) {
        float minError = 0;
        float maxError = 0;
        uint16_t signChanges = 0;
        float prevError = 0;
        
        for (uint16_t i = peakResponseIdx; i < sampleCount; i++) {
            float error = gyroSamples[i] - setpointSamples[i];
            minError = MIN(minError, error);
            maxError = MAX(maxError, error);
            
            // Count sign changes in error (oscillation frequency indicator)
            if (i > peakResponseIdx && error * prevError < 0) {
                signChanges++;
            }
            prevError = error;
        }
        
        // Oscillation = peak-to-peak error around setpoint
        metricsOut->oscillationAmplitude = maxError - minError;
        
        // High sign changes = high frequency oscillation (underdamped)
        // Scale by window length to get changes per 100 samples
        uint16_t windowLen = sampleCount - peakResponseIdx;
        float signChangeRate = (float)signChanges / (float)windowLen * 100.0f;
        
        // If many sign changes AND significant amplitude, definitely underdamped
        // Add sign change rate to amplitude to boost detection
        if (signChangeRate > 10.0f) {  // More than 10 sign changes per 100 samples
            metricsOut->oscillationAmplitude = MAX(metricsOut->oscillationAmplitude, 
                                                    signChangeRate * 3.0f);  // Scale up
        }
    }
    
    // Calculate oscillation frequency from settling region
    if (settleEnd > settleStart + 5) {
        metricsOut->oscillationFreqHz = autotuneDetectOscillationFreq(
            &gyroSamples[settleStart], 
            settleEnd - settleStart, 
            sampleRateHz
        );
    }
    
    // Calculate noise RMS (from quiet period before maneuver or after settling)
    if (maneuverStart > 10) {
        metricsOut->noiseRms = autotuneCalculateRms(gyroSamples, 0, maneuverStart - 1);
    } else if (settleEnd + 10 < sampleCount) {
        metricsOut->noiseRms = autotuneCalculateRms(gyroSamples, settleEnd, sampleCount);
    }
    
    // D-term oscillation analysis
    if (dtermSamples != NULL && settleEnd > settleStart) {
        float dtermOsc = autotuneCalculateRms(dtermSamples, settleStart, settleEnd);
        // Factor into oscillation amplitude
        metricsOut->oscillationAmplitude = MAX(metricsOut->oscillationAmplitude, dtermOsc * 2.0f);
    }
    
    // =========================================================================
    // I-TERM SPECIFIC METRICS
    // =========================================================================
    
    // Drift rate: look for consistent drift after settling
    // Calculate slope of gyro in late settling region
    if (settleEnd + 20 < sampleCount) {
        uint16_t driftStart = settleEnd;
        uint16_t driftEnd = MIN(driftStart + 40, sampleCount);
        
        // Linear regression for drift
        float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        uint16_t n = driftEnd - driftStart;
        
        for (uint16_t i = driftStart; i < driftEnd; i++) {
            float x = (float)(i - driftStart) * samplePeriodMs;
            float y = gyroSamples[i];
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }
        
        if (n > 2 && (n * sumX2 - sumX * sumX) > 0.001f) {
            // Slope in deg/s per ms, convert to deg/s per second
            metricsOut->driftRate = ((n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX)) * 1000.0f;
        }
    }
    
    // Bounceback: look for overshoot in opposite direction after primary overshoot
    if (peakResponseIdx + 20 < sampleCount) {
        float primaryDirection = (peakResponse > peakSetpoint) ? 1.0f : -1.0f;
        float bounceMax = 0.0f;
        
        // Look for peak in opposite direction
        for (uint16_t i = peakResponseIdx + 10; i < MIN(peakResponseIdx + 60, sampleCount); i++) {
            float deviation = (gyroSamples[i] - peakSetpoint) * (-primaryDirection);
            if (deviation > bounceMax) {
                bounceMax = deviation;
            }
        }
        
        if (fabsf(peakSetpoint) > 1.0f) {
            metricsOut->bouncebackPercent = bounceMax / fabsf(peakSetpoint) * 100.0f;
        }
    }
    
    // Slow oscillation: look for low-frequency oscillation (< 5 Hz) indicating I-term windup
    if (settleEnd > settleStart + 20) {
        metricsOut->slowOscillationHz = autotuneDetectOscillationFreq(
            &gyroSamples[settleStart],
            MIN(80, settleEnd - settleStart),  // Longer window for low freq
            sampleRateHz
        );
        // Only care about slow oscillations
        if (metricsOut->slowOscillationHz > 8.0f) {
            metricsOut->slowOscillationHz = 0.0f;  // Not a slow oscillation issue
        }
    }
    
    // =========================================================================
    // F-TERM (FEEDFORWARD) SPECIFIC METRICS
    // =========================================================================
    
    // Initial response delay: time from setpoint change to gyro starting to move
    metricsOut->initialResponseDelay = (float)(idx10) * samplePeriodMs;
    
    // Calculate peak stick velocity and velocity-weighted tracking error
    // F-term is most important when stick is moving fast, so we weight errors by stick velocity
    float peakStickVelocity = 0.0f;
    float velocityWeightedErrorSum = 0.0f;
    float velocityWeightSum = 0.0f;
    
    // Analyze the onset portion (from maneuver start to peak setpoint)
    for (uint16_t i = maneuverStart + 1; i < peakSetpointIdx && i < sampleCount; i++) {
        // Calculate stick velocity (change in setpoint per sample)
        float stickDelta = fabsf(setpointSamples[i] - setpointSamples[i-1]);
        
        // Track peak velocity
        if (stickDelta > peakStickVelocity) {
            peakStickVelocity = stickDelta;
        }
        
        // Calculate tracking error at this instant
        float trackingErr = fabsf(setpointSamples[i]) - fabsf(gyroSamples[i]);
        
        // Weight the error by stick velocity - fast stick movements with ANY lag matter more
        // Use velocity squared to emphasize high-velocity regions
        float weight = stickDelta * stickDelta;
        velocityWeightedErrorSum += trackingErr * weight;
        velocityWeightSum += weight;
    }
    
    metricsOut->peakStickVelocity = peakStickVelocity;
    
    if (velocityWeightSum > 0.01f) {
        metricsOut->velocityWeightedLag = velocityWeightedErrorSum / velocityWeightSum;
    }
    
    // Stick lead error: during ramp-up, how much does stick lead gyro?
    // Compare setpoint to gyro in the 10-50% rise region
    if (idx50 > idx10 + 2) {
        float leadErrorSum = 0.0f;
        uint16_t leadCount = 0;
        
        for (uint16_t i = idx10; i < idx50; i++) {
            // Positive = gyro lagging stick, negative = gyro leading stick
            float error = fabsf(setpointSamples[i]) - fabsf(gyroSamples[i]);
            leadErrorSum += error;
            leadCount++;
        }
        
        if (leadCount > 0) {
            metricsOut->stickLeadError = leadErrorSum / leadCount;
        }
    }
    
    // Phase tracking: cross-correlation to find phase shift
    // Simplified: compare when gyro reaches 50% vs when setpoint did
    uint16_t setpoint50Idx = 0;
    for (uint16_t i = maneuverStart; i < peakSetpointIdx; i++) {
        if (fabsf(setpointSamples[i]) >= fabsf(peakSetpoint) * 0.5f) {
            setpoint50Idx = i;
            break;
        }
    }
    
    if (setpoint50Idx > 0 && idx50 > setpoint50Idx) {
        // Gyro 50% came after setpoint 50% - there's a phase lag
        metricsOut->stickTrackingPhase = (float)(idx50 - setpoint50Idx) * samplePeriodMs;
    } else if (setpoint50Idx > 0 && idx50 < setpoint50Idx) {
        // Gyro is leading! F might be too high
        metricsOut->stickTrackingPhase = -(float)(setpoint50Idx - idx50) * samplePeriodMs;
    }
    
    metricsOut->isValid = true;
}

// ============================================================================
// RESPONSE CLASSIFICATION
// ============================================================================

autotuneResponseClass_e autotuneClassifyResponse(const autotuneMetrics_t *metrics)
{
    if (!metrics->isValid) {
        return RESPONSE_UNKNOWN;
    }
    
    // Check for noise limit first - noise always takes priority
    if (metrics->noiseRms > NOISE_TARGET_RMS * 1.5f) {
        return RESPONSE_NOISY;
    }
    
    // Check for oscillations - indicates instability regardless of damping
    if (metrics->oscillationAmplitude > OSCILLATION_THRESHOLD) {
        return RESPONSE_UNDERDAMPED;
    }
    
    // Classification based on damping ratio (more accurate than overshoot alone)
    // Damping ratio ζ thresholds:
    //   ζ < 0.5:  Underdamped - bouncy, oscillatory
    //   ζ 0.5-0.7: Critical - snappy, slight overshoot OK for FPV
    //   ζ 0.7-1.0: Excellent - optimal damping, minimal overshoot
    //   ζ > 1.0:  Overdamped - sluggish, slow response
    
    if (metrics->dampingRatio < 0.5f) {
        return RESPONSE_UNDERDAMPED;  // Bouncy, oscillatory
    } else if (metrics->dampingRatio < 0.7f) {
        // Snappy response with acceptable overshoot - good for FPV
        // But also check rise time to ensure responsiveness
        if (metrics->riseTimeMs < 50.0f && metrics->noiseRms < NOISE_TARGET_RMS) {
            return RESPONSE_CRITICAL;
        }
        return RESPONSE_UNDERDAMPED;  // Rise time too slow despite damping
    } else if (metrics->dampingRatio <= 1.0f) {
        // Optimal damping range - check other quality factors
        if (metrics->riseTimeMs < 40.0f && metrics->noiseRms < NOISE_TARGET_RMS) {
            return RESPONSE_EXCELLENT;
        }
        // Good damping but other issues
        if (metrics->riseTimeMs > 50.0f) {
            return RESPONSE_OVERDAMPED;  // Rise time suggests sluggish
        }
        return RESPONSE_CRITICAL;  // Acceptable
    } else {
        // ζ > 1.0: Overdamped - sluggish response
        return RESPONSE_OVERDAMPED;
    }
}

// ============================================================================
// GAIN ATTRIBUTION
// ============================================================================

void autotuneAttributeGains(
    const autotuneMetrics_t *metrics,
    autotuneResponseClass_e responseClass,
    autotuneAttribution_t *attributionOut
)
{
    memset(attributionOut, 0, sizeof(autotuneAttribution_t));
    attributionOut->confidence = 0.5f;  // Default medium confidence
    
    // =========================================================================
    // PHASE 0: Check for oscillation FIRST
    // =========================================================================
    // If there's significant oscillation, this is a P/D issue, not an I issue.
    // Skip I-term drift check if oscillating - the "drift" is just one swing of the oscillation.
    
    bool hasSignificantOscillation = (metrics->oscillationAmplitude > OSCILLATION_THRESHOLD * 0.5f);
    
    // =========================================================================
    // PHASE 1: Check for clear I-term issues (only if NOT oscillating)
    // =========================================================================
    
    if (!hasSignificantOscillation) {
        // Drift after settling = I too low (not holding position)
        // Only valid if quad is NOT oscillating, otherwise "drift" is just oscillation
        if (fabsf(metrics->driftRate) > 20.0f) {  // >20 deg/s per second of drift
            attributionOut->primary = GAIN_ATTRIBUTION_I;
            attributionOut->iDirection = ADJUST_INCREASE;
            attributionOut->confidence = 0.7f;
            return;  // Clear I issue, handle it
        }
        
        // Bounceback = I too high (windup causing overshoot in opposite direction)
        if (metrics->bouncebackPercent > 30.0f) {
            attributionOut->primary = GAIN_ATTRIBUTION_I;
            attributionOut->iDirection = ADJUST_DECREASE;
            attributionOut->secondary = GAIN_ATTRIBUTION_D;
            attributionOut->dDirection = ADJUST_INCREASE;  // D can help damp the bounce
            attributionOut->confidence = 0.7f;
            return;
        }
        
        // Slow oscillation (<5Hz) = I windup / I too high
        if (metrics->slowOscillationHz > 0.5f && metrics->slowOscillationHz < 5.0f) {
            attributionOut->primary = GAIN_ATTRIBUTION_I;
            attributionOut->iDirection = ADJUST_DECREASE;
            attributionOut->confidence = 0.6f;
            return;
        }
    }
    
    // =========================================================================
    // PHASE 2: P/D attribution based on response class (CHECK FIRST!)
    // =========================================================================
    // P/D response must be acceptable BEFORE optimizing F-term tracking.
    // Otherwise F gets adjusted constantly while P/D are starved.
    
    switch (responseClass) {
        case RESPONSE_UNDERDAMPED:
            // Too much overshoot / oscillation - P/D issue, handle and return
            if (metrics->oscillationAmplitude > OSCILLATION_THRESHOLD) {
                // High frequency oscillation = D too low (can't damp it)
                if (metrics->oscillationFreqHz > 15.0f) {
                    attributionOut->primary = GAIN_ATTRIBUTION_D;
                    attributionOut->dDirection = ADJUST_INCREASE;
                    attributionOut->confidence = 0.8f;
                } else {
                    // Lower frequency oscillation could be P too high
                    attributionOut->primary = GAIN_ATTRIBUTION_P;
                    attributionOut->pDirection = ADJUST_DECREASE;
                    attributionOut->secondary = GAIN_ATTRIBUTION_D;
                    attributionOut->dDirection = ADJUST_INCREASE;
                    attributionOut->confidence = 0.6f;
                }
            } else {
                // Just overshoot without oscillation = P too high
                attributionOut->primary = GAIN_ATTRIBUTION_P;
                attributionOut->pDirection = ADJUST_DECREASE;
                attributionOut->confidence = 0.7f;
            }
            return;  // P/D needs work - don't fall through to F-term checks
            
        case RESPONSE_OVERDAMPED:
            // Too slow / sluggish - P/D issue, handle and return
            if (metrics->riseTimeMs > 60.0f) {
                // Very slow = P is definitely too low
                attributionOut->primary = GAIN_ATTRIBUTION_P;
                attributionOut->pDirection = ADJUST_INCREASE;
                attributionOut->confidence = 0.8f;
            } else if (metrics->initialResponseDelay > 10.0f) {
                // Slow initial response - P needs attention first
                attributionOut->primary = GAIN_ATTRIBUTION_P;
                attributionOut->pDirection = ADJUST_INCREASE;
                attributionOut->secondary = GAIN_ATTRIBUTION_F;
                attributionOut->fDirection = ADJUST_INCREASE;
                attributionOut->confidence = 0.6f;
            } else {
                // Moderately slow = could be D too high (over-damping)
                attributionOut->primary = GAIN_ATTRIBUTION_D;
                attributionOut->dDirection = ADJUST_DECREASE;
                attributionOut->secondary = GAIN_ATTRIBUTION_P;
                attributionOut->pDirection = ADJUST_INCREASE;
                attributionOut->confidence = 0.6f;
            }
            return;  // P/D needs work - don't fall through to F-term checks
            
        case RESPONSE_NOISY:
            // Noise limiting - D/filter issue
            attributionOut->primary = GAIN_ATTRIBUTION_D;
            attributionOut->dDirection = ADJUST_DECREASE;
            attributionOut->secondary = GAIN_ATTRIBUTION_FILTER;
            attributionOut->confidence = 0.7f;
            return;  // Handle noise before F-term optimization
            
        case RESPONSE_CRITICAL:
        case RESPONSE_EXCELLENT:
            // Good P/D response - fall through to check F-term tracking
            break;
            
        default:
            // Unknown response - try conservative P increase and return
            attributionOut->primary = GAIN_ATTRIBUTION_P;
            attributionOut->pDirection = ADJUST_INCREASE;
            attributionOut->confidence = 0.3f;
            return;
    }
    
    // =========================================================================
    // PHASE 3: F-term optimization (ONLY when P/D response is already good)
    // =========================================================================
    // If we get here, responseClass is EXCELLENT or CRITICAL, meaning P/D are acceptable.
    // Now we can safely optimize feedforward tracking without starving P/D.
    
    // Velocity-weighted lag detection
    // If sticks were moving fast (high velocity) and there's ANY measurable lag, F needs adjustment
    // This catches the "last 50% of onset" scenario where small lags matter during fast moves
    bool highVelocityManeuver = (metrics->peakStickVelocity > 15.0f);  // Fast stick movement
    
    if (highVelocityManeuver) {
        // During fast stick movements, be very sensitive to tracking lag
        // Even 5 deg/s weighted lag during high velocity is significant
        if (metrics->velocityWeightedLag > 5.0f) {
            // Lag during fast stick movement = F too low
            attributionOut->primary = GAIN_ATTRIBUTION_F;
            attributionOut->fDirection = ADJUST_INCREASE;
            // Higher confidence when velocity was high and lag is clear
            attributionOut->confidence = MIN(0.9f, 0.5f + metrics->velocityWeightedLag / 50.0f);
            return;
        }
        
        // Also check for gyro leading stick during fast movements
        if (metrics->velocityWeightedLag < -3.0f) {
            // Gyro leading stick during fast movement = F too high
            attributionOut->primary = GAIN_ATTRIBUTION_F;
            attributionOut->fDirection = ADJUST_DECREASE;
            attributionOut->confidence = 0.7f;
            return;
        }
    }
    
    // Checks for moderate-speed maneuvers
    // Significant stick lead error = F too low (gyro lagging stick input)
    if (metrics->stickLeadError > 30.0f && metrics->riseTimeMs < 40.0f) {
        // Good P/D (fast rise), but gyro lags stick = needs more feedforward
        attributionOut->primary = GAIN_ATTRIBUTION_F;
        attributionOut->fDirection = ADJUST_INCREASE;
        attributionOut->confidence = 0.7f;
        return;
    }
    
    // Negative stick tracking phase (gyro leading stick) = F too high
    if (metrics->stickTrackingPhase < -3.0f) {  // Gyro leading by >3ms
        attributionOut->primary = GAIN_ATTRIBUTION_F;
        attributionOut->fDirection = ADJUST_DECREASE;
        attributionOut->confidence = 0.6f;
        return;
    }
    
    // Large positive phase lag with low overshoot = could use more F
    if (metrics->stickTrackingPhase > 10.0f && metrics->overshootPercent < 10.0f) {
        attributionOut->primary = GAIN_ATTRIBUTION_F;
        attributionOut->fDirection = ADJUST_INCREASE;
        attributionOut->confidence = 0.5f;
        return;
    }
    
    // =========================================================================
    // PHASE 4: Fine-tuning for EXCELLENT/CRITICAL response
    // =========================================================================
    // Response is good, F-term is good - look for minor improvements
    
    if (metrics->steadyStateError > 5.0f) {
        // Could use a bit more I
        attributionOut->primary = GAIN_ATTRIBUTION_I;
        attributionOut->iDirection = ADJUST_INCREASE;
        attributionOut->confidence = 0.4f;
    } else if (metrics->stickLeadError > 15.0f) {
        // Could use a bit more F (lower threshold since we already checked above)
        attributionOut->primary = GAIN_ATTRIBUTION_F;
        attributionOut->fDirection = ADJUST_INCREASE;
        attributionOut->confidence = 0.4f;
    } else {
        // Try pushing P slightly for more responsiveness
        attributionOut->primary = GAIN_ATTRIBUTION_P;
        attributionOut->pDirection = ADJUST_INCREASE;
        attributionOut->confidence = 0.3f;  // Very conservative
    }
    
    // =========================================================================
    // PHASE 4: Add secondary adjustments if not already set
    // =========================================================================
    
    if (attributionOut->secondary == GAIN_ATTRIBUTION_NONE) {
        // Steady state error suggests I needs attention
        if (metrics->steadyStateError > 10.0f) {
            attributionOut->secondary = GAIN_ATTRIBUTION_I;
            attributionOut->iDirection = ADJUST_INCREASE;
        }
        // Tracking error with good rise time suggests F needs attention
        else if (metrics->trackingError > 20.0f && metrics->riseTimeMs < 35.0f) {
            attributionOut->secondary = GAIN_ATTRIBUTION_F;
            attributionOut->fDirection = ADJUST_INCREASE;
        }
    }
    
    // =========================================================================
    // PHASE 5: Set directions for ALL gains (for tertiary adjustments)
    // Only set if not already set by primary/secondary logic above
    // =========================================================================
    
    // P direction based on rise time and overshoot
    if (attributionOut->pDirection == ADJUST_NONE) {
        if (metrics->riseTimeMs > 40.0f && metrics->overshootPercent < 25.0f) {
            attributionOut->pDirection = ADJUST_INCREASE;  // Too slow, needs more P
        } else if (metrics->overshootPercent > 25.0f || metrics->oscillationAmplitude > OSCILLATION_THRESHOLD * 0.7f) {
            attributionOut->pDirection = ADJUST_DECREASE;  // Too aggressive
        }
    }
    
    // D direction based on oscillation and overshoot
    if (attributionOut->dDirection == ADJUST_NONE) {
        if (metrics->oscillationAmplitude > OSCILLATION_THRESHOLD * 0.5f || metrics->overshootPercent > 22.0f) {
            attributionOut->dDirection = ADJUST_INCREASE;  // Needs more damping
        } else if (metrics->riseTimeMs > 50.0f && metrics->overshootPercent < 8.0f) {
            attributionOut->dDirection = ADJUST_DECREASE;  // Over-damped
        }
    }
    
    // F direction based on stick tracking lag
    if (attributionOut->fDirection == ADJUST_NONE) {
        if (metrics->velocityWeightedLag > 8.0f || metrics->stickLeadError > 20.0f) {
            attributionOut->fDirection = ADJUST_INCREASE;  // Lagging stick
        } else if (metrics->velocityWeightedLag < -5.0f || metrics->stickTrackingPhase < -5.0f) {
            attributionOut->fDirection = ADJUST_DECREASE;  // Leading stick
        }
    }
    
    // I direction based on drift and steady-state error
    if (attributionOut->iDirection == ADJUST_NONE) {
        if (metrics->steadyStateError > 8.0f || metrics->driftRate > 15.0f) {
            attributionOut->iDirection = ADJUST_INCREASE;  // Not holding position
        } else if (metrics->bouncebackPercent > 20.0f) {
            attributionOut->iDirection = ADJUST_DECREASE;  // Windup
        }
    }
}

// ============================================================================
// SCORING
// ============================================================================

float autotuneCalculateScore(const autotuneMetrics_t *metrics)
{
    if (!metrics->isValid) {
        return 1000.0f;  // Invalid = worst score
    }
    
    float score = 0.0f;
    
    // Overshoot penalty (target 5-18%)
    if (metrics->overshootPercent < OVERSHOOT_TARGET_MIN) {
        // Undershoot penalty
        score += (OVERSHOOT_TARGET_MIN - metrics->overshootPercent) * 2.0f;
    } else if (metrics->overshootPercent > OVERSHOOT_TARGET_MAX) {
        // Overshoot penalty (stronger)
        score += (metrics->overshootPercent - OVERSHOOT_TARGET_MAX) * 3.0f;
    }
    // In target range = no penalty
    
    // Rise time penalty (target < 30ms)
    if (metrics->riseTimeMs > 30.0f) {
        score += (metrics->riseTimeMs - 30.0f) * 1.0f;
    }
    
    // Oscillation penalty
    if (metrics->oscillationAmplitude > OSCILLATION_THRESHOLD) {
        score += (metrics->oscillationAmplitude - OSCILLATION_THRESHOLD) * 0.5f;
    }
    
    // Noise penalty (target < 4 deg/s)
    if (metrics->noiseRms > NOISE_TARGET_RMS) {
        score += (metrics->noiseRms - NOISE_TARGET_RMS) * 5.0f;
    }
    
    // Tracking error penalty (P/D related)
    score += metrics->trackingError * 0.5f;
    
    // =========================================================================
    // I-TERM PENALTIES
    // =========================================================================
    
    // Drift penalty (I too low)
    if (fabsf(metrics->driftRate) > 2.0f) {
        score += fabsf(metrics->driftRate) * 2.0f;
    }
    
    // Bounceback penalty (I too high)
    if (metrics->bouncebackPercent > 5.0f) {
        score += (metrics->bouncebackPercent - 5.0f) * 1.5f;
    }
    
    // Slow oscillation penalty (I windup)
    if (metrics->slowOscillationHz > 0.5f && metrics->slowOscillationHz < 5.0f) {
        score += 20.0f;  // Significant penalty for I-windup oscillation
    }
    
    // Steady-state error penalty
    if (metrics->steadyStateError > 3.0f) {
        score += (metrics->steadyStateError - 3.0f) * 2.0f;
    }
    
    // =========================================================================
    // F-TERM PENALTIES
    // =========================================================================
    
    // Stick lag penalty (F too low) - gyro not keeping up with stick
    if (metrics->stickLeadError > 10.0f) {
        score += (metrics->stickLeadError - 10.0f) * 0.3f;
    }
    
    // Phase lag penalty
    if (metrics->stickTrackingPhase > 5.0f) {
        score += (metrics->stickTrackingPhase - 5.0f) * 0.5f;
    }
    
    // Initial delay penalty (slow to start responding)
    if (metrics->initialResponseDelay > 8.0f) {
        score += (metrics->initialResponseDelay - 8.0f) * 0.5f;
    }
    
    // Negative phase (gyro leading stick = F too high) - feels twitchy
    if (metrics->stickTrackingPhase < -2.0f) {
        score += fabsf(metrics->stickTrackingPhase) * 1.0f;
    }
    
    return score;
}

// ============================================================================
// FILTER ANALYSIS
// ============================================================================

void autotuneAnalyzeNoise(
    const float *gyroSamples,
    const float *throttleSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz,
    autotuneFilterAnalysis_t *filterOut
)
{
    memset(filterOut, 0, sizeof(autotuneFilterAnalysis_t));
    
    if (sampleCount < 50 || gyroSamples == NULL) {
        return;
    }
    
    // Find high-throttle region (where throttle > 0.6)
    uint16_t highThrottleStart = 0;
    uint16_t highThrottleEnd = 0;
    bool inHighThrottle = false;
    
    for (uint16_t i = 0; i < sampleCount && throttleSamples != NULL; i++) {
        if (!inHighThrottle && throttleSamples[i] > 0.6f) {
            highThrottleStart = i;
            inHighThrottle = true;
        } else if (inHighThrottle && throttleSamples[i] < 0.5f) {
            highThrottleEnd = i;
            break;
        }
    }
    
    if (!inHighThrottle) {
        highThrottleStart = 0;
        highThrottleEnd = sampleCount;
    } else if (highThrottleEnd == 0) {
        highThrottleEnd = sampleCount;
    }
    
    // Calculate noise floor in high-throttle region (for RMS backstop)
    filterOut->noiseFloor = autotuneCalculateRms(gyroSamples, highThrottleStart, highThrottleEnd);
    
    // Use dynamic notch SDFT data for resonance detection
    // This is much more accurate than zero-crossing as it uses actual FFT peaks
    if (isDynNotchActive()) {
        int notchCount = getDynNotchCount();
        float highestPeakInRange = 0.0f;
        float highestPeakFreq = 0.0f;
        
        // Check all axes for resonances in the problematic frequency range
        for (int axis = 0; axis < 3; axis++) {
            for (int peak = 0; peak < notchCount; peak++) {
                float peakFreq = getDynNotchCenterFreq(axis, peak);
                
                // Only consider peaks in the problematic range (30-500Hz)
                // Below 30Hz is acceptable for motors, above 500Hz is usually filtered by LPF
                if (peakFreq > RESONANCE_MIN_FREQ_HZ && peakFreq < RESONANCE_MAX_FREQ_HZ) {
                    // Track the highest frequency peak in range
                    // (higher frequencies are typically more problematic for propwash/resonance)
                    if (peakFreq > highestPeakFreq) {
                        highestPeakFreq = peakFreq;
                        highestPeakInRange = peakFreq;  // We don't have amplitude from accessor
                    }
                }
            }
        }
        
        // If we found peaks in the problematic range, flag as resonance
        if (highestPeakInRange > RESONANCE_MIN_FREQ_HZ) {
            filterOut->resonanceDetected = true;
            filterOut->peakFrequency = highestPeakFreq;
            filterOut->peakAmplitude = filterOut->noiseFloor * 3.0f;  // Estimated
            filterOut->suggestedNotchHz = (uint16_t)highestPeakFreq;
        }
    } else {
        // Fallback: simple resonance detection using oscillation frequency
        float oscFreq = autotuneDetectOscillationFreq(
            &gyroSamples[highThrottleStart],
            highThrottleEnd - highThrottleStart,
            sampleRateHz
        );
        
        // Check if there's a dominant frequency in problematic range
        if (oscFreq > RESONANCE_MIN_FREQ_HZ && oscFreq < RESONANCE_MAX_FREQ_HZ) {
            // Could be frame resonance - check amplitude
            float peakAmp = 0.0f;
            for (uint16_t i = highThrottleStart; i < highThrottleEnd; i++) {
                peakAmp = MAX(peakAmp, fabsf(gyroSamples[i]));
            }
            
            if (peakAmp > filterOut->noiseFloor * 3.0f) {
                filterOut->resonanceDetected = true;
                filterOut->peakFrequency = oscFreq;
                filterOut->peakAmplitude = peakAmp;
                filterOut->suggestedNotchHz = (uint16_t)oscFreq;
            }
        }
    }
}

bool autotuneIsNoiseAcceptable(float noiseRms, float targetNoise)
{
    return noiseRms <= targetNoise;
}

// ============================================================================
// TERM-SPECIFIC METRIC MEASUREMENT FUNCTIONS
// ============================================================================

// Get the appropriate metric type for a tunable parameter
tuneMetric_e getMetricForParameter(tuneParameter_e param)
{
    switch (param) {
        case TUNE_PARAM_P:
            return METRIC_SETPOINT_TRACKING;
        case TUNE_PARAM_I:
            return METRIC_LONG_TERM_ERROR;
        case TUNE_PARAM_D:
            return METRIC_OSCILLATION;
        case TUNE_PARAM_F:
            return METRIC_STICK_TRACKING;
        case TUNE_PARAM_DTERM_LPF1:
        case TUNE_PARAM_DTERM_LPF2:
        case TUNE_PARAM_GYRO_LPF1:
        case TUNE_PARAM_GYRO_LPF2:
            return METRIC_MOTOR_RMS;
        default:
            return METRIC_MOTOR_RMS;
    }
}

// Get target value for a specific metric type
float getTargetForMetric(tuneMetric_e metric)
{
    switch (metric) {
        case METRIC_MOTOR_RMS:
            return TARGET_MOTOR_RMS;           // 15.0
        case METRIC_OSCILLATION:
            return TARGET_OSCILLATION;         // 5.0
        case METRIC_SETPOINT_TRACKING:
            return TARGET_SETPOINT_TRACKING;   // 0.95 (95%)
        case METRIC_STICK_TRACKING:
            return TARGET_STICK_TRACKING;      // 0.90 (90%)
        case METRIC_LONG_TERM_ERROR:
            return TARGET_LONG_TERM_ERROR;     // 2.0
        case METRIC_OVERSHOOT:
            return TARGET_OVERSHOOT;           // 10.0
        case METRIC_SETTLING_TIME:
            return TARGET_SETTLING_TIME;       // 150.0
        default:
            return TARGET_MOTOR_RMS;
    }
}

// Measure oscillation amplitude for D-term optimization
// Returns peak-to-peak oscillation amplitude in the settling region
// Lower values indicate better D-term tuning (less oscillation)
float measureOscillation(
    const float *gyroSamples,
    const float *dtermSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz
)
{
    UNUSED(sampleRateHz);
    
    if (sampleCount < 30 || gyroSamples == NULL) {
        return 1000.0f;  // Invalid - return high value
    }
    
    // Focus on the settling region (last 60% of samples)
    uint16_t settleStart = sampleCount * 4 / 10;  // Start at 40%
    uint16_t settleEnd = sampleCount;
    
    // Find peak-to-peak oscillation in gyro signal
    float gyroMin = gyroSamples[settleStart];
    float gyroMax = gyroSamples[settleStart];
    
    for (uint16_t i = settleStart; i < settleEnd; i++) {
        gyroMin = MIN(gyroMin, gyroSamples[i]);
        gyroMax = MAX(gyroMax, gyroSamples[i]);
    }
    
    float gyroOscillation = gyroMax - gyroMin;
    
    // If D-term samples available, also check D-term oscillation
    float dtermOscillation = 0.0f;
    if (dtermSamples != NULL) {
        float dtermMin = dtermSamples[settleStart];
        float dtermMax = dtermSamples[settleStart];
        
        for (uint16_t i = settleStart; i < settleEnd; i++) {
            dtermMin = MIN(dtermMin, dtermSamples[i]);
            dtermMax = MAX(dtermMax, dtermSamples[i]);
        }
        
        dtermOscillation = dtermMax - dtermMin;
    }
    
    // Return the larger of gyro or D-term oscillation
    // D-term oscillation is weighted higher as it directly affects motor output
    return MAX(gyroOscillation, dtermOscillation * 1.5f);
}

// Measure setpoint tracking accuracy for P-term optimization
// Returns tracking accuracy as a ratio (0-1.0, higher is better)
// Target is ~0.95 (95% tracking accuracy)
float measureSetpointTracking(
    const float *gyroSamples,
    const float *setpointSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz
)
{
    UNUSED(sampleRateHz);
    
    if (sampleCount < 20 || gyroSamples == NULL || setpointSamples == NULL) {
        return 0.0f;  // Invalid - return worst case
    }
    
    // Find the maneuver region (where setpoint is significant)
    uint16_t maneuverStart = 0;
    uint16_t maneuverEnd = sampleCount;
    float peakSetpoint = 0.0f;
    
    for (uint16_t i = 0; i < sampleCount; i++) {
        if (fabsf(setpointSamples[i]) > 50.0f) {
            if (maneuverStart == 0) {
                maneuverStart = i;
            }
            maneuverEnd = i;
        }
        if (fabsf(setpointSamples[i]) > fabsf(peakSetpoint)) {
            peakSetpoint = setpointSamples[i];
        }
    }
    
    if (fabsf(peakSetpoint) < 10.0f) {
        return 1.0f;  // No significant maneuver - assume tracking is fine
    }
    
    // Calculate tracking accuracy as ratio of actual response to expected
    // Focus on the region from 10% to 90% of response (rise time region)
    float targetThreshold10 = peakSetpoint * 0.10f;
    float targetThreshold90 = peakSetpoint * 0.90f;
    
    uint16_t idx10 = maneuverStart;
    uint16_t idx90 = maneuverEnd;
    
    for (uint16_t i = maneuverStart; i < maneuverEnd; i++) {
        if ((peakSetpoint > 0 && gyroSamples[i] >= targetThreshold10) ||
            (peakSetpoint < 0 && gyroSamples[i] <= targetThreshold10)) {
            idx10 = i;
            break;
        }
    }
    
    for (uint16_t i = idx10; i < maneuverEnd; i++) {
        if ((peakSetpoint > 0 && gyroSamples[i] >= targetThreshold90) ||
            (peakSetpoint < 0 && gyroSamples[i] <= targetThreshold90)) {
            idx90 = i;
            break;
        }
    }
    
    // Calculate average tracking ratio in the tracking region (after reaching 90%)
    float trackingSum = 0.0f;
    uint16_t trackingCount = 0;
    uint16_t trackRegionStart = idx90;
    uint16_t trackRegionEnd = MIN(maneuverEnd, idx90 + 30);  // 30 samples after 90%
    
    for (uint16_t i = trackRegionStart; i < trackRegionEnd; i++) {
        if (fabsf(setpointSamples[i]) > 10.0f) {
            float ratio = gyroSamples[i] / setpointSamples[i];
            // Clamp ratio to reasonable bounds
            ratio = constrainf(ratio, 0.0f, 2.0f);
            trackingSum += ratio;
            trackingCount++;
        }
    }
    
    if (trackingCount > 0) {
        float avgRatio = trackingSum / trackingCount;
        // Convert to accuracy (1.0 = perfect, deviation reduces score)
        float accuracy = 1.0f - fabsf(1.0f - avgRatio);
        return constrainf(accuracy, 0.0f, 1.0f);
    }
    
    return 0.5f;  // Default mid-range if we couldn't calculate
}

// Measure stick tracking during rapid movements for F-term optimization
// Returns velocity-weighted lag (lower is better, 0 = perfect tracking)
// This is the key metric for feedforward tuning
float measureStickTracking(
    const float *gyroSamples,
    const float *setpointSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz
)
{
    UNUSED(sampleRateHz);
    
    if (sampleCount < 10 || gyroSamples == NULL || setpointSamples == NULL) {
        return 1000.0f;  // Invalid - return high value
    }
    
    // Find peak setpoint and its index
    float peakSetpoint = 0.0f;
    uint16_t peakSetpointIdx = 0;
    uint16_t maneuverStart = 0;
    
    for (uint16_t i = 0; i < sampleCount; i++) {
        if (fabsf(setpointSamples[i]) > 50.0f && maneuverStart == 0) {
            maneuverStart = i;
        }
        if (fabsf(setpointSamples[i]) > fabsf(peakSetpoint)) {
            peakSetpoint = setpointSamples[i];
            peakSetpointIdx = i;
        }
    }
    
    if (fabsf(peakSetpoint) < 10.0f || peakSetpointIdx <= maneuverStart) {
        return 0.0f;  // No significant maneuver
    }
    
    // Calculate velocity-weighted tracking error
    // F-term is most important when stick is moving fast
    float velocityWeightedErrorSum = 0.0f;
    float velocityWeightSum = 0.0f;
    
    for (uint16_t i = maneuverStart + 1; i < peakSetpointIdx && i < sampleCount; i++) {
        // Stick velocity = change in setpoint
        float stickDelta = fabsf(setpointSamples[i] - setpointSamples[i-1]);
        
        // Only consider significant stick movements
        if (stickDelta > 2.0f) {
            // Tracking error at this instant (positive = gyro lagging)
            float trackingErr = fabsf(setpointSamples[i]) - fabsf(gyroSamples[i]);
            
            // Weight error by velocity squared (emphasizes high-velocity regions)
            float weight = stickDelta * stickDelta;
            velocityWeightedErrorSum += trackingErr * weight;
            velocityWeightSum += weight;
        }
    }
    
    if (velocityWeightSum > 0.01f) {
        return velocityWeightedErrorSum / velocityWeightSum;
    }
    
    return 0.0f;  // No rapid stick movement detected
}

// Measure long-term error/drift for I-term optimization
// Returns accumulated error (lower is better)
// Combines drift rate and steady-state error
float measureLongTermError(
    const float *gyroSamples,
    const float *setpointSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz
)
{
    if (sampleCount < 40 || gyroSamples == NULL || setpointSamples == NULL) {
        return 1000.0f;  // Invalid - return high value
    }
    
    const float samplePeriodMs = 1000.0f / sampleRateHz;
    
    // Find the settling region (last 30% of samples)
    uint16_t settleStart = sampleCount * 7 / 10;
    uint16_t settleEnd = sampleCount;
    
    // 1. Calculate steady-state error (average absolute error in settling region)
    float steadyStateErrorSum = 0.0f;
    uint16_t errorCount = 0;
    
    for (uint16_t i = settleStart; i < settleEnd; i++) {
        float error = fabsf(gyroSamples[i] - setpointSamples[i]);
        steadyStateErrorSum += error;
        errorCount++;
    }
    
    float steadyStateError = (errorCount > 0) ? (steadyStateErrorSum / errorCount) : 0.0f;
    
    // 2. Calculate drift rate using linear regression
    float driftRate = 0.0f;
    if (settleEnd - settleStart >= 10) {
        float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        uint16_t n = settleEnd - settleStart;
        
        for (uint16_t i = settleStart; i < settleEnd; i++) {
            float x = (float)(i - settleStart) * samplePeriodMs;
            float y = gyroSamples[i];
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }
        
        float denom = n * sumX2 - sumX * sumX;
        if (fabsf(denom) > 0.001f) {
            // Slope in deg/s per ms, convert to deg/s per second
            driftRate = fabsf(((n * sumXY - sumX * sumY) / denom) * 1000.0f);
        }
    }
    
    // 3. Combine metrics: steady-state error + drift contribution
    // Drift is weighted higher as it accumulates over time
    float longTermError = steadyStateError + driftRate * 0.5f;
    
    return longTermError;
}

// Get metric value from pre-computed metrics structure
// Convenient wrapper for Newton's method to get the right metric
float getMetricFromAnalysis(const autotuneMetrics_t *metrics, tuneMetric_e metricType)
{
    if (metrics == NULL || !metrics->isValid) {
        return 1000.0f;  // Invalid - return high value
    }
    
    switch (metricType) {
        case METRIC_MOTOR_RMS:
            return metrics->noiseRms;
            
        case METRIC_OSCILLATION:
            return metrics->oscillationAmplitude;
            
        case METRIC_SETPOINT_TRACKING:
            // Convert tracking error to accuracy (lower error = higher accuracy)
            // Tracking error of 0 = 1.0 (100%), error of 100 = 0.0
            return constrainf(1.0f - (metrics->trackingError / 100.0f), 0.0f, 1.0f);
            
        case METRIC_STICK_TRACKING:
            return metrics->velocityWeightedLag;
            
        case METRIC_LONG_TERM_ERROR:
            // Combine drift and steady-state error
            return fabsf(metrics->driftRate) + metrics->steadyStateError;
            
        case METRIC_OVERSHOOT:
            return metrics->overshootPercent;
            
        case METRIC_SETTLING_TIME:
            return metrics->settlingTimeMs;
            
        default:
            return 1000.0f;
    }
}

#endif // USE_AUTOTUNE
