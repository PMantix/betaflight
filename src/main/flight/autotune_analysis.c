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
    
    // Overshoot = (peak - setpoint) / setpoint * 100
    if (fabsf(peakSetpoint) > 1.0f) {
        float overshoot = (fabsf(peakResponse) - fabsf(peakSetpoint)) / fabsf(peakSetpoint) * 100.0f;
        metricsOut->overshootPercent = MAX(0.0f, overshoot);
    }
    
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
    
    // Calculate oscillation amplitude (peak-to-peak after peak response)
    if (peakResponseIdx + 10 < sampleCount) {
        float minVal = gyroSamples[peakResponseIdx];
        float maxVal = gyroSamples[peakResponseIdx];
        
        for (uint16_t i = peakResponseIdx; i < sampleCount; i++) {
            minVal = MIN(minVal, gyroSamples[i]);
            maxVal = MAX(maxVal, gyroSamples[i]);
        }
        
        metricsOut->oscillationAmplitude = maxVal - minVal;
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
    
    // Check for noise limit first
    if (metrics->noiseRms > NOISE_TARGET_RMS * 1.5f) {
        return RESPONSE_NOISY;
    }
    
    // Check overshoot
    if (metrics->overshootPercent > OVERSHOOT_TARGET_MAX * 1.5f) {
        return RESPONSE_UNDERDAMPED;
    }
    
    if (metrics->overshootPercent < OVERSHOOT_TARGET_MIN) {
        // Low overshoot - check if it's truly overdamped or excellent
        if (metrics->riseTimeMs > 50.0f) {
            return RESPONSE_OVERDAMPED;
        }
    }
    
    // Check for oscillations
    if (metrics->oscillationAmplitude > OSCILLATION_THRESHOLD) {
        return RESPONSE_UNDERDAMPED;
    }
    
    // Check if in target range
    if (metrics->overshootPercent >= OVERSHOOT_TARGET_MIN && 
        metrics->overshootPercent <= OVERSHOOT_TARGET_MAX &&
        metrics->riseTimeMs < 40.0f &&
        metrics->noiseRms < NOISE_TARGET_RMS) {
        return RESPONSE_EXCELLENT;
    }
    
    // Good but not perfect
    if (metrics->overshootPercent >= OVERSHOOT_TARGET_MIN * 0.5f &&
        metrics->overshootPercent <= OVERSHOOT_TARGET_MAX * 1.2f) {
        return RESPONSE_CRITICAL;
    }
    
    return RESPONSE_UNKNOWN;
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
    // PHASE 1: Check for clear I-term issues (independent of response class)
    // =========================================================================
    
    // Drift after settling = I too low (not holding position)
    // Note: This needs to be high enough to ignore measurement noise
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
    
    // =========================================================================
    // PHASE 2: Check for clear F-term issues
    // =========================================================================
    
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
    // PHASE 3: P/D attribution based on response class
    // =========================================================================
    
    switch (responseClass) {
        case RESPONSE_UNDERDAMPED:
            // Too much overshoot / oscillation
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
            break;
            
        case RESPONSE_OVERDAMPED:
            // Too slow / sluggish
            if (metrics->riseTimeMs > 60.0f) {
                // Very slow = P is definitely too low
                attributionOut->primary = GAIN_ATTRIBUTION_P;
                attributionOut->pDirection = ADJUST_INCREASE;
                attributionOut->confidence = 0.8f;
            } else if (metrics->initialResponseDelay > 10.0f) {
                // Slow initial response could be F issue
                attributionOut->primary = GAIN_ATTRIBUTION_F;
                attributionOut->fDirection = ADJUST_INCREASE;
                attributionOut->secondary = GAIN_ATTRIBUTION_P;
                attributionOut->pDirection = ADJUST_INCREASE;
                attributionOut->confidence = 0.5f;
            } else {
                // Moderately slow = could be D too high (over-damping)
                attributionOut->primary = GAIN_ATTRIBUTION_D;
                attributionOut->dDirection = ADJUST_DECREASE;
                attributionOut->secondary = GAIN_ATTRIBUTION_P;
                attributionOut->pDirection = ADJUST_INCREASE;
                attributionOut->confidence = 0.6f;
            }
            break;
            
        case RESPONSE_NOISY:
            // Noise limiting
            attributionOut->primary = GAIN_ATTRIBUTION_D;
            attributionOut->dDirection = ADJUST_DECREASE;
            attributionOut->secondary = GAIN_ATTRIBUTION_FILTER;
            attributionOut->confidence = 0.7f;
            break;
            
        case RESPONSE_CRITICAL:
        case RESPONSE_EXCELLENT:
            // Good P/D response - look for I/F improvements
            if (metrics->steadyStateError > 5.0f) {
                // Could use a bit more I
                attributionOut->primary = GAIN_ATTRIBUTION_I;
                attributionOut->iDirection = ADJUST_INCREASE;
                attributionOut->confidence = 0.4f;
            } else if (metrics->stickLeadError > 15.0f) {
                // Could use a bit more F
                attributionOut->primary = GAIN_ATTRIBUTION_F;
                attributionOut->fDirection = ADJUST_INCREASE;
                attributionOut->confidence = 0.4f;
            } else {
                // Try pushing P slightly for more responsiveness
                attributionOut->primary = GAIN_ATTRIBUTION_P;
                attributionOut->pDirection = ADJUST_INCREASE;
                attributionOut->confidence = 0.3f;  // Very conservative
            }
            break;
            
        default:
            // Unknown - conservative P increase
            attributionOut->primary = GAIN_ATTRIBUTION_P;
            attributionOut->pDirection = ADJUST_INCREASE;
            attributionOut->confidence = 0.3f;
            break;
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
    
    // Calculate noise floor in high-throttle region
    filterOut->noiseFloor = autotuneCalculateRms(gyroSamples, highThrottleStart, highThrottleEnd);
    
    // Simple resonance detection using oscillation frequency
    float oscFreq = autotuneDetectOscillationFreq(
        &gyroSamples[highThrottleStart],
        highThrottleEnd - highThrottleStart,
        sampleRateHz
    );
    
    // Check if there's a dominant frequency in typical motor noise range (100-400Hz)
    if (oscFreq > 80.0f && oscFreq < 500.0f) {
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

bool autotuneIsNoiseAcceptable(float noiseRms, float targetNoise)
{
    return noiseRms <= targetNoise;
}

#endif // USE_AUTOTUNE
