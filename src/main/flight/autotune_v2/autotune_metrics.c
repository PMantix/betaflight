/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_AUTOTUNE_V2

#include <string.h>
#include <math.h>

#include "common/maths.h"

#include "flight/autotune_v2/autotune_metrics.h"
#include "flight/autotune_v2/autotune_debug.h"

// ============================================================================
// Initialization
// ============================================================================

void autotuneMetricsInit(void)
{
    autotuneMetricsReset();
}

void autotuneMetricsReset(void)
{
    // Nothing to reset currently
}

// ============================================================================
// Metrics Computation
// ============================================================================

bool autotuneMetricsAnalyze(const eventBuffer_t *buffer, eventMetrics_t *metricsOut, float loopTimeUs)
{
    if (!buffer || !metricsOut || buffer->count < 10) {
        return false;
    }
    
    // Validate loopTimeUs - use sensible default if invalid
    if (loopTimeUs <= 0.0f || loopTimeUs > 10000.0f) {
        loopTimeUs = 250.0f;  // Default to 4kHz (250µs) if invalid
    }
    
    memset(metricsOut, 0, sizeof(eventMetrics_t));
    
    // Compute overshoot
    metricsOut->overshootPct = autotuneMetricsComputeOvershoot(
        buffer->gyro, buffer->setpoint, buffer->count);
    
    // Detect rebound using setpoint-relative threshold (CRIT-010 fix)
    metricsOut->hasRebound = autotuneMetricsDetectRebound(
        buffer->gyro, buffer->setpoint, buffer->count, AUTOTUNE_REBOUND_THRESHOLD);
    
    if (metricsOut->hasRebound) {
        metricsOut->reboundPct = autotuneMetricsComputeRebound(
            buffer->gyro, buffer->setpoint, buffer->count);
    }
    
    // Compute timing metrics using actual loop time (CRIT-008 fix)
    metricsOut->settlingTimeMs = autotuneMetricsComputeSettlingTime(
        buffer->gyro, buffer->setpoint, buffer->count, loopTimeUs);
    metricsOut->lagMs = autotuneMetricsComputeLag(
        buffer->gyro, buffer->setpoint, buffer->count, loopTimeUs);
    
    metricsOut->valid = true;
    return true;
}

// ============================================================================
// Overshoot Analysis
// ============================================================================

float autotuneMetricsComputeOvershoot(const float *gyro, const float *setpoint, uint16_t count)
{
    if (!gyro || !setpoint || count < 10) {
        return 0.0f;
    }
    
    // Find peak setpoint (the maximum commanded rate during the maneuver)
    // This is the reference for overshoot calculation
    float peakSetpoint = 0.0f;
    int peakSetpointIdx = 0;
    for (uint16_t i = 0; i < count; i++) {
        float absSetpoint = fabsf(setpoint[i]);
        if (absSetpoint > peakSetpoint) {
            peakSetpoint = absSetpoint;
            peakSetpointIdx = i;
        }
    }
    
    if (peakSetpoint < 50.0f) {  // Minimum 50 deg/s to be a valid maneuver
        return 0.0f;
    }
    
    // Find peak gyro value AFTER the setpoint peak (overshoot occurs after command)
    // Search from setpoint peak to end of buffer
    float peakGyro = 0.0f;
    for (uint16_t i = peakSetpointIdx; i < count; i++) {
        float absGyro = fabsf(gyro[i]);
        if (absGyro > peakGyro) {
            peakGyro = absGyro;
        }
    }
    
    // Overshoot = (peakGyro - peakSetpoint) / peakSetpoint * 100
    // If gyro exceeds setpoint, we have overshoot
    // If gyro is less than setpoint, overshoot is 0 (or negative = undershoot)
    const float overshoot = (peakGyro - peakSetpoint) / peakSetpoint * 100.0f;
    return overshoot > 0.0f ? overshoot : 0.0f;
}

bool autotuneMetricsOvershootInBand(float overshootPct, float targetLow, float targetHigh)
{
    return overshootPct >= targetLow && overshootPct <= targetHigh;
}

// ============================================================================
// Rebound Detection
// ============================================================================

// CRIT-010 fix: Use setpoint-relative threshold for consistent behavior
bool autotuneMetricsDetectRebound(const float *gyro, const float *setpoint, uint16_t count, float threshold)
{
    if (!gyro || !setpoint || count < 20) {
        return false;
    }
    
    int peakIndices[10];
    int numPeaks = autotuneMetricsFindAllPeaks(gyro, count, peakIndices, 10);
    
    if (numPeaks < 2) {
        return false;
    }
    
    // Compute final setpoint for reference (average of last few samples)
    float finalSetpoint = 0.0f;
    const int avgCount = count > 10 ? 5 : count / 2;
    for (int i = count - avgCount; i < count; i++) {
        finalSetpoint += fabsf(setpoint[i]);
    }
    finalSetpoint /= avgCount;
    
    // Use setpoint as reference for threshold, not first peak
    // This gives consistent behavior regardless of overshoot level
    const float reboundThreshold = finalSetpoint * threshold;
    
    // Check if any peak after the first exceeds threshold
    for (int i = 1; i < numPeaks; i++) {
        const float peakMag = fabsf(gyro[peakIndices[i]]);
        if (peakMag > reboundThreshold) {
            return true;
        }
    }
    
    return false;
}

float autotuneMetricsComputeRebound(const float *gyro, const float *setpoint, uint16_t count)
{
    if (!gyro || !setpoint || count < 20) {
        return 0.0f;
    }
    
    int peakIndices[10];
    int numPeaks = autotuneMetricsFindAllPeaks(gyro, count, peakIndices, 10);
    
    if (numPeaks < 2) {
        return 0.0f;
    }
    
    // Compute final setpoint for reference
    float finalSetpoint = 0.0f;
    const int avgCount = count > 10 ? 5 : count / 2;
    for (int i = count - avgCount; i < count; i++) {
        finalSetpoint += fabsf(setpoint[i]);
    }
    finalSetpoint /= avgCount;
    
    if (finalSetpoint < 0.001f) {
        return 0.0f;
    }
    
    // Find max rebound peak
    float maxRebound = 0.0f;
    for (int i = 1; i < numPeaks; i++) {
        const float peakMag = fabsf(gyro[peakIndices[i]]);
        if (peakMag > maxRebound) {
            maxRebound = peakMag;
        }
    }
    
    // Return rebound as percentage of setpoint
    return (maxRebound / finalSetpoint) * 100.0f;
}

// ============================================================================
// Timing Metrics
// ============================================================================

float autotuneMetricsComputeSettlingTime(const float *gyro, const float *setpoint,
                                          uint16_t count, float loopTimeUs)
{
    if (!gyro || !setpoint || count < 20) {
        return 0.0f;
    }
    
    // Find final setpoint (average of last 5 samples)
    float finalSetpoint = 0.0f;
    const int avgCount = 5;
    for (int i = count - avgCount; i < count; i++) {
        finalSetpoint += setpoint[i];
    }
    finalSetpoint /= avgCount;
    
    if (fabsf(finalSetpoint) < 0.001f) {
        return 0.0f;  // No significant target
    }
    
    // Settling band is 5% of final setpoint (uses AUTOTUNE_SETTLING_BAND from autotune_debug.h)
    const float settlingBand = fabsf(finalSetpoint) * AUTOTUNE_SETTLING_BAND;
    
    // Find last time the response was outside the settling band
    // Search backwards from end
    int lastOutsideIdx = -1;
    for (int i = count - 1; i >= 0; i--) {
        if (fabsf(gyro[i] - finalSetpoint) > settlingBand) {
            lastOutsideIdx = i;
            break;
        }
    }
    
    if (lastOutsideIdx < 0) {
        return 0.0f;  // Always within band
    }
    
    // Settling time = samples from lastOutsideIdx to end * sample period
    const float settlingTimeUs = (count - 1 - lastOutsideIdx) * loopTimeUs;
    return settlingTimeUs / 1000.0f;  // Return in milliseconds
}

float autotuneMetricsComputeLag(const float *gyro, const float *setpoint,
                                 uint16_t count, float loopTimeUs)
{
    if (!gyro || !setpoint || count < 10) {
        return 0.0f;
    }
    
    // Find PEAK setpoint (the maximum commanded rate during the maneuver)
    // This is the reference for lag calculation - same fix as overshoot
    float peakSetpoint = 0.0f;
    for (uint16_t i = 0; i < count; i++) {
        float absSetpoint = fabsf(setpoint[i]);
        if (absSetpoint > fabsf(peakSetpoint)) {
            peakSetpoint = setpoint[i];  // Keep sign for direction
        }
    }
    
    if (fabsf(peakSetpoint) < 50.0f) {
        return 0.0f;  // Minimum 50 deg/s to be valid
    }
    
    // Find when gyro first reaches 50% of PEAK setpoint
    // Search from start of buffer (before the peak)
    const float halfTarget = peakSetpoint * 0.5f;
    int crossingIdx = -1;
    
    // Handle both positive and negative targets
    if (peakSetpoint > 0) {
        for (int i = 0; i < count; i++) {
            if (gyro[i] >= halfTarget) {
                crossingIdx = i;
                break;
            }
        }
    } else {
        for (int i = 0; i < count; i++) {
            if (gyro[i] <= halfTarget) {
                crossingIdx = i;
                break;
            }
        }
    }
    
    if (crossingIdx < 0) {
        return 0.0f;  // Never reached 50%
    }
    
    // Lag = samples to reach 50% * sample period
    const float lagUs = crossingIdx * loopTimeUs;
    return lagUs / 1000.0f;  // Return in milliseconds
}

// ============================================================================
// Peak Detection
// ============================================================================

int autotuneMetricsFindFirstPeak(const float *gyro, uint16_t count)
{
    if (!gyro || count < 3) {
        return -1;
    }
    
    // Simple peak detection: find first local maximum
    for (uint16_t i = 1; i < count - 1; i++) {
        const float prev = fabsf(gyro[i - 1]);
        const float curr = fabsf(gyro[i]);
        const float next = fabsf(gyro[i + 1]);
        
        if (curr > prev && curr > next) {
            return i;
        }
    }
    
    return -1;
}

int autotuneMetricsFindAllPeaks(const float *gyro, uint16_t count,
                                 int *peakIndices, int maxPeaks)
{
    if (!gyro || !peakIndices || count < 3 || maxPeaks < 1) {
        return 0;
    }
    
    int numPeaks = 0;
    
    // Simple peak detection: find all local maxima
    for (uint16_t i = 1; i < count - 1 && numPeaks < maxPeaks; i++) {
        const float prev = fabsf(gyro[i - 1]);
        const float curr = fabsf(gyro[i]);
        const float next = fabsf(gyro[i + 1]);
        
        if (curr > prev && curr > next) {
            peakIndices[numPeaks++] = i;
        }
    }
    
    return numPeaks;
}

#endif // USE_AUTOTUNE_V2
