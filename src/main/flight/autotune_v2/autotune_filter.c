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

#include <math.h>
#include <string.h>

#include "common/maths.h"
#include "common/time.h"

#include "drivers/time.h"

#include "sensors/gyro.h"

#include "flight/autotune_v2/autotune_filter.h"
#include "flight/autotune_v2/autotune_debug.h"

// ============================================================================
// Static State
// ============================================================================

static filterCharState_t filterState;

#define NOISE_SAMPLE_BUFFER_SIZE 32
static float noiseSampleBuffer[NOISE_SAMPLE_BUFFER_SIZE];
static uint8_t noiseSampleIndex = 0;
static float hoverNoiseLevel = 0.0f;

// ============================================================================
// Noise Measurement
// ============================================================================

static float measureCurrentNoise(void)
{
    // Sample current gyro magnitude
    float gyroMag = sqrtf(sq(gyro.gyroADCf[0]) + 
                          sq(gyro.gyroADCf[1]) + 
                          sq(gyro.gyroADCf[2]));
    
    // Add to circular buffer
    noiseSampleBuffer[noiseSampleIndex & (NOISE_SAMPLE_BUFFER_SIZE - 1)] = gyroMag;
    noiseSampleIndex++;
    
    // Compute variance (RMS of deviation from mean)
    float sum = 0;
    for (int i = 0; i < NOISE_SAMPLE_BUFFER_SIZE; i++) {
        sum += noiseSampleBuffer[i];
    }
    float mean = sum / NOISE_SAMPLE_BUFFER_SIZE;
    
    float variance = 0;
    for (int i = 0; i < NOISE_SAMPLE_BUFFER_SIZE; i++) {
        float diff = noiseSampleBuffer[i] - mean;
        variance += diff * diff;
    }
    
    return sqrtf(variance / NOISE_SAMPLE_BUFFER_SIZE);  // RMS noise
}

// ============================================================================
// Initialization
// ============================================================================

void autotuneFilterInit(void)
{
    autotuneFilterReset();
}

void autotuneFilterReset(void)
{
    memset(&filterState, 0, sizeof(filterCharState_t));
    filterState.minThrottleSeen = 1.0f;
    filterState.maxThrottleSeen = 0.0f;
    
    // Reset noise measurement state
    memset(noiseSampleBuffer, 0, sizeof(noiseSampleBuffer));
    noiseSampleIndex = 0;
    hoverNoiseLevel = 0.0f;
}

// ============================================================================
// Throttle Sweep
// ============================================================================

// Throttle thresholds for sweep detection
#define SWEEP_HIGH_THROTTLE     0.50f   // 50% considered "high"
#define SWEEP_LOW_THROTTLE      0.30f   // 30% considered "low"
#define SWEEP_HOVER_LOW         0.30f   // Lower bound of hover
#define SWEEP_HOVER_HIGH        0.60f   // Upper bound of hover
#define SWEEP_STABLE_HOVER_US   2000000 // 2 seconds stable hover after sweeps
#define MIN_SWEEPS_REQUIRED     2       // Require at least 2 full sweeps

bool autotuneFilterUpdate(timeUs_t currentTimeUs, float throttle, float noiseLevel)
{
    UNUSED(noiseLevel);  // We measure noise directly from gyro
    
    // Track throttle range
    if (throttle < filterState.minThrottleSeen) {
        filterState.minThrottleSeen = throttle;
    }
    if (throttle > filterState.maxThrottleSeen) {
        filterState.maxThrottleSeen = throttle;
    }
    
    // Measure current noise from gyro
    float currentNoise = measureCurrentNoise();
    
    // Determine throttle band (10% bands: 0-10%, 10-20%, etc.)
    uint8_t throttleBand = (uint8_t)(throttle * 10.0f);
    if (throttleBand >= FILTER_CHAR_MAX_SAMPLES) {
        throttleBand = FILTER_CHAR_MAX_SAMPLES - 1;
    }
    
    // Store sample if this is a new band or higher noise in existing band
    if (filterState.throttlePoints[throttleBand] == 0.0f || 
        currentNoise > filterState.noiseLevel[throttleBand]) {
        filterState.throttlePoints[throttleBand] = throttle;
        filterState.noiseLevel[throttleBand] = currentNoise;
        if (throttleBand >= filterState.sampleCount) {
            filterState.sampleCount = throttleBand + 1;
        }
    }
    
    // Update hover noise if in hover throttle range
    if (throttle >= SWEEP_HOVER_LOW && throttle <= SWEEP_HOVER_HIGH) {
        // Use exponential moving average for hover noise
        if (hoverNoiseLevel == 0.0f) {
            hoverNoiseLevel = currentNoise;
        } else {
            hoverNoiseLevel = hoverNoiseLevel * 0.95f + currentNoise * 0.05f;
        }
    }
    
    // Track sweep state machine
    if (throttle >= SWEEP_HIGH_THROTTLE) {
        filterState.hasSeenHighThrottle = true;
        filterState.lastHighThrottleUs = currentTimeUs;
        filterState.hoverStableStartUs = 0;  // Not in stable hover
    }
    
    if (throttle <= SWEEP_LOW_THROTTLE) {
        // If we've seen high, going low completes a sweep
        if (filterState.hasSeenHighThrottle) {
            filterState.hasSeenLowThrottle = true;
        }
    }
    
    // Count a sweep when we've been high then low, then back to hover
    if (filterState.hasSeenHighThrottle && filterState.hasSeenLowThrottle) {
        if (throttle >= SWEEP_HOVER_LOW && throttle <= SWEEP_HOVER_HIGH) {
            // Returned to hover after sweep
            filterState.sweepCount++;
            filterState.hasSeenHighThrottle = false;
            filterState.hasSeenLowThrottle = false;
        }
    }
    
    // Track stable hover time after completing required sweeps
    if (filterState.sweepCount >= MIN_SWEEPS_REQUIRED) {
        if (throttle >= SWEEP_HOVER_LOW && throttle <= SWEEP_HOVER_HIGH) {
            if (filterState.hoverStableStartUs == 0) {
                filterState.hoverStableStartUs = currentTimeUs;
            }
        } else {
            filterState.hoverStableStartUs = 0;  // Reset if we leave hover
        }
    }
    
    return filterState.characterizationDone;
}

bool autotuneFilterSweepComplete(void)
{
    // Require: 
    // 1. At least MIN_SWEEPS_REQUIRED complete sweeps (high->low->hover)
    // 2. Currently in stable hover for SWEEP_STABLE_HOVER_US
    if (filterState.sweepCount < MIN_SWEEPS_REQUIRED) {
        return false;
    }
    
    if (filterState.hoverStableStartUs == 0) {
        return false;
    }
    
    const timeUs_t now = micros();
    const timeDelta_t hoverDuration = cmpTimeUs(now, filterState.hoverStableStartUs);
    return hoverDuration >= SWEEP_STABLE_HOVER_US;
}

const filterCharState_t* autotuneFilterGetState(void)
{
    return &filterState;
}

// ============================================================================
// Filter Recommendations
// ============================================================================

void autotuneFilterComputeRecommendations(void)
{
    // Find maximum noise level across all throttle bands
    float maxNoise = 0.0f;
    for (uint8_t i = 0; i < FILTER_CHAR_MAX_SAMPLES; i++) {
        if (filterState.noiseLevel[i] > maxNoise) {
            maxNoise = filterState.noiseLevel[i];
        }
    }
    
    // Recommend LPF based on max noise level
    if (maxNoise > 40.0f) {
        // High noise - aggressive filtering
        filterState.recommendedLpf = 150.0f;
    } else if (maxNoise > 20.0f) {
        // Medium noise - moderate filtering
        filterState.recommendedLpf = 200.0f;
    } else {
        // Low noise - light filtering for best response
        filterState.recommendedLpf = 250.0f;
    }
    
    // No notch filters by default (could be extended with FFT analysis)
    filterState.recommendedNotch1 = 0.0f;
    filterState.recommendedNotch2 = 0.0f;
    
    filterState.characterizationDone = true;
}

float autotuneFilterGetRecommendedLpf(void)
{
    return filterState.recommendedLpf;
}

float autotuneFilterGetRecommendedNotch1(void)
{
    return filterState.recommendedNotch1;
}

float autotuneFilterGetRecommendedNotch2(void)
{
    return filterState.recommendedNotch2;
}

void autotuneFilterSetRecommendedLpf(float lpfHz)
{
    // Clamp to minimum 100Hz to prevent over-filtering
    if (lpfHz < 100.0f) {
        lpfHz = 100.0f;
    }
    filterState.recommendedLpf = lpfHz;
}

// ============================================================================
// Filter Application
// ============================================================================

void autotuneFilterApplyRecommendations(void)
{
    // TODO: Phase 1 - apply filter settings to gyro and D-term filters
}

void autotuneFilterRollback(void)
{
    // TODO: Phase 1 - restore previous filter settings
}

// ============================================================================
// Noise Measurement
// ============================================================================

float autotuneFilterGetHoverNoise(void)
{
    return hoverNoiseLevel;
}

bool autotuneFilterNoiseAcceptable(void)
{
    return autotuneFilterGetHoverNoise() < AUTOTUNE_NOISE_ACCEPTABLE;
}

#endif // USE_AUTOTUNE_V2
