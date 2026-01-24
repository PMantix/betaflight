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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flight/autotune_v2/autotune_types.h"

// ============================================================================
// Initialization
// ============================================================================

// Initialize metrics analysis system
void autotuneMetricsInit(void);

// Reset metrics state
void autotuneMetricsReset(void);

// ============================================================================
// Metrics Computation
// ============================================================================

// Analyze event buffer and compute metrics
// loopTimeUs: actual PID loop period in microseconds (e.g., gyro.targetLooptime)
// Returns true if metrics are valid
bool autotuneMetricsAnalyze(const eventBuffer_t *buffer, eventMetrics_t *metricsOut, float loopTimeUs);

// ============================================================================
// Overshoot Analysis
// ============================================================================

// Compute overshoot percentage from gyro response
// overshoot = (peak - final) / final * 100
float autotuneMetricsComputeOvershoot(const float *gyro, const float *setpoint, uint16_t count);

// Check if overshoot is within target band
bool autotuneMetricsOvershootInBand(float overshootPct, float targetLow, float targetHigh);

// ============================================================================
// Rebound Detection
// ============================================================================

// Detect rebound (secondary peaks indicating oscillation)
// Uses setpoint-relative threshold for consistent behavior across overshoot levels
// Returns true if rebound detected above threshold
bool autotuneMetricsDetectRebound(const float *gyro, const float *setpoint, uint16_t count, float threshold);

// Compute rebound percentage (relative to setpoint)
float autotuneMetricsComputeRebound(const float *gyro, const float *setpoint, uint16_t count);

// ============================================================================
// Timing Metrics
// ============================================================================

// Compute settling time (time to enter settling band)
float autotuneMetricsComputeSettlingTime(const float *gyro, const float *setpoint, 
                                          uint16_t count, float loopTimeUs);

// Compute lag (time to 50% of target)
float autotuneMetricsComputeLag(const float *gyro, const float *setpoint,
                                 uint16_t count, float loopTimeUs);

// ============================================================================
// Peak Detection
// ============================================================================

// Find first peak in response
// Returns index of peak, or -1 if not found
int autotuneMetricsFindFirstPeak(const float *gyro, uint16_t count);

// Find all peaks in response (for rebound analysis)
// Returns number of peaks found
int autotuneMetricsFindAllPeaks(const float *gyro, uint16_t count, 
                                 int *peakIndices, int maxPeaks);
