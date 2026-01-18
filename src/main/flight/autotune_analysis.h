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

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "common/time.h"
#include "flight/autotune_types.h"

// ============================================================================
// ANALYSIS FUNCTIONS
// ============================================================================

// Main analysis entry point - analyzes collected samples
void autotuneAnalyzeResponse(
    const float *gyroSamples,
    const float *setpointSamples,
    const float *dtermSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz,
    autotuneMetrics_t *metricsOut
);

// Classify the response based on metrics
autotuneResponseClass_e autotuneClassifyResponse(const autotuneMetrics_t *metrics);

// Determine which gain is causing issues and what to adjust
void autotuneAttributeGains(
    const autotuneMetrics_t *metrics,
    autotuneResponseClass_e responseClass,
    autotuneAttribution_t *attributionOut
);

// Calculate a score for the current tuning (lower is better)
float autotuneCalculateScore(const autotuneMetrics_t *metrics);

// ============================================================================
// FILTER ANALYSIS FUNCTIONS
// ============================================================================

// Analyze noise profile from throttle punch data
void autotuneAnalyzeNoise(
    const float *gyroSamples,
    const float *throttleSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz,
    autotuneFilterAnalysis_t *filterOut
);

// Check if noise is within acceptable limits
bool autotuneIsNoiseAcceptable(float noiseRms, float targetNoise);

// ============================================================================
// TERM-SPECIFIC METRIC MEASUREMENT FUNCTIONS
// ============================================================================

// Get the appropriate metric type for a tunable parameter
tuneMetric_e getMetricForParameter(tuneParameter_e param);

// Get target value for a specific metric type
float getTargetForMetric(tuneMetric_e metric);

// Measure oscillation amplitude for D-term optimization
// Returns peak-to-peak oscillation amplitude in the settling region
float measureOscillation(
    const float *gyroSamples,
    const float *dtermSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz
);

// Measure setpoint tracking accuracy for P-term optimization
// Returns tracking accuracy as percentage (0-100%, higher is better)
float measureSetpointTracking(
    const float *gyroSamples,
    const float *setpointSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz
);

// Measure stick tracking during rapid movements for F-term optimization
// Returns velocity-weighted lag (lower is better, 0 = perfect tracking)
float measureStickTracking(
    const float *gyroSamples,
    const float *setpointSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz
);

// Measure long-term error/drift for I-term optimization
// Returns accumulated error over time (lower is better)
float measureLongTermError(
    const float *gyroSamples,
    const float *setpointSamples,
    uint16_t sampleCount,
    uint16_t sampleRateHz
);

// Get metric value from pre-computed metrics structure
// Convenient wrapper for Newton's method to get the right metric
float getMetricFromAnalysis(const autotuneMetrics_t *metrics, tuneMetric_e metricType);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

// Calculate RMS of a signal segment
float autotuneCalculateRms(const float *samples, uint16_t start, uint16_t end);

// Find peak value in a range
float autotuneFindPeak(const float *samples, uint16_t start, uint16_t end, uint16_t *peakIndex);

// Detect zero crossings for oscillation frequency
float autotuneDetectOscillationFreq(const float *samples, uint16_t count, uint16_t sampleRateHz);
