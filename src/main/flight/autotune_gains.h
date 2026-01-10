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
#include "flight/autotune_types.h"

// ============================================================================
// GAIN ADJUSTMENT FUNCTIONS
// ============================================================================

// Apply gain adjustments based on analysis results
// Returns true if gains were changed
bool autotuneApplyGainAdjustment(
    autotuneRuntime_t *runtime,
    const autotuneAttribution_t *attribution,
    autotuneResponseClass_e responseClass
);

// Calculate adjustment step size based on confidence and history
float autotuneCalculateAdjustmentStep(
    const autotuneRuntime_t *runtime,
    autotuneGainAttribution_e gainType,
    autotuneAdjustDir_e direction
);

// Save current gains to history
void autotuneSaveToHistory(autotuneRuntime_t *runtime, float score);

// Check if we've found a good solution
bool autotuneCheckConvergence(const autotuneRuntime_t *runtime);

// Restore best gains found
void autotuneRestoreBestGains(autotuneRuntime_t *runtime);

// ============================================================================
// FILTER ADJUSTMENT FUNCTIONS
// ============================================================================

// Apply filter adjustments based on noise analysis
bool autotuneApplyFilterAdjustment(
    const autotuneFilterAnalysis_t *filterAnalysis,
    float currentNoise,
    float targetNoise
);
