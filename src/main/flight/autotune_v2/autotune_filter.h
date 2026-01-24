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

#include "common/time.h"
#include "flight/autotune_v2/autotune_types.h"

// ============================================================================
// Initialization
// ============================================================================

// Initialize filter characterization system
void autotuneFilterInit(void);

// Reset filter characterization state
void autotuneFilterReset(void);

// ============================================================================
// Throttle Sweep
// ============================================================================

// Update filter characterization during throttle sweep
// Returns true when characterization is complete
bool autotuneFilterUpdate(timeUs_t currentTimeUs, float throttle, float noiseLevel);

// Check if throttle sweep is complete (sufficient range covered)
bool autotuneFilterSweepComplete(void);

// Get filter characterization state
const filterCharState_t* autotuneFilterGetState(void);

// ============================================================================
// Filter Recommendations
// ============================================================================

// Compute recommended filter settings based on characterization
void autotuneFilterComputeRecommendations(void);

// Get recommended LPF cutoff frequency
float autotuneFilterGetRecommendedLpf(void);

// Get recommended notch 1 frequency (0 if not needed)
float autotuneFilterGetRecommendedNotch1(void);

// Get recommended notch 2 frequency (0 if not needed)
float autotuneFilterGetRecommendedNotch2(void);

// Set recommended LPF cutoff frequency (for tightening during noise confirm)
void autotuneFilterSetRecommendedLpf(float lpfHz);

// ============================================================================
// Filter Application
// ============================================================================

// Apply recommended filter settings to the flight controller
void autotuneFilterApplyRecommendations(void);

// Rollback filter changes to previous values
void autotuneFilterRollback(void);

// ============================================================================
// Noise Measurement
// ============================================================================

// Get current noise level at hover
float autotuneFilterGetHoverNoise(void);

// Check if noise is acceptable (below ceiling)
bool autotuneFilterNoiseAcceptable(void);
