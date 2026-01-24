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

// Initialize rollback system
void autotuneRollbackInit(void);

// ============================================================================
// Parameter State Management
// ============================================================================

// Initialize a parameter state with current value and base step
void autotuneRollbackInitParam(parameterState_t *param, float currentValue, float baseStep);

// Store current value as previous (before making a change)
void autotuneRollbackSaveState(parameterState_t *param);

// ============================================================================
// Trust System
// ============================================================================

// Report good result - increase trust
void autotuneRollbackReportGood(parameterState_t *param);

// Report bad result - decrease trust
void autotuneRollbackReportBad(parameterState_t *param);

// Get current trust score (0.0 - 1.0)
float autotuneRollbackGetTrust(const parameterState_t *param);

// ============================================================================
// Step Size Computation
// ============================================================================

// Get effective step size (baseStep * trust * aggressiveness)
float autotuneRollbackGetStepSize(const parameterState_t *param, float aggressiveness);

// Compute new value after increase
float autotuneRollbackComputeIncrease(const parameterState_t *param, float aggressiveness);

// Compute new value after decrease
float autotuneRollbackComputeDecrease(const parameterState_t *param, float aggressiveness);

// ============================================================================
// Rollback Operations
// ============================================================================

// Perform rollback - restore previous value
void autotuneRollbackRevert(parameterState_t *param);

// Check if rollback is available (previous value different from current)
bool autotuneRollbackAvailable(const parameterState_t *param);

// ============================================================================
// Gain Limit Enforcement
// ============================================================================

// Clamp gain value to allowed range
float autotuneRollbackClampGain(float value, float minVal, float maxVal);

// Check if at minimum limit
bool autotuneRollbackAtMinLimit(const parameterState_t *param, float minVal);

// Check if at maximum limit
bool autotuneRollbackAtMaxLimit(const parameterState_t *param, float maxVal);
