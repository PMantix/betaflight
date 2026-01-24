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

// Initialize event detection system
void autotuneEventInit(void);

// Reset event detector state (call when starting new event detection)
void autotuneEventReset(void);

// ============================================================================
// Event Detection
// ============================================================================

// Update event detection (call every loop)
// Returns true when a complete event is ready
bool autotuneEventUpdate(timeUs_t currentTimeUs, uint8_t axis, float stickDeflection, float crossAxisPosition, float throttle);

// Get the current event data (valid when autotuneEventUpdate returns true)
const eventData_t* autotuneEventGetData(void);

// Get current event detector state
eventState_e autotuneEventGetState(void);

// ============================================================================
// Event Buffer
// ============================================================================

// Start capturing gyro/setpoint samples for metrics analysis
void autotuneEventBufferStart(void);

// Add sample to event buffer
void autotuneEventBufferAddSample(float gyro, float setpoint);

// Stop capturing and prepare buffer for analysis
void autotuneEventBufferStop(void);

// Get event buffer for metrics analysis
const eventBuffer_t* autotuneEventBufferGet(void);

// ============================================================================
// Quality Gates
// ============================================================================

// Check if deflection meets minimum threshold
bool autotuneEventCheckDeflection(float deflection);

// Check if cross-axis movement is acceptable
bool autotuneEventCheckCrossAxis(float crossAxis);

// Check if throttle is within acceptable band
bool autotuneEventCheckThrottle(float throttle);

// Check all quality gates for current event
bool autotuneEventCheckQuality(const eventData_t *event);
