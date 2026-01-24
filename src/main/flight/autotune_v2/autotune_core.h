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

// Initialize autotune system (called once at startup)
void autotuneInit(void);

// ============================================================================
// Main Update
// ============================================================================

// Main update function (called from PID loop at loop rate)
void autotuneUpdate(timeUs_t currentTimeUs);

// ============================================================================
// State Query
// ============================================================================

// Returns true if autotune is actively tuning (not IDLE or COMPLETE)
bool autotuneIsActive(void);

// Get current master state
autotuneState_e autotuneGetState(void);

// Get current axis being tuned (0=Roll, 1=Pitch, 2=Yaw)
uint8_t autotuneGetCurrentAxis(void);

// Get progress percentage (0-100)
uint8_t autotuneGetProgress(void);

// Get last reason code (for debugging/telemetry)
uint16_t autotuneGetReasonCode(void);

// Get last decision made
autotuneDecision_e autotuneGetLastDecision(void);

// ============================================================================
// Control
// ============================================================================

// Force abort autotune with reason code
void autotuneAbort(uint16_t reasonCode);

// Handle mode activation (called when autotune mode box changes)
void autotuneUpdateActivation(bool enabled, timeUs_t currentTimeUs);
