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
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

// Window state machine for debug visibility
typedef enum {
    FF_WINDOW_IDLE = 0,      // Setpoint below threshold
    FF_WINDOW_RISING,        // In tracking window, acceleration positive
    FF_WINDOW_PEAK,          // In window, acceleration slowing
    FF_WINDOW_FALLING,       // Exiting window
    FF_WINDOW_SETTLING,      // Post-maneuver settle time
    FF_WINDOW_ADJUSTING      // Applying gain change
} ffWindowState_e;

// Bracket convergence state
typedef enum {
    FF_BRACKET_SEARCHING = 0,  // No bracket established yet
    FF_BRACKET_BRACKETED,      // Optimal is between two known gains
    FF_BRACKET_CONVERGED       // Converged to optimal
} ffBracketState_e;

// Assessment values for debug
typedef enum {
    FF_ASSESS_LAG = -1,       // Gyro lagging setpoint (need more FF)
    FF_ASSESS_OPTIMAL = 0,    // Within deadband
    FF_ASSESS_LEAD = 1        // Gyro leading/overshoot (need less FF)
} ffAssessment_e;

void ffAutotuneInit(void);
void ffAutotuneUpdate(int axis, float setpoint, float gyroRate, float setpointDelta, timeUs_t currentTimeUs);

bool ffAutotuneIsActive(void);
uint8_t ffAutotuneGetGain(int axis);
bool ffAutotuneNeedsSave(void);
void ffAutotuneSaveGains(void);
void ffAutotuneReset(void);
