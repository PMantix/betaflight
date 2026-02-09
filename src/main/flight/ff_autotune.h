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
    FF_WINDOW_IDLE = 0,      // Waiting for rise: neutral setpoint, low accel
    FF_WINDOW_RISING,        // Setpoint in range, accel away from center - MEASURE HERE
    FF_WINDOW_ADJUSTING,     // No longer rising: wait 100ms then adjust F term
    FF_WINDOW_WAITING        // Wait for setpoint to return to neutral before next cycle
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

// Autotune phase (per-axis)
typedef enum {
    FF_AUTOTUNE_PHASE1_FF = 0,    // Tuning feedforward gain
    FF_AUTOTUNE_PHASE2_PD,        // 2a: Tuning P/D ratio for ringing suppression
    FF_AUTOTUNE_PHASE2B_SCALE,    // 2b: P/D scale-down for noise reduction
    FF_AUTOTUNE_COMPLETE           // All metrics converged
} ffAutotunePhase_e;

// Ringing assessment
typedef enum {
    FF_RING_WELL_DAMPED = 0,  // No adjustment needed
    FF_RING_MILD,             // Borderline ringing
    FF_RING_RINGING           // Needs P/D adjustment
} ffRingAssessment_e;

// Noise assessment (Phase 2b)
typedef enum {
    FF_NOISE_HIGH = 0,        // Noise still high, continue reducing
    FF_NOISE_ACCEPTABLE,      // Noise acceptable, stop
    FF_NOISE_MINIMAL          // Noise negligible
} ffNoiseAssessment_e;

void ffAutotuneInit(void);
void ffAutotuneUpdate(int axis, float setpoint, float gyroRate, float setpointDelta, timeUs_t currentTimeUs);

bool ffAutotuneIsActive(void);
bool ffAutotuneHasLearnedGains(void);
uint8_t ffAutotuneGetGain(int axis);
bool ffAutotuneNeedsSave(void);
void ffAutotuneSaveGains(void);
void ffAutotuneOnDisarm(void);
void ffAutotuneReset(void);

// Phase 2 API
bool ffAutotuneIsPhase2Active(void);
int16_t ffAutotuneGetPAdjustment(int axis);
int16_t ffAutotuneGetDAdjustment(int axis);
int16_t ffAutotuneGetLpf2Adjustment(void);

// COMPLETE notification wiggle offset (added to setpoint in pid.c)
float ffAutotuneGetWiggleOffset(int axis);
