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

// State machine states
typedef enum {
    AUTOTUNE_STATE_IDLE = 0,
    AUTOTUNE_STATE_SETUP,
    AUTOTUNE_STATE_WAIT_STABLE,
    AUTOTUNE_STATE_EXCITE,
    AUTOTUNE_STATE_MEASURE,
    AUTOTUNE_STATE_ANALYZE,
    AUTOTUNE_STATE_ADJUST,
    AUTOTUNE_STATE_COMPLETE,
    AUTOTUNE_STATE_ABORTED,
} autotuneState_e;

// Public functions
void autotuneInit(void);
void autotuneUpdate(timeUs_t currentTimeUs);
bool autotuneIsActive(void);
autotuneState_e autotuneGetState(void);
const char* autotuneGetStateName(void);
float autotuneModifySetpoint(uint8_t axis, float setpoint, timeUs_t currentTimeUs);
