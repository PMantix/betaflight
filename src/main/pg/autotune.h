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
#include "pg/pg.h"

typedef enum {
    AUTOTUNE_MODE_STEP = 0,
    AUTOTUNE_MODE_DOUBLET,
    AUTOTUNE_MODE_RELAY,
} autotuneMode_e;

typedef struct autotuneConfig_s {
    uint8_t  autotune_enabled;        // Master enable flag
    uint8_t  autotune_mode;           // autotuneMode_e
    uint8_t  autotune_axes;           // Bitmask: bit0=roll, bit1=pitch, bit2=yaw
    uint8_t  step_amplitude;          // Step size in degrees/sec (10-200)
    uint16_t step_duration_ms;        // Duration of step (50-500ms)
    uint8_t  target_overshoot;        // Desired overshoot % (0-30)
    uint8_t  max_iterations;          // Max tune iterations per axis (1-20)
    uint8_t  safety_margin;           // Attitude limit during tune (degrees)
    uint8_t  save_on_complete;        // 1=auto-save, 0=require manual save
} autotuneConfig_t;

PG_DECLARE(autotuneConfig_t, autotuneConfig);
