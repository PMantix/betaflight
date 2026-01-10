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

typedef struct autotuneConfig_s {
    uint8_t  autotune_enabled;        // Master enable flag
    uint8_t  max_iterations;          // Max tune iterations per axis (1-50)
    uint8_t  safety_margin;           // Attitude limit during tune (degrees)
    uint8_t  save_on_complete;        // 1=auto-save, 0=require manual save
    uint8_t  target_overshoot_min;    // Minimum target overshoot % (0-20)
    uint8_t  target_overshoot_max;    // Maximum target overshoot % (10-40)
    uint8_t  noise_target;            // Target noise level (deg/s RMS)
    uint8_t  gain_step_percent;       // Base gain adjustment step %
} autotuneConfig_t;

PG_DECLARE(autotuneConfig_t, autotuneConfig);
