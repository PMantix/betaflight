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

#include "platform.h"

#ifdef USE_AUTOTUNE

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/autotune.h"

PG_REGISTER_WITH_RESET_TEMPLATE(autotuneConfig_t, autotuneConfig, PG_AUTOTUNE_CONFIG, 0);

PG_RESET_TEMPLATE(autotuneConfig_t, autotuneConfig,
    .autotune_enabled = 1,
    .autotune_mode = AUTOTUNE_MODE_STEP,
    .autotune_axes = 0x03,              // Roll + Pitch (bits 0 and 1)
    .step_amplitude = 100,              // 100 deg/sec
    .step_duration_ms = 100,            // 100ms
    .target_overshoot = 10,             // 10%
    .max_iterations = 10,
    .safety_margin = 30,                // 30 degrees
    .save_on_complete = 0,              // Don't auto-save
);

#endif // USE_AUTOTUNE
