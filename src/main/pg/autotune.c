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

PG_REGISTER_WITH_RESET_TEMPLATE(autotuneConfig_t, autotuneConfig, PG_AUTOTUNE_CONFIG, 1);

PG_RESET_TEMPLATE(autotuneConfig_t, autotuneConfig,
    .autotune_enabled = 1,
    .max_iterations = 20,
    .safety_margin = 45,                // 45 degrees - allow flips/rolls
    .save_on_complete = 0,              // Don't auto-save
    .target_overshoot_min = 5,          // 5% minimum
    .target_overshoot_max = 18,         // 18% maximum
    .noise_target = 4,                  // 4 deg/s RMS
    .gain_step_percent = 8,             // 8% step size
);

#endif // USE_AUTOTUNE
