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

#include "platform.h"

#ifdef USE_FF_AUTOTUNE

#include "pg/pg_ids.h"
#include "pg/ff_autotune.h"

PG_REGISTER_WITH_RESET_TEMPLATE(ffAutotuneConfig_t, ffAutotuneConfig, PG_FF_AUTOTUNE_CONFIG, 0);

PG_RESET_TEMPLATE(ffAutotuneConfig_t, ffAutotuneConfig,
    .enabled = 0,
    .setpoint_low = 100,        // 100 deg/s
    .setpoint_high = 600,       // 600 deg/s
    .min_accel = 100,           // 100 × 100 = 10000 deg/s²
    .gain_step = 5,             // Step size for searching
    .gain_max = 200,            // Max gain limit
    .gain_min = 0,              // Min gain limit
    .error_deadband = 10,       // 10 deg/s deadband
    .converge_threshold = 3,    // Converge when bracket ≤ 3
    .gain_roll = 0,             // Start with no FF
    .gain_pitch = 0,            // Start with no FF
);

#endif // USE_FF_AUTOTUNE
