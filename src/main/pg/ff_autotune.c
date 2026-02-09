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

PG_REGISTER_WITH_RESET_TEMPLATE(ffAutotuneConfig_t, ffAutotuneConfig, PG_FF_AUTOTUNE_CONFIG, 5);

PG_RESET_TEMPLATE(ffAutotuneConfig_t, ffAutotuneConfig,
    // Phase 1: F-term tuning
    .enabled = 0,
    .setpoint_low = 100,        // 100 deg/s
    .setpoint_high = 600,       // 600 deg/s
    .min_accel = 100,           // 100 x 100 = 10000 deg/s^2
    .gain_step = 5,             // Step size for searching
    .gain_max = 200,            // Max gain limit
    .gain_min = 0,              // Min gain limit
    .error_deadband = 10,       // 10 deg/s deadband
    .converge_threshold = 3,    // Converge when bracket <= 3
    .gain_roll = 0,             // Start with no FF
    .gain_pitch = 0,            // Start with no FF
    // Phase 2: P/D ratio tuning
    .pd_enabled = 1,            // Enabled by default (requires Phase 1 convergence)
    .ring_window_ms = 150,      // 150ms analysis window
    .ring_threshold = 20,       // Ringing score threshold
    .ring_deadband = 5,         // 5 deg/s zero-crossing deadband
    .p_step = 2,                // P adjustment step
    .d_step = 1,                // D adjustment step (Phase 2b)
    .p_adjust_max = 20,         // Max cumulative P reduction
    .d_adjust_max = 10,         // Max cumulative D increase
    .p_adj_roll = 0,            // No initial P adjustment
    .p_adj_pitch = 0,           // No initial P adjustment
    .d_adj_roll = 0,            // No initial D adjustment
    .d_adj_pitch = 0,           // No initial D adjustment
    // Phase 2b: Noise reduction (filter adjustment + P/D scale-down)
    .noise_floor = 600,         // Absolute noise score (avg|D|×10) below which noise is acceptable
    .noise_threshold = 10,      // 10% noise improvement threshold to continue
    .lpf2_step = 25,            // 25 Hz LPF2 cutoff reduction per iteration
    .lpf2_min = 150,            // Minimum 150 Hz LPF2 cutoff
    .scale_step = 1,            // 1 unit step per iteration
    .scale_max = 15,            // Max 15 units cumulative scale-down
    .lpf2_adj = 0,              // No initial LPF2 adjustment
    .scale_adj_roll = 0,        // No initial scale adjustment
    .scale_adj_pitch = 0,       // No initial scale adjustment
);

#endif // USE_FF_AUTOTUNE
