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

#include "pg/pg.h"

typedef struct ffAutotuneConfig_s {
    // Phase 1: F-term tuning
    uint8_t  enabled;           // Master enable
    uint16_t setpoint_low;      // Min setpoint for tracking window (deg/s)
    uint16_t setpoint_high;     // Max setpoint for tracking window (deg/s)
    uint16_t min_accel;         // Min acceleration (x100 = deg/s^2)
    uint8_t  gain_step;         // F adjustment step size
    uint8_t  gain_max;          // Maximum allowed FF gain (absolute, for bracket search)
    uint8_t  gain_min;          // Minimum allowed FF gain (absolute, for bracket search)
    uint8_t  error_deadband;    // Error deadband for "optimal" (deg/s)
    uint8_t  converge_threshold;// Bracket width to declare converged
    int8_t   f_adj_roll;        // Learned roll F adjustment (persisted)
    int8_t   f_adj_pitch;       // Learned pitch F adjustment (persisted)

    // Phase 2: P/D ratio tuning
    uint8_t  pd_enabled;        // Enable Phase 2 P/D tuning
    uint8_t  ring_window_ms;    // Ringing analysis window duration (ms)
    uint8_t  ring_threshold;    // Ringing score threshold for adjustment
    uint8_t  ring_deadband;     // Error deadband for zero-crossing detection (deg/s)
    uint8_t  p_step;            // P-term adjustment step size per maneuver
    uint8_t  d_step;            // D-term adjustment step size (Phase 2b)
    uint8_t  p_adjust_max;      // Maximum cumulative P adjustment magnitude
    uint8_t  d_adjust_max;      // Maximum cumulative D adjustment magnitude
    int8_t   p_adj_roll;        // Learned Roll P adjustment (persisted)
    int8_t   p_adj_pitch;       // Learned Pitch P adjustment (persisted)
    int8_t   d_adj_roll;        // Learned Roll D adjustment (persisted)
    int8_t   d_adj_pitch;       // Learned Pitch D adjustment (persisted)

    // Phase 2b: Noise reduction (filter adjustment + ratio-preserving gain scale)
    uint16_t noise_floor;       // Absolute noise score (avg|D|×10) below which noise is acceptable
    uint8_t  noise_threshold;   // Noise score improvement threshold (%) to continue reducing
    uint16_t gyro_noise_threshold; // Gyro noise score above which noise is filter-related
    uint8_t  lpf2_step;         // Gyro LPF2 cutoff reduction step per iteration (Hz)
    uint16_t lpf2_min;          // Minimum gyro LPF2 cutoff frequency (Hz)
    uint8_t  dterm_lpf2_step;   // D-term LPF2 cutoff reduction step per iteration (Hz)
    uint16_t dterm_lpf2_min;    // Minimum D-term LPF2 cutoff frequency (Hz)
    uint8_t  gain_scale_step;   // Percentage step per noise trigger (default 5)
    uint8_t  gain_scale_min;    // Minimum gain scale percent (default 50)
    int16_t  lpf2_adj;          // Persisted gyro LPF2 cutoff adjustment (Hz)
    int16_t  dterm_lpf2_adj;    // Persisted D-term LPF2 cutoff adjustment (Hz)
    uint8_t  gain_scale_roll;   // Persisted roll gain scale percent (default 100)
    uint8_t  gain_scale_pitch;  // Persisted pitch gain scale percent (default 100)

    // D noise ceiling learning
    int8_t   d_noise_ceiling_roll;  // Learned D ceiling for roll (0 = not learned)
    int8_t   d_noise_ceiling_pitch; // Learned D ceiling for pitch (0 = not learned)

    // Phase 0: Per-motor gain correction (permil deviation from 1.0×)
    int16_t  motor_trim_1;          // Motor 0 gain correction (0 = 1.000×, 56 = 1.056×)
    int16_t  motor_trim_2;          // Motor 1 gain correction
    int16_t  motor_trim_3;          // Motor 2 gain correction
    int16_t  motor_trim_4;          // Motor 3 gain correction
} ffAutotuneConfig_t;

PG_DECLARE(ffAutotuneConfig_t, ffAutotuneConfig);
