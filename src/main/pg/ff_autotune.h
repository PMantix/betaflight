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
    uint8_t  gain_step;         // Initial gain step size
    uint8_t  gain_max;          // Maximum allowed FF gain
    uint8_t  gain_min;          // Minimum allowed FF gain
    uint8_t  error_deadband;    // Error deadband for "optimal" (deg/s)
    uint8_t  converge_threshold;// Bracket width to declare converged
    uint8_t  gain_roll;         // Learned roll FF gain
    uint8_t  gain_pitch;        // Learned pitch FF gain

    // Phase 2: P/D ratio tuning
    uint8_t  pd_enabled;        // Enable Phase 2 P/D tuning
    uint8_t  ring_window_ms;    // Ringing analysis window duration (ms)
    uint8_t  ring_threshold;    // Ringing score threshold for adjustment
    uint8_t  ring_deadband;     // Error deadband for zero-crossing detection (deg/s)
    uint8_t  p_step;            // P-term adjustment step size per maneuver
    uint8_t  d_step;            // D-term adjustment step size (Phase 2b)
    uint8_t  p_adjust_max;      // Maximum cumulative P reduction
    uint8_t  d_adjust_max;      // Maximum cumulative D increase
    int8_t   p_adj_roll;        // Learned Roll P adjustment (persisted)
    int8_t   p_adj_pitch;       // Learned Pitch P adjustment (persisted)
    int8_t   d_adj_roll;        // Learned Roll D adjustment (persisted)
    int8_t   d_adj_pitch;       // Learned Pitch D adjustment (persisted)
} ffAutotuneConfig_t;

PG_DECLARE(ffAutotuneConfig_t, ffAutotuneConfig);
