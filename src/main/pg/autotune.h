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
    
    // Phase 1 parameters
    uint8_t  phase1_start_p_percent;      // Starting P % (50-90)
    uint8_t  phase1_start_d_percent;      // Starting D % (50-90)
    uint8_t  phase1_initial_ratio;        // Initial D/P ratio × 100 (40-100)
    uint8_t  phase1_target_overshoot_min; // Min overshoot for critical damping (0-20)
    uint8_t  phase1_target_overshoot_max; // Max overshoot for critical damping (5-30)
    uint8_t  phase1_max_iterations;       // Max iterations for Phase 1 (3-10)
    
    // Phase 2 parameters
    uint8_t  phase2_aggressive_multiplier;  // Aggressive scale factor × 100 (110-150)
    uint8_t  phase2_cautious_multiplier;    // Cautious scale factor × 100 (105-125)
    uint8_t  phase2_max_iterations;         // Max iterations for Phase 2 (5-15)
    
    // Phase 3 parameters
    uint8_t  phase3_max_iterations;         // Max iterations for Phase 3 (1-5)
} autotuneConfig_t;

PG_DECLARE(autotuneConfig_t, autotuneConfig);
