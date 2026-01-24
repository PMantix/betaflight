/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "pg/pg.h"

typedef struct autotuneConfig_s {
    uint8_t enabled;            // Autotune enabled (0 = off, 1 = on)
    uint8_t axes;               // Axis bitmask: 1=Roll, 2=Pitch, 4=Yaw (7=all)
    uint8_t aggressiveness;     // Aggressiveness 0-100 (50 = balanced)
    uint8_t pStepPercent;       // Base P step size percentage (default 10)
    uint8_t dStepPercent;       // Base D step size percentage (default 10)
    uint8_t fStepPercent;       // Base F step size percentage (default 5)
    uint8_t tuneFeedforward;    // Enable F tuning (0 = skip, 1 = tune)
    uint8_t minEvents;          // Minimum events before deciding (default 1)
    uint8_t maxEventsPerAxis;   // Maximum events per axis before abort (default 10)
    uint8_t stickThreshold;     // Minimum stick deflection degrees (default 15)
    uint8_t crossAxisThreshold; // Maximum cross-axis deflection degrees (default 10)
    uint8_t overshootTargetLow; // Lower bound of target overshoot % (default 5)
    uint8_t overshootTargetHigh;// Upper bound of target overshoot % (default 10)
} autotuneConfig_t;

PG_DECLARE(autotuneConfig_t, autotuneConfig);
