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

#include "platform.h"

#ifdef USE_AUTOTUNE_V2

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/autotune.h"

PG_REGISTER_WITH_RESET_TEMPLATE(autotuneConfig_t, autotuneConfig, PG_AUTOTUNE_CONFIG, 0);

PG_RESET_TEMPLATE(autotuneConfig_t, autotuneConfig,
    .enabled = 0,                   // Disabled by default
    .axes = 7,                      // All axes (Roll + Pitch + Yaw)
    .aggressiveness = 50,           // Balanced (0-100)
    .pStepPercent = 10,             // 10% base P step
    .dStepPercent = 10,             // 10% base D step
    .fStepPercent = 5,              // 5% base F step
    .tuneFeedforward = 1,           // Enable F tuning by default
    .minEvents = 1,                 // Single event decisions
    .maxEventsPerAxis = 10,         // Max 10 events per axis
    .stickThreshold = 15,           // 15 degrees minimum deflection
    .crossAxisThreshold = 10,       // 10 degrees max cross-axis
    .overshootTargetLow = 5,        // 5% lower bound
    .overshootTargetHigh = 10,      // 10% upper bound
);

#endif // USE_AUTOTUNE_V2
