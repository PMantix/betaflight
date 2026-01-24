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

#include <math.h>

#include "common/maths.h"

#include "flight/autotune_v2/autotune_rollback.h"
#include "flight/autotune_v2/autotune_debug.h"

// ============================================================================
// Initialization
// ============================================================================

void autotuneRollbackInit(void)
{
    // Nothing to initialize globally
}

// ============================================================================
// Parameter State Management
// ============================================================================

void autotuneRollbackInitParam(parameterState_t *param, float currentValue, float baseStep)
{
    if (!param) {
        return;
    }
    
    param->currentValue = currentValue;
    param->previousValue = currentValue;
    param->trustScore = AUTOTUNE_TRUST_INITIAL;
    param->baseStepSize = baseStep;
    param->consecutiveGood = 0;
    param->consecutiveBad = 0;
}

void autotuneRollbackSaveState(parameterState_t *param)
{
    if (!param) {
        return;
    }
    
    param->previousValue = param->currentValue;
}

// ============================================================================
// Trust System
// ============================================================================

void autotuneRollbackReportGood(parameterState_t *param)
{
    if (!param) {
        return;
    }
    
    param->consecutiveGood++;
    param->consecutiveBad = 0;
    
    // Increase trust
    param->trustScore += AUTOTUNE_TRUST_INCREASE;
    if (param->trustScore > AUTOTUNE_TRUST_MAX) {
        param->trustScore = AUTOTUNE_TRUST_MAX;
    }
}

void autotuneRollbackReportBad(parameterState_t *param)
{
    if (!param) {
        return;
    }
    
    param->consecutiveBad++;
    param->consecutiveGood = 0;
    
    // Decrease trust
    param->trustScore -= AUTOTUNE_TRUST_DECREASE;
    if (param->trustScore < AUTOTUNE_TRUST_MIN) {
        param->trustScore = AUTOTUNE_TRUST_MIN;
    }
}

float autotuneRollbackGetTrust(const parameterState_t *param)
{
    if (!param) {
        return AUTOTUNE_TRUST_INITIAL;
    }
    
    return param->trustScore;
}

// ============================================================================
// Step Size Computation
// ============================================================================

float autotuneRollbackGetStepSize(const parameterState_t *param, float aggressiveness)
{
    if (!param) {
        return 0.0f;
    }
    
    // Effective step = baseStep * trust * (0.5 + 0.5 * aggressiveness)
    float effectiveStep = param->baseStepSize * param->trustScore;
    effectiveStep *= (0.5f + 0.5f * aggressiveness);
    
    // Clamp to limits
    const float minStep = AUTOTUNE_STEP_MIN_PCT / 100.0f;
    const float maxStep = AUTOTUNE_STEP_MAX_PCT / 100.0f;
    
    return constrainf(effectiveStep, minStep, maxStep);
}

float autotuneRollbackComputeIncrease(const parameterState_t *param, float aggressiveness)
{
    if (!param) {
        return 0.0f;
    }
    
    const float step = autotuneRollbackGetStepSize(param, aggressiveness);
    return param->currentValue * (1.0f + step);
}

float autotuneRollbackComputeDecrease(const parameterState_t *param, float aggressiveness)
{
    if (!param) {
        return 0.0f;
    }
    
    const float step = autotuneRollbackGetStepSize(param, aggressiveness);
    return param->currentValue * (1.0f - step);
}

// ============================================================================
// Rollback Operations
// ============================================================================

void autotuneRollbackRevert(parameterState_t *param)
{
    if (!param) {
        return;
    }
    
    param->currentValue = param->previousValue;
}

bool autotuneRollbackAvailable(const parameterState_t *param)
{
    if (!param) {
        return false;
    }
    
    // Rollback available if previous differs from current
    const float diff = fabsf(param->currentValue - param->previousValue);
    return diff > 0.001f;
}

// ============================================================================
// Gain Limit Enforcement
// ============================================================================

float autotuneRollbackClampGain(float value, float minVal, float maxVal)
{
    return constrainf(value, minVal, maxVal);
}

bool autotuneRollbackAtMinLimit(const parameterState_t *param, float minVal)
{
    if (!param) {
        return false;
    }
    
    return param->currentValue <= minVal;
}

bool autotuneRollbackAtMaxLimit(const parameterState_t *param, float maxVal)
{
    if (!param) {
        return false;
    }
    
    return param->currentValue >= maxVal;
}

#endif // USE_AUTOTUNE_V2
