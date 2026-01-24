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
#include <string.h>

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "config/config.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"

#include "sensors/gyro.h"

#include "flight/autotune_v2/autotune_core.h"
#include "flight/autotune_v2/autotune_types.h"
#include "flight/autotune_v2/autotune_debug.h"
#include "flight/autotune_v2/autotune_event.h"
#include "flight/autotune_v2/autotune_filter.h"
#include "flight/autotune_v2/autotune_metrics.h"
#include "flight/autotune_v2/autotune_rollback.h"
#include "flight/autotune_v2/autotune_feedback.h"

#include "flight/pid.h"

#include "pg/autotune.h"

// ============================================================================
// Static State
// ============================================================================

static autotuneRuntime_t runtime;

// ============================================================================
// Forward Declarations - State Handlers
// ============================================================================

static void stateIdleEnter(timeUs_t currentTimeUs);
static void stateIdleUpdate(timeUs_t currentTimeUs);

static void stateHoverLockEnter(timeUs_t currentTimeUs);
static void stateHoverLockUpdate(timeUs_t currentTimeUs);

static void stateThrottleSweepEnter(timeUs_t currentTimeUs);
static void stateThrottleSweepUpdate(timeUs_t currentTimeUs);

static void stateNoiseConfirmEnter(timeUs_t currentTimeUs);
static void stateNoiseConfirmUpdate(timeUs_t currentTimeUs);

static void statePdRatioSeekEnter(timeUs_t currentTimeUs);
static void statePdRatioSeekUpdate(timeUs_t currentTimeUs);

static void statePdScaleUpEnter(timeUs_t currentTimeUs);
static void statePdScaleUpUpdate(timeUs_t currentTimeUs);

static void stateFTuneEnter(timeUs_t currentTimeUs);
static void stateFTuneUpdate(timeUs_t currentTimeUs);

static void statePdRetuneEnter(timeUs_t currentTimeUs);
static void statePdRetuneUpdate(timeUs_t currentTimeUs);

static void stateCompleteEnter(timeUs_t currentTimeUs);
static void stateCompleteUpdate(timeUs_t currentTimeUs);

static void advanceToNextAxisOrComplete(timeUs_t currentTimeUs);
static void checkTimeouts(timeUs_t currentTimeUs);

// ============================================================================
// State Transition
// ============================================================================

static void transitionToState(autotuneState_e newState, timeUs_t currentTimeUs)
{
    if (runtime.masterState == newState) {
        return;
    }

    runtime.masterState = newState;
    runtime.stateEntryTimeUs = currentTimeUs;
    
    // Trigger unique wiggle pattern for each state:
    // State 1 (HOVER_LOCK):     B           (quick nudge)
    // State 2 (THROTTLE_SWEEP): W           (full sway)
    // State 3 (NOISE_CONFIRM):  B-B         (dit-dit)
    // State 4 (PD_RATIO_SEEK):  W-W         (big milestone!)
    // State 5 (PD_SCALE_UP):    B-W         (dit-dah)
    // State 6 (F_TUNE):         W-B         (dah-dit)
    // State 7 (PD_RETUNE):      B-B-B       (dit-dit-dit)
    // State 8 (COMPLETE):       W-W-W       (celebration!)
    if (newState >= AUTOTUNE_STATE_HOVER_LOCK && newState <= AUTOTUNE_STATE_COMPLETE) {
        autotuneFeedbackStateAdvance((uint8_t)newState);
    }

    // Call enter handler for new state
    switch (newState) {
        case AUTOTUNE_STATE_IDLE:
            stateIdleEnter(currentTimeUs);
            break;
        case AUTOTUNE_STATE_HOVER_LOCK:
            stateHoverLockEnter(currentTimeUs);
            break;
        case AUTOTUNE_STATE_THROTTLE_SWEEP:
            stateThrottleSweepEnter(currentTimeUs);
            break;
        case AUTOTUNE_STATE_NOISE_CONFIRM:
            stateNoiseConfirmEnter(currentTimeUs);
            break;
        case AUTOTUNE_STATE_PD_RATIO_SEEK:
            statePdRatioSeekEnter(currentTimeUs);
            break;
        case AUTOTUNE_STATE_PD_SCALE_UP:
            statePdScaleUpEnter(currentTimeUs);
            break;
        case AUTOTUNE_STATE_F_TUNE:
            stateFTuneEnter(currentTimeUs);
            break;
        case AUTOTUNE_STATE_PD_RETUNE_AFTER_F:
            statePdRetuneEnter(currentTimeUs);
            break;
        case AUTOTUNE_STATE_COMPLETE:
            stateCompleteEnter(currentTimeUs);
            break;
        default:
            break;
    }

    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_STATE, newState);
}

// ============================================================================
// State Handlers - IDLE
// ============================================================================

static void stateIdleEnter(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    runtime.reasonCode = AUTOTUNE_REASON_NONE;
    runtime.lastDecision = AUTOTUNE_DECISION_NONE;
}

static void stateIdleUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    // Nothing to do in idle state
    // Activation is handled by autotuneUpdateActivation()
}

// ============================================================================
// State Handlers - HOVER_LOCK
// ============================================================================

static void stateHoverLockEnter(timeUs_t currentTimeUs)
{
    runtime.hoverLockStartUs = currentTimeUs;
    runtime.reasonCode = AUTOTUNE_REASON_NONE;
    
    // Initialize throttle tracking (normalize to 0-1 range)
    runtime.lastThrottle = rcCommand[THROTTLE] / 1000.0f;
    
    // Initialize filter characterization
    autotuneFilterReset();
}

static void stateHoverLockUpdate(timeUs_t currentTimeUs)
{
    // Get current throttle as float 0-1
    const float currentThrottle = rcCommand[THROTTLE] / 1000.0f;
    
    // Check sticks centered (roll, pitch, yaw < 5% deflection)
    // rcCommand for sticks is ±500, so 5% = 25
    const float stickThreshold = 500.0f * AUTOTUNE_STICK_CENTER_THRESHOLD;
    const bool sticksCentered = (fabsf(rcCommand[FD_ROLL]) < stickThreshold) &&
                                 (fabsf(rcCommand[FD_PITCH]) < stickThreshold) &&
                                 (fabsf(rcCommand[FD_YAW]) < stickThreshold);
    
    // Check throttle stable (within ±5% of last reading)
    const float throttleDelta = fabsf(currentThrottle - runtime.lastThrottle);
    const bool throttleStable = throttleDelta < AUTOTUNE_THROTTLE_STABLE_BAND;
    
    // Update last throttle for next iteration
    runtime.lastThrottle = currentThrottle;
    
    // If not stable, reset hover lock start time
    if (!sticksCentered || !throttleStable) {
        runtime.hoverLockStartUs = currentTimeUs;
    }
    
    // Check for timeout (10 seconds to achieve hover lock)
    if (currentTimeUs - runtime.stateEntryTimeUs > AUTOTUNE_HOVER_LOCK_TIMEOUT_US) {
        runtime.reasonCode = AUTOTUNE_REASON_ABORT_TIMEOUT;
        transitionToState(AUTOTUNE_STATE_IDLE, currentTimeUs);
        return;
    }
    
    // Check if hover locked for required duration (1 second)
    if (currentTimeUs - runtime.hoverLockStartUs >= AUTOTUNE_HOVER_LOCK_DURATION_US) {
        // Success! Capture baseline state and transition
        autotuneFeedbackHoverLocked();
        transitionToState(AUTOTUNE_STATE_THROTTLE_SWEEP, currentTimeUs);
    }
}

// ============================================================================
// State Handlers - THROTTLE_SWEEP
// ============================================================================

static void stateThrottleSweepEnter(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    runtime.reasonCode = AUTOTUNE_REASON_NONE;
    
    // Initialize throttle tracking for sweep
    runtime.lastThrottle = rcCommand[THROTTLE] / 1000.0f;
}

static void stateThrottleSweepUpdate(timeUs_t currentTimeUs)
{
    // Get current throttle as float 0-1
    const float currentThrottle = rcCommand[THROTTLE] / 1000.0f;
    
    // Update filter characterization with current throttle and noise
    // noiseLevel parameter is ignored - filter module measures directly from gyro
    autotuneFilterUpdate(currentTimeUs, currentThrottle, 0.0f);
    
    // Update debug with throttle range progress (use OVERSHOOT channel, not REASON)
    const filterCharState_t *filterState = autotuneFilterGetState();
    const float throttleRange = filterState->maxThrottleSeen - filterState->minThrottleSeen;
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_OVERSHOOT, (int16_t)(throttleRange * 100.0f));
    
    // Show sweep count in DECISION channel: sweepCount * 100 + stable hover progress
    // e.g., 100 = 1 sweep done, 200 = 2 sweeps done, 250 = 2 sweeps + 50% hover time
    int16_t sweepProgress = filterState->sweepCount * 100;
    if (filterState->sweepCount >= 2 && filterState->hoverStableStartUs > 0) {
        const timeDelta_t hoverMs = cmpTimeUs(currentTimeUs, filterState->hoverStableStartUs) / 1000;
        sweepProgress += (int16_t)(hoverMs * 50 / 2000);  // 0-50 for 0-2000ms
    }
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_DECISION, sweepProgress);
    
    // Check if sweep is complete (sufficient throttle range covered)
    if (autotuneFilterSweepComplete()) {
        // Compute filter recommendations based on noise profile
        autotuneFilterComputeRecommendations();
        
        // Log the recommended LPF
        AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_DECISION, (int16_t)autotuneFilterGetRecommendedLpf());
        
        // Trigger feedback and transition to noise confirm
        autotuneFeedbackFiltersSet();
        runtime.reasonCode = AUTOTUNE_REASON_FILTERS_SET;
        transitionToState(AUTOTUNE_STATE_NOISE_CONFIRM, currentTimeUs);
    }
    
    // Store current throttle for next iteration
    runtime.lastThrottle = currentThrottle;
}

// ============================================================================
// State Handlers - NOISE_CONFIRM
// ============================================================================

// Static variable for iteration tracking (reset in Enter, used in Update)
static uint8_t noiseConfirmIterations = 0;

static void stateNoiseConfirmEnter(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    runtime.reasonCode = AUTOTUNE_REASON_NONE;
    
    // CRIT-005 fix: Reset iteration counter on state entry
    noiseConfirmIterations = 0;
    
    // Apply recommended filter settings
    autotuneFilterApplyRecommendations();
}

static void stateNoiseConfirmUpdate(timeUs_t currentTimeUs)
{
    // Limit iterations to prevent infinite loop (SFA-006 fix)
    const uint8_t MAX_NOISE_CONFIRM_ITERATIONS = 5;
    
    // Wait for filters to stabilize (500ms)
    const timeDelta_t settlingDuration = 500000;  // 500ms
    if (cmpTimeUs(currentTimeUs, runtime.stateEntryTimeUs) < settlingDuration) {
        return;
    }
    
    // Increment iteration counter
    noiseConfirmIterations++;
    
    // Measure noise with new filter settings
    float currentNoise = autotuneFilterGetHoverNoise();
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_OVERSHOOT, (int16_t)(currentNoise * 10.0f));
    
    // Check if noise is acceptable OR max iterations reached
    if (autotuneFilterNoiseAcceptable() || noiseConfirmIterations >= MAX_NOISE_CONFIRM_ITERATIONS) {
        // Noise is good (or we gave up) - proceed to PD tuning
        runtime.reasonCode = AUTOTUNE_REASON_HOVER_LOCKED;
        noiseConfirmIterations = 0;  // Reset for next time
        transitionToState(AUTOTUNE_STATE_PD_RATIO_SEEK, currentTimeUs);
    } else {
        // Noise still too high - need tighter filters
        runtime.reasonCode = AUTOTUNE_REASON_NOISE_TOO_HIGH;
        
        // Get current LPF and tighten by 15%
        float currentLpf = autotuneFilterGetRecommendedLpf();
        float tighterLpf = currentLpf * 0.85f;
        
        // Clamp to minimum 100Hz
        if (tighterLpf < 100.0f) {
            tighterLpf = 100.0f;
        }
        
        // If we're already at minimum, just proceed anyway
        if (currentLpf <= 100.0f) {
            // Can't tighten further, proceed with current filters
            noiseConfirmIterations = 0;  // Reset for next time
            transitionToState(AUTOTUNE_STATE_PD_RATIO_SEEK, currentTimeUs);
        } else {
            // SFA-002 fix: Set the tighter LPF value before reapplying
            autotuneFilterSetRecommendedLpf(tighterLpf);
            autotuneFilterApplyRecommendations();
            runtime.stateEntryTimeUs = currentTimeUs;  // Reset settling timer
        }
    }
}

// ============================================================================
// Gain Application
// ============================================================================

static void applyPGain(uint8_t axis, float newP)
{
    // OBS-003 fix: Capture old value for logging
    uint8_t oldP = currentPidProfile->pid[axis].P;
    
    uint8_t pValue = constrain(lrintf(newP), AUTOTUNE_GAIN_MIN, AUTOTUNE_GAIN_MAX);
    currentPidProfile->pid[axis].P = pValue;
    
    // Log both old and new: debug[5]=new, debug[3]=delta (packed for audit)
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_P, pValue);
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_DECISION, (int16_t)(pValue - oldP));
}

// ============================================================================
// Consecutive Bad Event Tracking
// ============================================================================

#define AUTOTUNE_MAX_CONSECUTIVE_BAD    3   // Force advance after 3 consecutive bad events

static void trackBadEvent(axisTuneState_t *axis)
{
    axis->consecutiveBadEvents++;
}

static void trackGoodEvent(axisTuneState_t *axis)
{
    axis->consecutiveBadEvents = 0;
}

static bool shouldForceAdvanceOnBadEvents(const axisTuneState_t *axis)
{
    return axis->consecutiveBadEvents >= AUTOTUNE_MAX_CONSECUTIVE_BAD;
}

// ============================================================================
// PD Ratio Seek Decision (Bracket + Newton)
// ============================================================================

static autotuneDecision_e pdRatioSeekDecision(
    const eventMetrics_t *metrics,
    axisTuneState_t *axisState,
    float targetOvershoot,
    float *newP)
{
    float currentP = axisState->pState.currentValue;
    float overshoot = metrics->overshootPct;
    bracketState_t *bracket = &axisState->bracket;
    
    // === SAFETY FIRST: Rebound means we're too high ===
    if (metrics->hasRebound) {
        bracket->pHigh = currentP;
        bracket->osHigh = overshoot;
        bracket->haveHigh = true;
        
        // Rollback trust and track bad event
        autotuneRollbackReportBad(&axisState->pState);
        trackBadEvent(axisState);
        
        // Check for repeated failures
        if (shouldForceAdvanceOnBadEvents(axisState)) {
            *newP = bracket->haveLow ? bracket->pLow : currentP * 0.8f;
            return AUTOTUNE_DECISION_ADVANCE;
        }
        
        if (bracket->haveLow) {
            // Have lower bound - snap to it
            *newP = bracket->pLow;
            return AUTOTUNE_DECISION_ROLLBACK;
        }
        
        // Need to find lower bound - step down aggressively
        float step = autotuneRollbackGetStepSize(&axisState->pState, 1.0f);
        *newP = currentP * (1.0f - step * 1.5f);  // 1.5x step on rebound
        *newP = fmaxf(*newP, AUTOTUNE_GAIN_MIN);
        return AUTOTUNE_DECISION_DECREASE;
    }
    
    // === UPDATE BRACKET based on overshoot ===
    if (overshoot > targetOvershoot) {
        bracket->pHigh = currentP;
        bracket->osHigh = overshoot;
        bracket->haveHigh = true;
    } else {
        bracket->pLow = currentP;
        bracket->osLow = overshoot;
        bracket->haveLow = true;
    }
    
    // === CHECK CONVERGENCE ===
    const float tolerance = 2.0f;  // +/-2% is close enough
    if (fabsf(overshoot - targetOvershoot) < tolerance) {
        autotuneRollbackReportGood(&axisState->pState);
        trackGoodEvent(axisState);
        *newP = currentP;
        return AUTOTUNE_DECISION_ADVANCE;
    }
    
    // === PHASE 1: BRACKETING (still missing a bound) ===
    if (!bracket->haveLow || !bracket->haveHigh) {
        if (overshoot > targetOvershoot) {
            // Too much overshoot - decrease P
            float step = autotuneRollbackGetStepSize(&axisState->pState, 1.0f);
            *newP = currentP * (1.0f - step);
            *newP = fmaxf(*newP, AUTOTUNE_GAIN_MIN);
            autotuneRollbackReportGood(&axisState->pState);
            return AUTOTUNE_DECISION_DECREASE;
        } else {
            // Undershoot but starting assumption is high P has overshoot
            // This means we're already at a good point
            autotuneRollbackReportGood(&axisState->pState);
            *newP = currentP;
            return AUTOTUNE_DECISION_ADVANCE;
        }
    }
    
    // === PHASE 2: NEWTON REFINEMENT (have bracket) ===
    float bracketWidth = bracket->pHigh - bracket->pLow;
    
    // Check if bracket is narrow enough to stop
    const float minBracketWidth = 3.0f;  // 3 P units
    if (bracketWidth < minBracketWidth) {
        autotuneRollbackReportGood(&axisState->pState);
        *newP = bracket->pLow;  // Conservative choice
        return AUTOTUNE_DECISION_ADVANCE;
    }
    
    // Compute slope (overshoot vs P) using bracket endpoints
    float osDiff = bracket->osHigh - bracket->osLow;
    const float minSlope = 0.1f;
    
    if (fabsf(osDiff) > minSlope * bracketWidth) {
        // Valid slope - use Newton step
        float slope = osDiff / bracketWidth;
        float pNewton = currentP - (overshoot - targetOvershoot) / slope;
        
        // Clamp to bracket with 10% margin
        float margin = bracketWidth * 0.1f;
        pNewton = fmaxf(pNewton, bracket->pLow + margin);
        pNewton = fminf(pNewton, bracket->pHigh - margin);
        
        autotuneRollbackReportGood(&axisState->pState);
        *newP = pNewton;
        return (pNewton < currentP) ? AUTOTUNE_DECISION_DECREASE : AUTOTUNE_DECISION_INCREASE;
    }
    
    // Fallback: midpoint bisection
    autotuneRollbackReportGood(&axisState->pState);
    *newP = (bracket->pLow + bracket->pHigh) / 2.0f;
    return (*newP < currentP) ? AUTOTUNE_DECISION_DECREASE : AUTOTUNE_DECISION_INCREASE;
}

// ============================================================================
// State Handlers - PD_RATIO_SEEK
// ============================================================================

static void statePdRatioSeekEnter(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    runtime.reasonCode = AUTOTUNE_REASON_NONE;
    
    // Initialize axis state for current axis
    axisTuneState_t *axis = &runtime.axisState[runtime.currentAxisIndex];
    axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
    axis->eventCount = 0;
    axis->complete = false;
    
    // Store original gains for abort recovery
    axis->originalP = currentPidProfile->pid[runtime.currentAxisIndex].P;
    axis->originalD = currentPidProfile->pid[runtime.currentAxisIndex].D;
    axis->originalF = currentPidProfile->pid[runtime.currentAxisIndex].F;
    
    // Initialize P state with rollback support
    autotuneRollbackInitParam(&axis->pState, 
                               axis->originalP,
                               AUTOTUNE_P_STEP_BASE);
    
    // D stays fixed during ratio seek
    axis->dState.currentValue = axis->originalD;
    
    // Initialize bracket state
    axis->bracket.haveLow = false;
    axis->bracket.haveHigh = false;
    axis->bracket.pLow = 0.0f;
    axis->bracket.pHigh = 0.0f;
    axis->bracket.osLow = 0.0f;
    axis->bracket.osHigh = 0.0f;
    
    // Compute target overshoot from aggressiveness setting
    // aggressiveness 0-100 maps to overshoot:
    // 0 -> overshootTargetLow (5%), 100 -> overshootTargetHigh (10%)
    const float aggNorm = autotuneConfig()->aggressiveness / 100.0f;
    runtime.targetOvershootPct = autotuneConfig()->overshootTargetLow + 
        aggNorm * (autotuneConfig()->overshootTargetHigh - autotuneConfig()->overshootTargetLow);
    
    // Reset event detector for this axis
    autotuneEventReset();
    
    // Debug output
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_P, axis->originalP);
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_D, axis->originalD);
}

static void statePdRatioSeekUpdate(timeUs_t currentTimeUs)
{
    axisTuneState_t *axis = &runtime.axisState[runtime.currentAxisIndex];
    
    // Get current stick position for the active axis (normalized -1.0 to 1.0)
    // rcCommand is ±500, so divide by 500 for normalization
    const float stickDeflection = rcCommand[runtime.currentAxisIndex] / 500.0f;
    
    // Cross-axis is max of other two axes (normalized)
    float crossAxis = 0.0f;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        if (i != runtime.currentAxisIndex) {
            crossAxis = fmaxf(crossAxis, fabsf(rcCommand[i] / 500.0f));
        }
    }
    
    // Throttle normalized 0.0 to 1.0 (rcCommand[THROTTLE] is 0-1000)
    const float throttle = rcCommand[THROTTLE] / 1000.0f;
    
    switch (axis->substate) {
        case AXIS_SUBSTATE_WAIT_EVENT:
            {
                // Check if we're starting a new event
                eventState_e eventState = autotuneEventGetState();
                
                if (eventState == EVENT_STATE_IDLE) {
                    // Ready for new event - nothing to do until stick moves
                }
                
                // Update event detection
                bool eventComplete = autotuneEventUpdate(currentTimeUs, runtime.currentAxisIndex, 
                                                          stickDeflection, crossAxis, throttle);
                
                // Get updated state after the update
                eventState = autotuneEventGetState();
                
                if (eventState == EVENT_STATE_DEFLECTING && !autotuneEventBufferGet()->capturing) {
                    // Event just started - begin capturing samples
                    autotuneEventBufferStart();
                }
                
                if (eventState == EVENT_STATE_DEFLECTING || eventState == EVENT_STATE_RETURNING) {
                    // Event in progress - capture gyro/setpoint samples
                    const float gyroValue = gyro.gyroADCf[runtime.currentAxisIndex];
                    const float setpointValue = getSetpointRate(runtime.currentAxisIndex);
                    autotuneEventBufferAddSample(gyroValue, setpointValue);
                }
                
                if (eventComplete) {
                    // Event complete - stop capturing and transition to analyzing
                    autotuneEventBufferStop();
                    axis->substate = AXIS_SUBSTATE_ANALYZING;
                }
            }
            break;
            
        case AXIS_SUBSTATE_ANALYZING:
            {
                const eventData_t *eventData = autotuneEventGetData();
                
                if (!eventData->qualityGatesPassed) {
                    // Event rejected by quality gates - log reason and wait for next
                    // Determine which gate failed for debug output
                    if (!autotuneEventCheckDeflection(eventData->stickDeflection)) {
                        runtime.reasonCode = AUTOTUNE_REASON_INSUFFICIENT_DEFLECTION;
                    } else if (!autotuneEventCheckCrossAxis(eventData->crossAxisMovement)) {
                        runtime.reasonCode = AUTOTUNE_REASON_CROSS_AXIS_CONTAMINATION;
                    } else if (!autotuneEventCheckThrottle(eventData->throttle)) {
                        runtime.reasonCode = AUTOTUNE_REASON_THROTTLE_OUT_OF_BAND;
                    } else {
                        runtime.reasonCode = AUTOTUNE_REASON_ABNORMAL_DURATION;
                    }
                    
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_REASON, runtime.reasonCode);
                    
                    autotuneEventReset();
                    axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
                    break;
                }
                
                // Compute metrics from buffer
                const eventBuffer_t *buffer = autotuneEventBufferGet();
                eventMetrics_t metrics;
                
                // CRIT-008 fix: Pass actual loop time instead of hardcoded value
                if (autotuneMetricsAnalyze(buffer, &metrics, gyro.targetLooptime)) {
                    // Store metrics for later use
                    axis->lastMetrics = metrics;
                    axis->eventCount++;
                    runtime.totalEventsProcessed++;
                    runtime.reasonCode = AUTOTUNE_REASON_EVENT_DETECTED;
                    runtime.lastEventTimeUs = currentTimeUs;
                    
                    // Debug output - overshoot scaled by 10 for integer display
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_OVERSHOOT, (int16_t)(metrics.overshootPct * 10));
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_REASON, runtime.reasonCode);
                    
                    // Make tuning decision based on metrics
                    float newP = 0.0f;
                    autotuneDecision_e decision = pdRatioSeekDecision(&metrics, axis, runtime.targetOvershootPct, &newP);
                    
                    if (decision == AUTOTUNE_DECISION_INCREASE || 
                        decision == AUTOTUNE_DECISION_DECREASE ||
                        decision == AUTOTUNE_DECISION_ROLLBACK) {
                        // Apply new P gain
                        applyPGain(runtime.currentAxisIndex, newP);
                        axis->pState.currentValue = newP;
                        
                        // Save rollback state
                        autotuneRollbackSaveState(&axis->pState);
                        
                        // Log the new P value (scaled for debug display)
                        AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_P, (int16_t)axis->pState.currentValue);
                    } else if (decision == AUTOTUNE_DECISION_ADVANCE) {
                        // Target reached - transition to PD_SCALE_UP
                        runtime.reasonCode = AUTOTUNE_REASON_TARGET_REACHED;
                        transitionToState(AUTOTUNE_STATE_PD_SCALE_UP, currentTimeUs);
                        return;
                    }
                    // AUTOTUNE_DECISION_HOLD means continue collecting events
                    
                    // Check if we've hit event limit for this axis
                    if (axis->eventCount >= AUTOTUNE_MAX_EVENTS_PER_AXIS) {
                        runtime.reasonCode = AUTOTUNE_REASON_EVENT_LIMIT;
                        // Exceeded event limit - advance anyway
                        transitionToState(AUTOTUNE_STATE_PD_SCALE_UP, currentTimeUs);
                        return;
                    }
                } else {
                    // Metrics computation failed
                    runtime.reasonCode = AUTOTUNE_REASON_INVALID_METRICS;
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_REASON, runtime.reasonCode);
                }
                
                // Reset event detector and go back to waiting
                autotuneEventReset();
                axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
            }
            break;
            
        case AXIS_SUBSTATE_APPLYING:
        case AXIS_SUBSTATE_CONFIRMING:
            // TODO: Phase 3 implementation
            // These substates will be used when gain adjustments are made
            break;
            
        default:
            axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
            break;
    }
    
    // Update debug state
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_STATE, runtime.masterState);
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_AXIS, runtime.currentAxisIndex);
}

// ============================================================================
// State Handlers - PD_SCALE_UP
// ============================================================================

// Helper to apply scale factor to both P and D gains
static void applyScale(axisTuneState_t *axis, float scale)
{
    // OBS-003 fix: Capture old values for logging
    uint8_t oldP = currentPidProfile->pid[runtime.currentAxisIndex].P;
    uint8_t oldD = currentPidProfile->pid[runtime.currentAxisIndex].D;
    
    // Apply scale to both P and D
    float newP = axis->ratioSeekP * scale;
    float newD = axis->ratioSeekD * scale;
    
    // Clamp to valid range
    newP = constrainf(newP, AUTOTUNE_GAIN_MIN, AUTOTUNE_GAIN_MAX);
    newD = constrainf(newD, AUTOTUNE_GAIN_MIN, AUTOTUNE_GAIN_MAX);
    
    // Apply to PID profile
    currentPidProfile->pid[runtime.currentAxisIndex].P = lrintf(newP);
    currentPidProfile->pid[runtime.currentAxisIndex].D = lrintf(newD);
    
    // Update state tracking
    axis->pState.currentValue = newP;
    axis->dState.currentValue = newD;
    axis->currentScale = scale;
    
    // Debug output with delta info
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_P, (int16_t)newP);
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_D, (int16_t)newD);
    // Pack deltas: P delta in high byte, D delta in low byte
    int8_t deltaP = (int8_t)(lrintf(newP) - oldP);
    int8_t deltaD = (int8_t)(lrintf(newD) - oldD);
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_DECISION, (int16_t)((deltaP << 8) | (deltaD & 0xFF)));
}

// Newton-primary decision function for PD scale up
static autotuneDecision_e pdScaleUpDecision(
    const eventMetrics_t *metrics,
    axisTuneState_t *axisState,
    float lagTargetMs,
    float *newScale)
{
    float currentScale = axisState->currentScale;
    float lag = metrics->lagMs;
    scaleHistory_t *history = &axisState->scaleHistory;
    
    // === SAFETY FIRST: Rebound means instability ===
    if (metrics->hasRebound) {
        autotuneRollbackReportBad(&axisState->pState);
        autotuneRollbackReportBad(&axisState->dState);
        trackBadEvent(axisState);
        
        // Check for repeated failures
        if (shouldForceAdvanceOnBadEvents(axisState)) {
            *newScale = currentScale;
            return AUTOTUNE_DECISION_ADVANCE;
        }
        
        // Rollback to previous scale or 1.0
        if (history->count > 1) {
            *newScale = history->scale[history->count - 2];
        } else {
            *newScale = 1.0f;  // Back to ratio seek values
        }
        return AUTOTUNE_DECISION_ROLLBACK;
    }
    
    // === RECORD HISTORY ===
    if (history->count < MAX_SCALE_HISTORY) {
        history->scale[history->count] = currentScale;
        history->lag[history->count] = lag;
        history->count++;
    } else {
        // Shift history
        for (int i = 0; i < MAX_SCALE_HISTORY - 1; i++) {
            history->scale[i] = history->scale[i + 1];
            history->lag[i] = history->lag[i + 1];
        }
        history->scale[MAX_SCALE_HISTORY - 1] = currentScale;
        history->lag[MAX_SCALE_HISTORY - 1] = lag;
    }
    
    // === OVERSHOOT CHECK ===
    if (metrics->overshootPct > runtime.targetOvershootPct * 1.5f) {
        autotuneRollbackReportBad(&axisState->pState);
        trackBadEvent(axisState);
        
        // Check for repeated failures
        if (shouldForceAdvanceOnBadEvents(axisState)) {
            *newScale = currentScale;
            return AUTOTUNE_DECISION_ADVANCE;
        }
        
        // Rollback one step
        if (history->count > 1) {
            *newScale = history->scale[history->count - 2];
        } else {
            *newScale = 1.0f;
        }
        return AUTOTUNE_DECISION_ROLLBACK;
    }
    
    autotuneRollbackReportGood(&axisState->pState);
    autotuneRollbackReportGood(&axisState->dState);
    trackGoodEvent(axisState);
    
    // === CHECK CONVERGENCE ===
    if (lag <= lagTargetMs) {
        *newScale = currentScale;
        return AUTOTUNE_DECISION_ADVANCE;
    }
    
    // === AT GAIN LIMIT ===
    if (axisState->pState.currentValue >= AUTOTUNE_GAIN_MAX) {
        runtime.reasonCode = AUTOTUNE_REASON_GAIN_LIMIT_MAX;
        *newScale = currentScale;
        return AUTOTUNE_DECISION_ADVANCE;
    }
    
    // === NEWTON ESTIMATION ===
    // Lag varies smoothly with scale, so Newton works well here
    if (history->count >= 2) {
        int n = history->count;
        float prevScale = history->scale[n - 2];
        float prevLag = history->lag[n - 2];
        
        // Estimate derivative: d(lag)/d(scale)
        float dScale = currentScale - prevScale;
        float dLag = lag - prevLag;
        
        if (fabsf(dScale) > 0.01f && fabsf(dLag) > 0.5f) {
            float slope = dLag / dScale;  // Typically negative (more scale = less lag)
            
            // Newton: scale_new = scale_current - (lag - target) / slope
            float scaleNewton = currentScale - (lag - lagTargetMs) / slope;
            
            // Clamp to reasonable bounds
            scaleNewton = fmaxf(scaleNewton, currentScale * 0.8f);   // Don't go too low
            scaleNewton = fminf(scaleNewton, currentScale * 1.3f);  // Don't jump too far
            scaleNewton = fmaxf(scaleNewton, 1.0f);  // Never below starting scale
            
            *newScale = scaleNewton;
            return AUTOTUNE_DECISION_INCREASE;
        }
    }
    
    // === FALLBACK: Fixed step ===
    float step = autotuneRollbackGetStepSize(&axisState->pState, 1.0f);
    *newScale = currentScale * (1.0f + step);
    return AUTOTUNE_DECISION_INCREASE;
}

static void statePdScaleUpEnter(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    runtime.reasonCode = AUTOTUNE_REASON_NONE;
    
    axisTuneState_t *axis = &runtime.axisState[runtime.currentAxisIndex];
    
    // Capture current P/D values as baseline for scaling
    axis->ratioSeekP = axis->pState.currentValue;
    axis->ratioSeekD = axis->dState.currentValue;
    
    // Calculate and store the P/D ratio to maintain during scaling
    if (axis->ratioSeekD > 0.0f) {
        axis->pdRatio = axis->ratioSeekP / axis->ratioSeekD;
    } else {
        axis->pdRatio = 1.0f;  // Fallback if D is zero
    }
    
    // Initialize scale to 1.0 (current values)
    axis->currentScale = 1.0f;
    
    // Clear scale history for Newton estimation
    axis->scaleHistory.count = 0;
    
    // Initialize D state for rollback tracking
    autotuneRollbackInitParam(&axis->dState, axis->ratioSeekD, 
                              autotuneConfig()->dStepPercent / 100.0f);
    
    // Sync D trust score with P (they succeeded together in ratio seek)
    axis->dState.trustScore = axis->pState.trustScore;
    
    // CRIT-009 fix: Set lag target based on aggressiveness (0.0-1.0 -> 25-10ms)
    // PRD specifies t50 target of 10-25ms, not 25-100ms
    runtime.lagTargetMs = 25.0f - (runtime.aggressiveness * 15.0f);
    
    axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
    axis->eventCount = 0;
    
    // Debug output
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_P, (int16_t)axis->ratioSeekP);
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_D, (int16_t)axis->ratioSeekD);
}

static void statePdScaleUpUpdate(timeUs_t currentTimeUs)
{
    axisTuneState_t *axis = &runtime.axisState[runtime.currentAxisIndex];
    
    // Get current stick position for the active axis
    const float stickDeflection = rcCommand[runtime.currentAxisIndex] / 500.0f;
    
    // Cross-axis is max of other two axes
    float crossAxis = 0.0f;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        if (i != runtime.currentAxisIndex) {
            crossAxis = fmaxf(crossAxis, fabsf(rcCommand[i] / 500.0f));
        }
    }
    
    // Throttle normalized 0.0 to 1.0
    const float throttle = rcCommand[THROTTLE] / 1000.0f;
    
    switch (axis->substate) {
        case AXIS_SUBSTATE_WAIT_EVENT:
            {
                eventState_e eventState = autotuneEventGetState();
                
                bool eventComplete = autotuneEventUpdate(currentTimeUs, runtime.currentAxisIndex,
                                                          stickDeflection, crossAxis, throttle);
                
                eventState = autotuneEventGetState();
                
                if (eventState == EVENT_STATE_DEFLECTING && !autotuneEventBufferGet()->capturing) {
                    autotuneEventBufferStart();
                }
                
                if (eventState == EVENT_STATE_DEFLECTING || eventState == EVENT_STATE_RETURNING) {
                    const float gyroValue = gyro.gyroADCf[runtime.currentAxisIndex];
                    const float setpointValue = getSetpointRate(runtime.currentAxisIndex);
                    autotuneEventBufferAddSample(gyroValue, setpointValue);
                }
                
                if (eventComplete) {
                    autotuneEventBufferStop();
                    axis->substate = AXIS_SUBSTATE_ANALYZING;
                }
            }
            break;
            
        case AXIS_SUBSTATE_ANALYZING:
            {
                const eventData_t *eventData = autotuneEventGetData();
                
                if (!eventData->qualityGatesPassed) {
                    // Event rejected - determine reason and wait for next
                    if (!autotuneEventCheckDeflection(eventData->stickDeflection)) {
                        runtime.reasonCode = AUTOTUNE_REASON_INSUFFICIENT_DEFLECTION;
                    } else if (!autotuneEventCheckCrossAxis(eventData->crossAxisMovement)) {
                        runtime.reasonCode = AUTOTUNE_REASON_CROSS_AXIS_CONTAMINATION;
                    } else if (!autotuneEventCheckThrottle(eventData->throttle)) {
                        runtime.reasonCode = AUTOTUNE_REASON_THROTTLE_OUT_OF_BAND;
                    } else {
                        runtime.reasonCode = AUTOTUNE_REASON_ABNORMAL_DURATION;
                    }
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_REASON, runtime.reasonCode);
                    autotuneEventReset();
                    axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
                    break;
                }
                
                // Compute metrics from buffer
                const eventBuffer_t *buffer = autotuneEventBufferGet();
                eventMetrics_t metrics;
                
                // CRIT-008 fix: Pass actual loop time instead of hardcoded value
                if (autotuneMetricsAnalyze(buffer, &metrics, gyro.targetLooptime)) {
                    axis->lastMetrics = metrics;
                    axis->eventCount++;
                    runtime.totalEventsProcessed++;
                    runtime.reasonCode = AUTOTUNE_REASON_EVENT_DETECTED;
                    runtime.lastEventTimeUs = currentTimeUs;
                    
                    // OBS-005 fix: Log overshoot and lag metrics for observability
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_OVERSHOOT, (int16_t)(metrics.overshootPct * 10));
                    // Use GAIN_F channel for lag during PD_SCALE_UP (F not being tuned here)
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_F, (int16_t)(metrics.lagMs * 10));
                    
                    // Make scale up decision
                    float newScale = 0.0f;
                    autotuneDecision_e decision = pdScaleUpDecision(&metrics, axis, 
                                                                     runtime.lagTargetMs, &newScale);
                    
                    if (decision == AUTOTUNE_DECISION_INCREASE || 
                        decision == AUTOTUNE_DECISION_DECREASE) {
                        // Apply new scale
                        applyScale(axis, newScale);
                        autotuneRollbackSaveState(&axis->pState);
                        autotuneRollbackSaveState(&axis->dState);
                    } else if (decision == AUTOTUNE_DECISION_ADVANCE) {
                        // Scale up complete - go to F tune or next axis
                        runtime.reasonCode = AUTOTUNE_REASON_PHASE_COMPLETE;
                        if (autotuneConfig()->tuneFeedforward) {
                            transitionToState(AUTOTUNE_STATE_F_TUNE, currentTimeUs);
                        } else {
                            advanceToNextAxisOrComplete(currentTimeUs);
                        }
                        return;
                    } else if (decision == AUTOTUNE_DECISION_ROLLBACK) {
                        // Apply rollback scale
                        applyScale(axis, newScale);
                        runtime.reasonCode = AUTOTUNE_REASON_ROLLBACK_OSCILLATION;
                    }
                    
                    // Check event limit
                    if (axis->eventCount >= AUTOTUNE_MAX_EVENTS_PER_AXIS) {
                        runtime.reasonCode = AUTOTUNE_REASON_EVENT_LIMIT;
                        if (autotuneConfig()->tuneFeedforward) {
                            transitionToState(AUTOTUNE_STATE_F_TUNE, currentTimeUs);
                        } else {
                            advanceToNextAxisOrComplete(currentTimeUs);
                        }
                        return;
                    }
                } else {
                    runtime.reasonCode = AUTOTUNE_REASON_INVALID_METRICS;
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_REASON, runtime.reasonCode);
                }
                
                autotuneEventReset();
                axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
            }
            break;
            
        default:
            axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
            break;
    }
    
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_STATE, runtime.masterState);
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_AXIS, runtime.currentAxisIndex);
}

// ============================================================================
// State Handlers - F_TUNE
// ============================================================================

#define F_MAX 200
#define F_MAX_STEP_PCT 0.15f  // Max 15% step per event

// Helper to apply F gain value
static void applyF(uint8_t axis, float newF)
{
    // OBS-003 fix: Capture old value for logging
    uint16_t oldF = currentPidProfile->pid[axis].F;
    
    uint16_t fValue = constrain(lrintf(newF), 0, F_MAX);
    currentPidProfile->pid[axis].F = fValue;
    
    // Log F gain
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_F, fValue);
    // Log delta in decision channel
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_DECISION, (int16_t)(fValue - oldF));
}

// Cautious Newton decision function for F tuning
static autotuneDecision_e fTuneDecision(
    const eventMetrics_t *metrics,
    axisTuneState_t *axisState,
    float lagTargetMs,
    float *newF)
{
    float currentF = axisState->fState.currentValue;
    float lag = metrics->lagMs;
    fHistory_t *history = &axisState->fHistory;
    
    // === SAFETY: Overshoot increased too much ===
    if (metrics->overshootPct > runtime.targetOvershootPct * 1.3f) {
        autotuneRollbackReportBad(&axisState->fState);
        trackBadEvent(axisState);
        axisState->pdRetuneNeeded = true;  // F affected P/D balance
        
        // Check for repeated failures
        if (shouldForceAdvanceOnBadEvents(axisState)) {
            *newF = currentF;
            return AUTOTUNE_DECISION_ADVANCE;
        }
        
        // Rollback to previous F
        if (history->count > 1) {
            *newF = history->fValues[history->count - 2];
        } else {
            *newF = axisState->originalF;
        }
        return AUTOTUNE_DECISION_ROLLBACK;
    }
    
    autotuneRollbackReportGood(&axisState->fState);
    trackGoodEvent(axisState);
    
    // === RECORD HISTORY ===
    if (history->count < MAX_F_HISTORY) {
        history->fValues[history->count] = currentF;
        history->lagValues[history->count] = lag;
        history->count++;
    } else {
        // Shift history
        for (int i = 0; i < MAX_F_HISTORY - 1; i++) {
            history->fValues[i] = history->fValues[i + 1];
            history->lagValues[i] = history->lagValues[i + 1];
        }
        history->fValues[MAX_F_HISTORY - 1] = currentF;
        history->lagValues[MAX_F_HISTORY - 1] = lag;
    }
    
    // === CHECK CONVERGENCE ===
    if (lag <= lagTargetMs) {
        *newF = currentF;
        return AUTOTUNE_DECISION_ADVANCE;
    }
    
    // === AT LIMIT ===
    if (currentF >= F_MAX) {
        runtime.reasonCode = AUTOTUNE_REASON_GAIN_LIMIT_MAX;
        *newF = currentF;
        return AUTOTUNE_DECISION_ADVANCE;
    }
    
    // === CAUTIOUS NEWTON ===
    // Only use Newton if we have 2+ samples AND trend is clear
    if (history->count >= 2) {
        int n = history->count;
        float prevF = history->fValues[n - 2];
        float prevLag = history->lagValues[n - 2];
        
        float dF = currentF - prevF;
        float dLag = lag - prevLag;
        
        // Need clear decreasing trend (more F = less lag)
        if (dF > 0.5f && dLag < -0.5f) {
            float slope = dLag / dF;  // Should be negative
            
            // Newton prediction
            float fNewton = currentF - (lag - lagTargetMs) / slope;
            
            // CAUTIOUS: Clamp step size to max allowed
            float maxStep = currentF * F_MAX_STEP_PCT;
            maxStep = fmaxf(maxStep, 10.0f);  // Minimum step of 10
            
            float stepSize = fNewton - currentF;
            if (stepSize > maxStep) {
                fNewton = currentF + maxStep;
            }
            
            // Ensure increase only (never decrease F during tune)
            fNewton = fmaxf(fNewton, currentF);
            fNewton = fminf(fNewton, (float)F_MAX);
            
            *newF = fNewton;
            return AUTOTUNE_DECISION_INCREASE;
        }
    }
    
    // === FALLBACK: Small fixed step ===
    float step = fmaxf(currentF * 0.1f, 10.0f);  // 10% or min 10
    *newF = fminf(currentF + step, (float)F_MAX);
    return AUTOTUNE_DECISION_INCREASE;
}

static void stateFTuneEnter(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    runtime.reasonCode = AUTOTUNE_REASON_NONE;
    
    axisTuneState_t *axis = &runtime.axisState[runtime.currentAxisIndex];
    
    // Save pre-F-tune baseline metrics for comparison
    axis->preFTuneOvershoot = axis->lastMetrics.overshootPct;
    axis->preFTuneLag = axis->lastMetrics.lagMs;
    
    // Get current F value from profile
    float currentF = currentPidProfile->pid[runtime.currentAxisIndex].F;
    
    // Initialize F state for rollback tracking
    autotuneRollbackInitParam(&axis->fState, currentF, 
                              autotuneConfig()->fStepPercent / 100.0f);
    
    // Clear F history for Newton estimation
    axis->fHistory.count = 0;
    
    // Reset event counter for this phase
    axis->eventCount = 0;
    axis->pdRetuneNeeded = false;
    
    axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
    
    // Debug output
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_OVERSHOOT, (int16_t)(axis->preFTuneOvershoot * 10));
}

static void stateFTuneUpdate(timeUs_t currentTimeUs)
{
    axisTuneState_t *axis = &runtime.axisState[runtime.currentAxisIndex];
    
    // Get current stick position
    const float stickDeflection = rcCommand[runtime.currentAxisIndex] / 500.0f;
    
    // Cross-axis is max of other two axes
    float crossAxis = 0.0f;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        if (i != runtime.currentAxisIndex) {
            crossAxis = fmaxf(crossAxis, fabsf(rcCommand[i] / 500.0f));
        }
    }
    
    const float throttle = rcCommand[THROTTLE] / 1000.0f;
    
    switch (axis->substate) {
        case AXIS_SUBSTATE_WAIT_EVENT:
            {
                eventState_e eventState = autotuneEventGetState();
                
                bool eventComplete = autotuneEventUpdate(currentTimeUs, runtime.currentAxisIndex,
                                                          stickDeflection, crossAxis, throttle);
                
                eventState = autotuneEventGetState();
                
                if (eventState == EVENT_STATE_DEFLECTING && !autotuneEventBufferGet()->capturing) {
                    autotuneEventBufferStart();
                }
                
                if (eventState == EVENT_STATE_DEFLECTING || eventState == EVENT_STATE_RETURNING) {
                    const float gyroValue = gyro.gyroADCf[runtime.currentAxisIndex];
                    const float setpointValue = getSetpointRate(runtime.currentAxisIndex);
                    autotuneEventBufferAddSample(gyroValue, setpointValue);
                }
                
                if (eventComplete) {
                    autotuneEventBufferStop();
                    axis->substate = AXIS_SUBSTATE_ANALYZING;
                }
            }
            break;
            
        case AXIS_SUBSTATE_ANALYZING:
            {
                const eventData_t *eventData = autotuneEventGetData();
                
                if (!eventData->qualityGatesPassed) {
                    if (!autotuneEventCheckDeflection(eventData->stickDeflection)) {
                        runtime.reasonCode = AUTOTUNE_REASON_INSUFFICIENT_DEFLECTION;
                    } else if (!autotuneEventCheckCrossAxis(eventData->crossAxisMovement)) {
                        runtime.reasonCode = AUTOTUNE_REASON_CROSS_AXIS_CONTAMINATION;
                    } else if (!autotuneEventCheckThrottle(eventData->throttle)) {
                        runtime.reasonCode = AUTOTUNE_REASON_THROTTLE_OUT_OF_BAND;
                    } else {
                        runtime.reasonCode = AUTOTUNE_REASON_ABNORMAL_DURATION;
                    }
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_REASON, runtime.reasonCode);
                    autotuneEventReset();
                    axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
                    break;
                }
                
                const eventBuffer_t *buffer = autotuneEventBufferGet();
                eventMetrics_t metrics;
                
                // CRIT-008 fix: Pass actual loop time instead of hardcoded value
                if (autotuneMetricsAnalyze(buffer, &metrics, gyro.targetLooptime)) {
                    axis->lastMetrics = metrics;
                    axis->eventCount++;
                    runtime.totalEventsProcessed++;
                    runtime.reasonCode = AUTOTUNE_REASON_EVENT_DETECTED;
                    runtime.lastEventTimeUs = currentTimeUs;
                    
                    // OBS-005 fix: Log both overshoot and lag for F-tune observability
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_OVERSHOOT, (int16_t)(metrics.overshootPct * 10));
                    // Pack lag into decision channel since F is logged separately
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_DECISION, (int16_t)(metrics.lagMs * 10));
                    
                    // Make F tune decision
                    float newF = 0.0f;
                    autotuneDecision_e decision = fTuneDecision(&metrics, axis, 
                                                                 runtime.lagTargetMs, &newF);
                    
                    if (decision == AUTOTUNE_DECISION_INCREASE) {
                        // Apply new F value
                        applyF(runtime.currentAxisIndex, newF);
                        axis->fState.currentValue = newF;
                        autotuneRollbackSaveState(&axis->fState);
                    } else if (decision == AUTOTUNE_DECISION_ADVANCE) {
                        // F tune complete - check if PD needs retune
                        runtime.reasonCode = AUTOTUNE_REASON_PHASE_COMPLETE;
                        if (axis->pdRetuneNeeded) {
                            transitionToState(AUTOTUNE_STATE_PD_RETUNE_AFTER_F, currentTimeUs);
                        } else {
                            advanceToNextAxisOrComplete(currentTimeUs);
                        }
                        return;
                    } else if (decision == AUTOTUNE_DECISION_ROLLBACK) {
                        // Apply rollback F value
                        applyF(runtime.currentAxisIndex, newF);
                        axis->fState.currentValue = newF;
                        runtime.reasonCode = AUTOTUNE_REASON_ROLLBACK_OVERSHOOT;
                        // Continue with retune
                        transitionToState(AUTOTUNE_STATE_PD_RETUNE_AFTER_F, currentTimeUs);
                        return;
                    }
                    
                    // Check event limit
                    if (axis->eventCount >= AUTOTUNE_MAX_EVENTS_PER_AXIS) {
                        runtime.reasonCode = AUTOTUNE_REASON_EVENT_LIMIT;
                        if (axis->pdRetuneNeeded) {
                            transitionToState(AUTOTUNE_STATE_PD_RETUNE_AFTER_F, currentTimeUs);
                        } else {
                            advanceToNextAxisOrComplete(currentTimeUs);
                        }
                        return;
                    }
                } else {
                    runtime.reasonCode = AUTOTUNE_REASON_INVALID_METRICS;
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_REASON, runtime.reasonCode);
                }
                
                autotuneEventReset();
                axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
            }
            break;
            
        default:
            axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
            break;
    }
    
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_STATE, runtime.masterState);
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_AXIS, runtime.currentAxisIndex);
}

// ============================================================================
// State Handlers - PD_RETUNE_AFTER_F
// ============================================================================

static void statePdRetuneEnter(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    runtime.reasonCode = AUTOTUNE_REASON_NONE;
    
    axisTuneState_t *axis = &runtime.axisState[runtime.currentAxisIndex];
    
    // Compare current overshoot to pre-F-tune baseline
    float overshootDelta = axis->lastMetrics.overshootPct - axis->preFTuneOvershoot;
    
    if (overshootDelta > 3.0f) {
        // Overshoot increased significantly - need to adjust P/D
        axis->pdRetuneNeeded = true;
    } else {
        // Damping acceptable - no adjustment needed
        axis->pdRetuneNeeded = false;
    }
    
    // Reset event counter for this quick validation
    axis->eventCount = 0;
    axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
}

static void statePdRetuneUpdate(timeUs_t currentTimeUs)
{
    axisTuneState_t *axis = &runtime.axisState[runtime.currentAxisIndex];
    
    // Quick validation: max 3 events
    const uint8_t MAX_RETUNE_EVENTS = 3;
    
    // Get current stick position
    const float stickDeflection = rcCommand[runtime.currentAxisIndex] / 500.0f;
    
    float crossAxis = 0.0f;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        if (i != runtime.currentAxisIndex) {
            crossAxis = fmaxf(crossAxis, fabsf(rcCommand[i] / 500.0f));
        }
    }
    
    const float throttle = rcCommand[THROTTLE] / 1000.0f;
    
    switch (axis->substate) {
        case AXIS_SUBSTATE_WAIT_EVENT:
            {
                eventState_e eventState = autotuneEventGetState();
                
                bool eventComplete = autotuneEventUpdate(currentTimeUs, runtime.currentAxisIndex,
                                                          stickDeflection, crossAxis, throttle);
                
                eventState = autotuneEventGetState();
                
                if (eventState == EVENT_STATE_DEFLECTING && !autotuneEventBufferGet()->capturing) {
                    autotuneEventBufferStart();
                }
                
                if (eventState == EVENT_STATE_DEFLECTING || eventState == EVENT_STATE_RETURNING) {
                    const float gyroValue = gyro.gyroADCf[runtime.currentAxisIndex];
                    const float setpointValue = getSetpointRate(runtime.currentAxisIndex);
                    autotuneEventBufferAddSample(gyroValue, setpointValue);
                }
                
                if (eventComplete) {
                    autotuneEventBufferStop();
                    axis->substate = AXIS_SUBSTATE_ANALYZING;
                }
            }
            break;
            
        case AXIS_SUBSTATE_ANALYZING:
            {
                const eventData_t *eventData = autotuneEventGetData();
                
                if (!eventData->qualityGatesPassed) {
                    runtime.reasonCode = AUTOTUNE_REASON_ABNORMAL_DURATION;
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_REASON, runtime.reasonCode);
                    autotuneEventReset();
                    axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
                    break;
                }
                
                const eventBuffer_t *buffer = autotuneEventBufferGet();
                eventMetrics_t metrics;
                
                // CRIT-008 fix: Pass actual loop time instead of hardcoded value
                if (autotuneMetricsAnalyze(buffer, &metrics, gyro.targetLooptime)) {
                    axis->lastMetrics = metrics;
                    axis->eventCount++;
                    runtime.totalEventsProcessed++;
                    
                    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_OVERSHOOT, (int16_t)(metrics.overshootPct * 10));
                    
                    // Quick validation logic
                    bool overshootOk = (metrics.overshootPct <= runtime.targetOvershootPct * 1.2f);
                    bool hasRebound = metrics.hasRebound;
                    
                    if (hasRebound) {
                        // Rebound detected - decrease P slightly
                        float newP = axis->pState.currentValue * 0.95f;
                        newP = fmaxf(newP, AUTOTUNE_GAIN_MIN);
                        applyPGain(runtime.currentAxisIndex, newP);
                        axis->pState.currentValue = newP;
                        
                        // Also decrease D proportionally
                        float newD = axis->dState.currentValue * 0.95f;
                        newD = fmaxf(newD, AUTOTUNE_GAIN_MIN);
                        currentPidProfile->pid[runtime.currentAxisIndex].D = lrintf(newD);
                        axis->dState.currentValue = newD;
                        
                        runtime.reasonCode = AUTOTUNE_REASON_ROLLBACK_OSCILLATION;
                    } else if (overshootOk || axis->eventCount >= MAX_RETUNE_EVENTS) {
                        // Overshoot acceptable or we've done enough - advance
                        runtime.reasonCode = AUTOTUNE_REASON_AXIS_COMPLETE;
                        advanceToNextAxisOrComplete(currentTimeUs);
                        return;
                    }
                } else {
                    runtime.reasonCode = AUTOTUNE_REASON_INVALID_METRICS;
                }
                
                // Check event limit
                if (axis->eventCount >= MAX_RETUNE_EVENTS) {
                    runtime.reasonCode = AUTOTUNE_REASON_EVENT_LIMIT;
                    advanceToNextAxisOrComplete(currentTimeUs);
                    return;
                }
                
                autotuneEventReset();
                axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
            }
            break;
            
        default:
            axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
            break;
    }
    
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_STATE, runtime.masterState);
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_AXIS, runtime.currentAxisIndex);
}

// ============================================================================
// Axis Loop
// ============================================================================

static void advanceToNextAxisOrComplete(timeUs_t currentTimeUs)
{
    // Mark current axis complete
    runtime.axisState[runtime.currentAxisIndex].complete = true;
    
    // Feedback: axis complete
    autotuneFeedbackAxisComplete(runtime.currentAxisIndex);
    
    // Find next enabled axis
    for (int i = runtime.currentAxisIndex + 1; i < AUTOTUNE_AXIS_COUNT; i++) {
        if (autotuneConfig()->axes & (1 << i)) {
            runtime.currentAxisIndex = i;
            
            // Initialize the new axis
            axisTuneState_t *axis = &runtime.axisState[i];
            axis->axis = i;
            axis->complete = false;
            axis->eventCount = 0;
            
            // Save original gains for this axis
            axis->originalP = currentPidProfile->pid[i].P;
            axis->originalD = currentPidProfile->pid[i].D;
            axis->originalF = currentPidProfile->pid[i].F;
            
            // Start at PD_RATIO_SEEK for the new axis
            transitionToState(AUTOTUNE_STATE_PD_RATIO_SEEK, currentTimeUs);
            return;
        }
    }
    
    // All axes complete!
    transitionToState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
    autotuneFeedbackComplete();  // All done
}

// ============================================================================
// Timeout Checking
// ============================================================================

static void checkTimeouts(timeUs_t currentTimeUs)
{
    // Check for total timeout
    if (runtime.startTimeUs > 0) {
        const timeDelta_t totalElapsed = cmpTimeUs(currentTimeUs, runtime.startTimeUs);
        if (totalElapsed > AUTOTUNE_TOTAL_TIMEOUT_US) {
            runtime.reasonCode = AUTOTUNE_REASON_ABORT_TIMEOUT;
            transitionToState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
            return;
        }
    }
    
    // Check for per-state timeout
    const timeDelta_t stateElapsed = cmpTimeUs(currentTimeUs, runtime.stateEntryTimeUs);
    
    switch (runtime.masterState) {
        case AUTOTUNE_STATE_HOVER_LOCK:
            // Hover lock already has its own timeout handling
            break;
            
        case AUTOTUNE_STATE_PD_RATIO_SEEK:
        case AUTOTUNE_STATE_PD_SCALE_UP:
        case AUTOTUNE_STATE_F_TUNE:
        case AUTOTUNE_STATE_PD_RETUNE_AFTER_F:
            if (stateElapsed > AUTOTUNE_AXIS_TIMEOUT_US) {
                runtime.reasonCode = AUTOTUNE_REASON_ABORT_TIMEOUT;
                advanceToNextAxisOrComplete(currentTimeUs);
            }
            break;
            
        default:
            break;
    }
}

// ============================================================================
// State Handlers - COMPLETE
// ============================================================================

static void stateCompleteEnter(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    runtime.reasonCode = AUTOTUNE_REASON_PHASE_COMPLETE;
    
    // Trigger completion feedback
    autotuneFeedbackComplete();
}

static void stateCompleteUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    // Stay in complete until deactivated
    // Transition to IDLE handled by autotuneUpdateActivation()
}

// ============================================================================
// Axis Management
// ============================================================================

// This function will be used in Phase 3+ when axis tuning is implemented
LOCAL_UNUSED_FUNCTION static bool advanceToNextAxis(timeUs_t currentTimeUs)
{
    const uint8_t axisMask = autotuneConfig()->axes;
    
    // Mark current axis complete
    runtime.axisState[runtime.currentAxisIndex].complete = true;
    runtime.reasonCode = AUTOTUNE_REASON_AXIS_COMPLETE;
    
    // Trigger axis complete feedback
    autotuneFeedbackAxisComplete(runtime.currentAxisIndex);
    
    // Find next enabled axis
    for (uint8_t i = runtime.currentAxisIndex + 1; i < AUTOTUNE_AXIS_COUNT; i++) {
        if (axisMask & (1 << i)) {
            runtime.currentAxisIndex = i;
            AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_AXIS, i);
            return true;
        }
    }
    
    // No more axes - complete
    transitionToState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
    return false;
}

static void initializeAxisState(uint8_t axisIndex)
{
    axisTuneState_t *axis = &runtime.axisState[axisIndex];
    
    axis->axis = axisIndex;
    axis->substate = AXIS_SUBSTATE_WAIT_EVENT;
    axis->eventCount = 0;
    axis->complete = false;
    
    // Initialize parameter states with trust system defaults
    axis->pState.trustScore = AUTOTUNE_TRUST_INITIAL;
    axis->pState.baseStepSize = AUTOTUNE_P_STEP_BASE;
    axis->pState.consecutiveGood = 0;
    axis->pState.consecutiveBad = 0;
    
    axis->dState.trustScore = AUTOTUNE_TRUST_INITIAL;
    axis->dState.baseStepSize = AUTOTUNE_D_STEP_BASE;
    axis->dState.consecutiveGood = 0;
    axis->dState.consecutiveBad = 0;
    
    axis->fState.trustScore = AUTOTUNE_TRUST_INITIAL;
    axis->fState.baseStepSize = AUTOTUNE_F_STEP_BASE;
    axis->fState.consecutiveGood = 0;
    axis->fState.consecutiveBad = 0;
    
    // Original gains will be captured when tuning begins
    axis->originalP = 0;
    axis->originalD = 0;
    axis->originalF = 0;
    
    // Clear metrics
    memset(&axis->lastMetrics, 0, sizeof(eventMetrics_t));
}

// ============================================================================
// Public API Implementation
// ============================================================================

void autotuneInit(void)
{
    memset(&runtime, 0, sizeof(autotuneRuntime_t));
    
    runtime.masterState = AUTOTUNE_STATE_IDLE;
    runtime.aggressiveness = autotuneConfig()->aggressiveness / 100.0f;
    runtime.targetOvershootPct = AUTOTUNE_OVERSHOOT_TARGET_DEFAULT;
    runtime.lagTargetMs = 10.0f; // Default lag target
    
    // Initialize all axis states
    for (uint8_t i = 0; i < AUTOTUNE_AXIS_COUNT; i++) {
        initializeAxisState(i);
    }
    
    // Initialize sub-modules
    autotuneEventInit();
    autotuneFilterInit();
    autotuneMetricsInit();
    autotuneRollbackInit();
    autotuneFeedbackInit();
}

void autotuneUpdate(timeUs_t currentTimeUs)
{
    // Update sub-modules
    autotuneFeedbackUpdate(currentTimeUs);
    
    // Check for timeouts when active
    if (runtime.masterState != AUTOTUNE_STATE_IDLE && 
        runtime.masterState != AUTOTUNE_STATE_COMPLETE) {
        checkTimeouts(currentTimeUs);
    }
    
    // Run state machine
    switch (runtime.masterState) {
        case AUTOTUNE_STATE_IDLE:
            stateIdleUpdate(currentTimeUs);
            break;
        case AUTOTUNE_STATE_HOVER_LOCK:
            stateHoverLockUpdate(currentTimeUs);
            break;
        case AUTOTUNE_STATE_THROTTLE_SWEEP:
            stateThrottleSweepUpdate(currentTimeUs);
            break;
        case AUTOTUNE_STATE_NOISE_CONFIRM:
            stateNoiseConfirmUpdate(currentTimeUs);
            break;
        case AUTOTUNE_STATE_PD_RATIO_SEEK:
            statePdRatioSeekUpdate(currentTimeUs);
            break;
        case AUTOTUNE_STATE_PD_SCALE_UP:
            statePdScaleUpUpdate(currentTimeUs);
            break;
        case AUTOTUNE_STATE_F_TUNE:
            stateFTuneUpdate(currentTimeUs);
            break;
        case AUTOTUNE_STATE_PD_RETUNE_AFTER_F:
            statePdRetuneUpdate(currentTimeUs);
            break;
        case AUTOTUNE_STATE_COMPLETE:
            stateCompleteUpdate(currentTimeUs);
            break;
        default:
            break;
    }
}

bool autotuneIsActive(void)
{
    return runtime.masterState != AUTOTUNE_STATE_IDLE &&
           runtime.masterState != AUTOTUNE_STATE_COMPLETE;
}

autotuneState_e autotuneGetState(void)
{
    return runtime.masterState;
}

uint8_t autotuneGetCurrentAxis(void)
{
    return runtime.currentAxisIndex;
}

uint8_t autotuneGetProgress(void)
{
    if (runtime.masterState == AUTOTUNE_STATE_IDLE) {
        return 0;
    }
    if (runtime.masterState == AUTOTUNE_STATE_COMPLETE) {
        return 100;
    }
    
    // Rough progress estimate based on state and axis
    const uint8_t stateProgress[] = {
        0,   // IDLE
        5,   // HOVER_LOCK
        15,  // THROTTLE_SWEEP
        20,  // NOISE_CONFIRM
        40,  // PD_RATIO_SEEK
        60,  // PD_SCALE_UP
        80,  // F_TUNE
        90,  // PD_RETUNE_AFTER_F
        100  // COMPLETE
    };
    
    uint8_t baseProgress = stateProgress[runtime.masterState];
    
    // Add axis contribution for tuning states
    if (runtime.masterState >= AUTOTUNE_STATE_PD_RATIO_SEEK &&
        runtime.masterState <= AUTOTUNE_STATE_PD_RETUNE_AFTER_F) {
        baseProgress += runtime.currentAxisIndex * 10;
    }
    
    return baseProgress > 100 ? 100 : baseProgress;
}

uint16_t autotuneGetReasonCode(void)
{
    return runtime.reasonCode;
}

autotuneDecision_e autotuneGetLastDecision(void)
{
    return runtime.lastDecision;
}

void autotuneAbort(uint16_t reasonCode)
{
    runtime.reasonCode = reasonCode;
    runtime.lastDecision = AUTOTUNE_DECISION_NONE;
    
    // OBS-002 fix: Restore original gains for all axes that were modified
    const uint8_t axisMask = autotuneConfig()->axes;
    for (uint8_t i = 0; i < AUTOTUNE_AXIS_COUNT; i++) {
        if (axisMask & (1 << i)) {
            axisTuneState_t *axis = &runtime.axisState[i];
            // Only restore if original values were captured (non-zero)
            if (axis->originalP > 0) {
                currentPidProfile->pid[i].P = (uint8_t)lrintf(axis->originalP);
            }
            if (axis->originalD > 0) {
                currentPidProfile->pid[i].D = (uint8_t)lrintf(axis->originalD);
            }
            if (axis->originalF > 0) {
                currentPidProfile->pid[i].F = (uint16_t)lrintf(axis->originalF);
            }
        }
    }
    
    // Log restored values for observability
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_REASON, reasonCode);
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_P, currentPidProfile->pid[FD_ROLL].P);
    AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_D, currentPidProfile->pid[FD_ROLL].D);
    
    transitionToState(AUTOTUNE_STATE_IDLE, micros());
}

void autotuneUpdateActivation(bool enabled, timeUs_t currentTimeUs)
{
    if (enabled && runtime.masterState == AUTOTUNE_STATE_IDLE) {
        // Start autotune
        runtime.startTimeUs = currentTimeUs;  // Record session start time
        runtime.totalEventsProcessed = 0;
        
        // Find first enabled axis
        const uint8_t axisMask = autotuneConfig()->axes;
        for (uint8_t i = 0; i < AUTOTUNE_AXIS_COUNT; i++) {
            if (axisMask & (1 << i)) {
                runtime.currentAxisIndex = i;
                break;
            }
        }
        
        AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_AXIS, runtime.currentAxisIndex);
        transitionToState(AUTOTUNE_STATE_HOVER_LOCK, currentTimeUs);
        
    } else if (!enabled && runtime.masterState != AUTOTUNE_STATE_IDLE) {
        // Stop autotune
        autotuneAbort(AUTOTUNE_REASON_ABORT_SWITCH);
    }
}

#endif // USE_AUTOTUNE_V2
