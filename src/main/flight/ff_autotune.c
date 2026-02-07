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

/*
 * FF Autotune v3: Setpoint Tracking-Based Feedforward Tuning
 * ==========================================================
 * 
 * This module automatically tunes feedforward gain by monitoring setpoint
 * tracking quality during maneuvers and using intelligent bracketing:
 * 
 * 1. During maneuvers (setpoint in tracking window with sufficient acceleration):
 *    - Accumulate tracking error (gyro - setpoint)
 *    
 * 2. At end of maneuver (setpoint exits tracking window):
 *    - Calculate average tracking error
 *    - Update history buffer with (gain, avgError) pair
 *    - Use history to establish/narrow bracket around optimal gain
 *    
 * 3. Bracketing logic:
 *    - SEARCHING: Step in direction indicated by error
 *    - BRACKETED: Binary search within bracket
 *    - CONVERGED: Stop adjusting
 *    
 * 4. Gains saved to EEPROM only when mode switch is turned off
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_FF_AUTOTUNE

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"

#include "config/config.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"

#include "pg/ff_autotune.h"

#include "sensors/gyro.h"

#include "ff_autotune.h"

// ============================================================================
// CONSTANTS
// ============================================================================

#define FF_AUTOTUNE_HISTORY_SIZE    8       // Ring buffer size per axis
#define FF_AUTOTUNE_MIN_SAMPLES     50      // Minimum samples for valid maneuver
#define FF_AUTOTUNE_ADJUST_DELAY_US 100000  // 100ms delay before adjusting F term

// ============================================================================
// HISTORY ENTRY
// ============================================================================

typedef struct {
    uint8_t  gain;              // Gain setting used
    int16_t  avgError;          // Average tracking error (×10 for precision)
    uint16_t sampleCount;       // Number of samples in measurement
    bool     valid;             // Entry is valid
} ffHistoryEntry_t;

// ============================================================================
// PER-AXIS STATE
// ============================================================================

typedef struct {
    // Current gain (runtime, may differ from config during tuning)
    uint8_t gain;
    
    // Window state machine
    ffWindowState_e windowState;
    
    // Maneuver tracking
    float errorAccumulator;         // Sum of tracking errors
    uint32_t sampleCount;           // Samples in current maneuver
    float prevSetpoint;             // For acceleration calc
    float prevAccel;                // Previous acceleration (for peak detection)
    
    // Post-maneuver processing
    timeUs_t maneuverEndTime;       // When maneuver ended
    int16_t lastAvgError;           // Last completed maneuver's avg error (×10)
    int8_t lastAssessment;          // Last assessment (-1=lag, 0=optimal, +1=lead)
    
    // History ring buffer with intelligent management
    ffHistoryEntry_t history[FF_AUTOTUNE_HISTORY_SIZE];
    uint8_t historyCount;           // Number of valid entries
    
    // Bracket state
    uint8_t lowerGain;              // Best gain that still lags (highest)
    uint8_t upperGain;              // Best gain that leads (lowest)
    int16_t lowerError;             // Error at lower bound
    int16_t upperError;             // Error at upper bound
    ffBracketState_e bracketState;  // Current bracket state
    
    // Statistics
    uint16_t maneuverCount;         // Total maneuvers analyzed
    uint16_t adjustmentCount;       // Number of gain adjustments
} ffAxisState_t;

// ============================================================================
// RUNTIME STATE
// ============================================================================

static struct {
    bool active;                    // Mode is currently active
    bool wasActive;                 // Was active last iteration
    bool gainsModified;             // Gains changed since last save
    ffAxisState_t axis[2];          // Roll and Pitch only
} runtime;

// ============================================================================
// INITIALIZATION
// ============================================================================

void ffAutotuneInit(void)
{
    memset(&runtime, 0, sizeof(runtime));
    
    // Load initial gains from config
    runtime.axis[FD_ROLL].gain = ffAutotuneConfig()->gain_roll;
    runtime.axis[FD_PITCH].gain = ffAutotuneConfig()->gain_pitch;
    
    // Initialize bracket bounds to extremes
    for (int i = 0; i < 2; i++) {
        runtime.axis[i].lowerGain = 0;
        runtime.axis[i].upperGain = 255;
        runtime.axis[i].bracketState = FF_BRACKET_SEARCHING;
    }
}

// ============================================================================
// MODE MANAGEMENT
// ============================================================================

bool ffAutotuneIsActive(void)
{
    return runtime.active;
}

uint8_t ffAutotuneGetGain(int axis)
{
    if (axis > FD_PITCH) {
        return 0;
    }
    return runtime.axis[axis].gain;
}

bool ffAutotuneNeedsSave(void)
{
    return runtime.gainsModified;
}

void ffAutotuneSaveGains(void)
{
    if (!runtime.gainsModified) {
        return;
    }
    
    ffAutotuneConfigMutable()->gain_roll = runtime.axis[FD_ROLL].gain;
    ffAutotuneConfigMutable()->gain_pitch = runtime.axis[FD_PITCH].gain;
    
    writeEEPROM();
    
    runtime.gainsModified = false;
    
    beeper(BEEPER_READY_BEEP);
}

void ffAutotuneReset(void)
{
    ffAutotuneInit();
    runtime.gainsModified = true;
}

// ============================================================================
// INTELLIGENT HISTORY MANAGEMENT
// ============================================================================

// Classify an error value
static int8_t classifyError(int16_t avgError, int16_t deadband)
{
    if (avgError < -deadband) return -1;      // Lag
    if (avgError > deadband) return 1;        // Lead
    return 0;                                  // Optimal
}

// Find index to evict - intelligent selection
static int findEvictionIndex(ffAxisState_t *state, int8_t newType, int16_t deadband)
{
    // Count samples by type
    int lagCount = 0, optimalCount = 0, leadCount = 0;
    int oldestLag = -1, oldestOptimal = -1, oldestLead = -1;
    
    for (int i = 0; i < FF_AUTOTUNE_HISTORY_SIZE; i++) {
        if (!state->history[i].valid) continue;
        
        int8_t type = classifyError(state->history[i].avgError, deadband);
        
        // Track oldest of each type (lower index = older, simple heuristic)
        if (type < 0) {
            lagCount++;
            if (oldestLag < 0) oldestLag = i;
        } else if (type > 0) {
            leadCount++;
            if (oldestLead < 0) oldestLead = i;
        } else {
            optimalCount++;
            if (oldestOptimal < 0) oldestOptimal = i;
        }
    }
    
    // Check if any candidate is a bracket bound (don't evict those!)
    bool isLowerBound[FF_AUTOTUNE_HISTORY_SIZE] = {false};
    bool isUpperBound[FF_AUTOTUNE_HISTORY_SIZE] = {false};
    
    for (int i = 0; i < FF_AUTOTUNE_HISTORY_SIZE; i++) {
        if (!state->history[i].valid) continue;
        if (state->history[i].gain == state->lowerGain && 
            state->history[i].avgError == state->lowerError) {
            isLowerBound[i] = true;
        }
        if (state->history[i].gain == state->upperGain && 
            state->history[i].avgError == state->upperError) {
            isUpperBound[i] = true;
        }
    }
    
    // Prefer to evict same type as new sample (preserve diversity)
    int candidate = -1;
    
    if (newType < 0 && lagCount > 1) {
        // New sample is lag, evict oldest lag (but not lower bound)
        for (int i = 0; i < FF_AUTOTUNE_HISTORY_SIZE; i++) {
            if (state->history[i].valid && 
                classifyError(state->history[i].avgError, deadband) < 0 &&
                !isLowerBound[i]) {
                candidate = i;
                break;
            }
        }
    } else if (newType > 0 && leadCount > 1) {
        // New sample is lead, evict oldest lead (but not upper bound)
        for (int i = 0; i < FF_AUTOTUNE_HISTORY_SIZE; i++) {
            if (state->history[i].valid && 
                classifyError(state->history[i].avgError, deadband) > 0 &&
                !isUpperBound[i]) {
                candidate = i;
                break;
            }
        }
    } else if (newType == 0 && optimalCount > 1) {
        // New sample is optimal, evict oldest optimal
        candidate = oldestOptimal;
    }
    
    // Fallback: evict oldest non-bracket-bound entry
    if (candidate < 0) {
        for (int i = 0; i < FF_AUTOTUNE_HISTORY_SIZE; i++) {
            if (state->history[i].valid && !isLowerBound[i] && !isUpperBound[i]) {
                candidate = i;
                break;
            }
        }
    }
    
    // Last resort: evict first valid entry (shouldn't happen if logic is right)
    if (candidate < 0) {
        for (int i = 0; i < FF_AUTOTUNE_HISTORY_SIZE; i++) {
            if (state->history[i].valid) {
                candidate = i;
                break;
            }
        }
    }
    
    return candidate;
}

// Add entry to history with intelligent eviction
static void addHistoryEntry(ffAxisState_t *state, uint8_t gain, int16_t avgError, uint16_t sampleCount)
{
    const int16_t deadband = ffAutotuneConfig()->error_deadband * 10;  // Scale to match avgError
    int8_t newType = classifyError(avgError, deadband);
    
    int writeIndex;
    
    if (state->historyCount < FF_AUTOTUNE_HISTORY_SIZE) {
        // Buffer not full, find first empty slot
        writeIndex = state->historyCount;
        for (int i = 0; i < FF_AUTOTUNE_HISTORY_SIZE; i++) {
            if (!state->history[i].valid) {
                writeIndex = i;
                break;
            }
        }
        state->historyCount++;
    } else {
        // Buffer full, use intelligent eviction
        writeIndex = findEvictionIndex(state, newType, deadband);
    }
    
    // Write the new entry
    state->history[writeIndex].gain = gain;
    state->history[writeIndex].avgError = avgError;
    state->history[writeIndex].sampleCount = sampleCount;
    state->history[writeIndex].valid = true;
    
    // Update bracket bounds if this improves them
    if (avgError < -deadband) {
        // Lag sample: want HIGHEST gain that still lags
        if (gain > state->lowerGain || state->lowerGain == 0) {
            state->lowerGain = gain;
            state->lowerError = avgError;
        }
    } else if (avgError > deadband) {
        // Lead sample: want LOWEST gain that leads
        if (gain < state->upperGain || state->upperGain == 255) {
            state->upperGain = gain;
            state->upperError = avgError;
        }
    }
    
    // Check if bracket is now established
    if (state->lowerGain > 0 && state->upperGain < 255 && 
        state->lowerGain < state->upperGain) {
        if (state->bracketState == FF_BRACKET_SEARCHING) {
            state->bracketState = FF_BRACKET_BRACKETED;
        }
        
        // Check for convergence
        uint8_t bracketWidth = state->upperGain - state->lowerGain;
        if (bracketWidth <= ffAutotuneConfig()->converge_threshold) {
            state->bracketState = FF_BRACKET_CONVERGED;
        }
    }
}

// ============================================================================
// GAIN ADJUSTMENT
// ============================================================================

static uint8_t calculateNextGain(ffAxisState_t *state, int16_t avgError)
{
    const uint8_t gainStep = ffAutotuneConfig()->gain_step;
    const uint8_t gainMin = ffAutotuneConfig()->gain_min;
    const uint8_t gainMax = ffAutotuneConfig()->gain_max;
    const int16_t deadband = ffAutotuneConfig()->error_deadband * 10;
    
    int16_t newGain = state->gain;
    
    switch (state->bracketState) {
        case FF_BRACKET_SEARCHING:
            // Linear search: step in direction indicated by error
            if (avgError < -deadband) {
                // Lag: need more FF
                newGain = state->gain + gainStep;
            } else if (avgError > deadband) {
                // Lead: need less FF
                newGain = state->gain - gainStep;
            }
            break;
            
        case FF_BRACKET_BRACKETED:
            // Binary search: test midpoint of bracket
            newGain = (state->lowerGain + state->upperGain) / 2;
            break;
            
        case FF_BRACKET_CONVERGED:
            // Already converged, use midpoint of final bracket
            newGain = (state->lowerGain + state->upperGain) / 2;
            break;
    }
    
    return constrain(newGain, gainMin, gainMax);
}

// ============================================================================
// MANEUVER PROCESSING
// ============================================================================

static void processManeuverEnd(ffAxisState_t *state, int axis)
{
    if (state->sampleCount < FF_AUTOTUNE_MIN_SAMPLES) {
        // Not enough samples, discard - but still go to WAITING
        state->errorAccumulator = 0.0f;
        state->sampleCount = 0;
        return;
    }
    
    // Calculate average error (×10 for precision in storage)
    const float avgErrorF = state->errorAccumulator / (float)state->sampleCount;
    const int16_t avgError = lrintf(avgErrorF * 10.0f);
    
    // Store results
    state->lastAvgError = avgError;
    state->lastAssessment = classifyError(avgError, ffAutotuneConfig()->error_deadband * 10);
    
    // Add to history
    addHistoryEntry(state, state->gain, avgError, state->sampleCount);
    
    // Calculate next gain (unless converged)
    if (state->bracketState != FF_BRACKET_CONVERGED) {
        uint8_t nextGain = calculateNextGain(state, avgError);
        
        if (nextGain != state->gain) {
            state->gain = nextGain;
            state->adjustmentCount++;
            runtime.gainsModified = true;
        }
    }
    
    state->maneuverCount++;
    
    // Reset accumulators for next maneuver (state transition handled by caller)
    state->errorAccumulator = 0.0f;
    state->sampleCount = 0;
    
    UNUSED(axis);
}

// ============================================================================
// TRACKING UPDATE
// ============================================================================

static void updateAxisTracking(int axis, float setpoint, float gyroRate, 
                                float setpointDelta, float pidFrequency, timeUs_t currentTimeUs)
{
    ffAxisState_t *state = &runtime.axis[axis];
    
    const float absSetpoint = fabsf(setpoint);
    const float setpointLow = ffAutotuneConfig()->setpoint_low;
    const float setpointHigh = ffAutotuneConfig()->setpoint_high;
    const float minAccel = ffAutotuneConfig()->min_accel * 100.0f;
    
    // Calculate setpoint acceleration (deg/s²)
    const float setpointAccel = setpointDelta * pidFrequency;
    const float absAccel = fabsf(setpointAccel);
    
    // Tracking error using magnitude comparison (works for both positive and negative maneuvers)
    // Positive = gyro magnitude ahead of setpoint magnitude (LEAD)
    // Negative = gyro magnitude behind setpoint magnitude (LAG)
    const float trackingError = fabsf(gyroRate) - fabsf(setpoint);
    
    // Determine if setpoint magnitude is increasing (stick moving away from center)
    const bool magnitudeIncreasing = (setpoint * setpointDelta) > 0;
    
    // Zone checks
    const bool isNeutral = (absSetpoint < setpointLow);
    const bool inSetpointWindow = (absSetpoint >= setpointLow) && (absSetpoint <= setpointHigh);
    const bool hasSignificantAccel = (absAccel >= minAccel);
    
    // Simple state machine:
    // IDLE -> RISING -> ADJUSTING -> WAITING -> IDLE
    switch (state->windowState) {
        case FF_WINDOW_IDLE:
            // IDLE: waiting for rise - neutral setpoint and low accel
            // Transition to RISING when setpoint enters window with increasing magnitude
            if (inSetpointWindow && magnitudeIncreasing && hasSignificantAccel) {
                state->windowState = FF_WINDOW_RISING;
                state->errorAccumulator = trackingError;
                state->sampleCount = 1;
            }
            break;
            
        case FF_WINDOW_RISING:
            // RISING: setpoint in range, accel away from center - MEASURE HERE
            // Stay in RISING as long as magnitude is increasing with significant accel
            if (magnitudeIncreasing && hasSignificantAccel) {
                // Still rising - accumulate tracking error
                state->errorAccumulator += trackingError;
                state->sampleCount++;
            } else {
                // No longer rising (accel dropped or direction changed) -> ADJUSTING
                state->windowState = FF_WINDOW_ADJUSTING;
                state->maneuverEndTime = currentTimeUs;
            }
            break;
            
        case FF_WINDOW_ADJUSTING:
            // ADJUSTING: wait 100ms, then process and adjust F term, then -> WAITING
            if (cmpTimeUs(currentTimeUs, state->maneuverEndTime) >= FF_AUTOTUNE_ADJUST_DELAY_US) {
                processManeuverEnd(state, axis);
                state->windowState = FF_WINDOW_WAITING;
            }
            break;
            
        case FF_WINDOW_WAITING:
            // WAITING: wait for setpoint to return to neutral and accel to settle
            // Once neutral with low accel -> back to IDLE (one test cycle complete)
            if (isNeutral && !hasSignificantAccel) {
                state->windowState = FF_WINDOW_IDLE;
            }
            break;
    }
    
    state->prevAccel = absAccel;
    state->prevSetpoint = setpoint;
    
    // Debug output
    if (axis == gyro.gyroDebugAxis) {
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 0, state->gain);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 1, lrintf(trackingError));
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 2, state->windowState);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 3, state->lastAssessment);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 4, state->lastAvgError);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 5, state->sampleCount > 255 ? 255 : state->sampleCount);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 6, state->historyCount);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 7, state->bracketState);
    }
}

// ============================================================================
// MAIN UPDATE
// ============================================================================

void ffAutotuneUpdate(int axis, float setpoint, float gyroRate, float setpointDelta, timeUs_t currentTimeUs)
{
    // Only roll and pitch
    if (axis > FD_PITCH) {
        return;
    }
    
    // Check mode activation
    const bool modeActive = IS_RC_MODE_ACTIVE(BOXFFAUTOTUNE) && 
                            ffAutotuneConfig()->enabled &&
                            ARMING_FLAG(ARMED);
    
    // Mode transition handling
    if (modeActive && !runtime.wasActive) {
        // Just activated
        runtime.active = true;
        beeper(BEEPER_RX_SET);
        
    } else if (!modeActive && runtime.wasActive) {
        // Just deactivated - save if modified
        runtime.active = false;
        
        if (runtime.gainsModified) {
            ffAutotuneSaveGains();
        }
    }
    
    runtime.wasActive = modeActive;
    
    // Update tracking if active
    if (runtime.active) {
        const float pidFrequency = gyro.targetLooptime > 0 ? 
                                   1000000.0f / gyro.targetLooptime : 8000.0f;
        
        updateAxisTracking(axis, setpoint, gyroRate, setpointDelta, pidFrequency, currentTimeUs);
    }
}

#endif // USE_FF_AUTOTUNE
