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
 * Phase 1: Tunes feedforward gain by monitoring setpoint tracking quality
 *          during maneuvers and using intelligent bracketing.
 *
 * Phase 2: Tunes P/D ratio for ringing suppression after Phase 1 converges.
 *          Measures oscillation in the plateau region after maneuver rise.
 *
 * Phase 3: F-term spot check after P/D changes to validate F is still optimal.
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
#define FF_PHASE3_RECHECK_COUNT     3       // Number of maneuvers for F-term spot check

// ============================================================================
// HISTORY ENTRY (Phase 1)
// ============================================================================

typedef struct {
    uint8_t  gain;              // Gain setting used
    int16_t  avgError;          // Average tracking error (x10 for precision)
    uint16_t sampleCount;       // Number of samples in measurement
    bool     valid;             // Entry is valid
} ffHistoryEntry_t;

// ============================================================================
// RINGING HISTORY ENTRY (Phase 2)
// ============================================================================

typedef struct {
    int16_t  pAdjustment;       // P adjustment at measurement time
    int16_t  dAdjustment;       // D adjustment at measurement time
    uint16_t ringingScore;      // Combined ringing score (x10)
    uint16_t zeroCrossings;     // Zero-crossing count
    uint16_t amplitude;         // Peak-to-peak amplitude (x10)
    bool     valid;             // Entry is valid
} ffRingHistoryEntry_t;

// ============================================================================
// PER-AXIS STATE
// ============================================================================

typedef struct {
    // Current gain (runtime, may differ from config during tuning)
    uint8_t gain;

    // Window state machine
    ffWindowState_e windowState;

    // Autotune phase
    ffAutotunePhase_e phase;

    // Maneuver tracking (Phase 1)
    float errorAccumulator;         // Sum of tracking errors
    uint32_t sampleCount;           // Samples in current maneuver
    float prevSetpoint;             // For acceleration calc
    float prevAccel;                // Previous acceleration (for peak detection)

    // Post-maneuver processing
    timeUs_t maneuverEndTime;       // When maneuver ended
    int16_t lastAvgError;           // Last completed maneuver's avg error (x10)
    int8_t lastAssessment;          // Last assessment (-1=lag, 0=optimal, +1=lead)

    // History ring buffer with intelligent management (Phase 1)
    ffHistoryEntry_t history[FF_AUTOTUNE_HISTORY_SIZE];
    uint8_t historyCount;           // Number of valid entries

    // Bracket state (Phase 1 F-term)
    uint8_t lowerGain;              // Best gain that still lags (highest)
    uint8_t upperGain;              // Best gain that leads (lowest)
    int16_t lowerError;             // Error at lower bound
    int16_t upperError;             // Error at upper bound
    ffBracketState_e bracketState;  // Current bracket state

    // Phase 2: Ringing measurement
    float ringPeakPos;              // Max positive error in window (after 1st overshoot)
    float ringPeakNeg;              // Max negative error in window
    uint16_t ringZeroCrossings;     // Zero-crossing count in window
    uint16_t ringSampleCount;       // Samples in current analysis window
    float ringPrevError;            // Previous error for zero-crossing detection
    bool ringFirstPeakPassed;       // Have we passed the first overshoot?
    bool ringWindowActive;          // Currently in analysis window
    timeUs_t ringWindowStartTime;   // When analysis window opened
    float ringFirstPeakValue;       // Tracks the first peak for skipping
    bool ringFirstPeakSign;         // Sign of first peak (true = positive)

    // Phase 2: P/D adjustment state
    int16_t pAdjustment;            // Cumulative P adjustment (negative = decreased)
    int16_t dAdjustment;            // Cumulative D adjustment (positive = increased)
    bool adjustingD;                // Currently in Phase 2b (D adjustment)
    uint16_t lastRingingScore;      // Last measured ringing score (x10)
    ffRingAssessment_e lastRingAssessment; // Last ringing assessment

    // Phase 2: Ringing history for bracketing
    ffRingHistoryEntry_t ringHistory[FF_AUTOTUNE_HISTORY_SIZE];
    uint8_t ringHistoryCount;

    // Phase 2: P bracket state
    int16_t ringLowerP;             // P adjustment with ringing (less negative = higher P)
    int16_t ringUpperP;             // P adjustment well-damped (more negative = lower P)
    ffBracketState_e ringBracketState;

    // Phase 3: F-term recheck
    float recheckErrorAccum;        // Accumulated tracking error for spot check
    uint8_t recheckManeuverCount;   // Number of maneuvers collected
    int16_t recheckAvgError;        // Average error from Phase 1 convergence

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
    bool pendingSave;               // EEPROM write deferred until disarm
    bool gainsLearned;              // Learned gains should be applied (survives mode-off)
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

    // Load Phase 2 adjustments from config
    runtime.axis[FD_ROLL].pAdjustment = ffAutotuneConfig()->p_adj_roll;
    runtime.axis[FD_PITCH].pAdjustment = ffAutotuneConfig()->p_adj_pitch;
    runtime.axis[FD_ROLL].dAdjustment = ffAutotuneConfig()->d_adj_roll;
    runtime.axis[FD_PITCH].dAdjustment = ffAutotuneConfig()->d_adj_pitch;

    // Initialize bracket bounds to extremes
    for (int i = 0; i < 2; i++) {
        runtime.axis[i].lowerGain = 0;
        runtime.axis[i].upperGain = 255;
        runtime.axis[i].bracketState = FF_BRACKET_SEARCHING;
        runtime.axis[i].phase = FF_AUTOTUNE_PHASE1_FF;
        runtime.axis[i].ringBracketState = FF_BRACKET_SEARCHING;
        runtime.axis[i].ringLowerP = 1;     // Sentinel: no bracket yet
        runtime.axis[i].ringUpperP = -100;  // Sentinel: no bracket yet
    }
}

// ============================================================================
// MODE MANAGEMENT
// ============================================================================

bool ffAutotuneIsActive(void)
{
    return runtime.active;
}

bool ffAutotuneHasLearnedGains(void)
{
    return runtime.gainsLearned;
}

bool ffAutotuneIsPhase2Active(void)
{
    if (!runtime.active) {
        return false;
    }
    for (int i = 0; i < 2; i++) {
        if (runtime.axis[i].phase >= FF_AUTOTUNE_PHASE2_PD) {
            return true;
        }
    }
    // Also return true if persisted adjustments exist (even if not actively tuning)
    if (ffAutotuneConfig()->pd_enabled) {
        for (int i = 0; i < 2; i++) {
            if (runtime.axis[i].pAdjustment != 0 || runtime.axis[i].dAdjustment != 0) {
                return true;
            }
        }
    }
    return false;
}

int16_t ffAutotuneGetPAdjustment(int axis)
{
    if (axis > FD_PITCH) {
        return 0;
    }
    return runtime.axis[axis].pAdjustment;
}

int16_t ffAutotuneGetDAdjustment(int axis)
{
    if (axis > FD_PITCH) {
        return 0;
    }
    return runtime.axis[axis].dAdjustment;
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

    // Save Phase 1 gains to RAM config
    ffAutotuneConfigMutable()->gain_roll = runtime.axis[FD_ROLL].gain;
    ffAutotuneConfigMutable()->gain_pitch = runtime.axis[FD_PITCH].gain;

    // Save Phase 2 adjustments to RAM config
    ffAutotuneConfigMutable()->p_adj_roll = runtime.axis[FD_ROLL].pAdjustment;
    ffAutotuneConfigMutable()->p_adj_pitch = runtime.axis[FD_PITCH].pAdjustment;
    ffAutotuneConfigMutable()->d_adj_roll = runtime.axis[FD_ROLL].dAdjustment;
    ffAutotuneConfigMutable()->d_adj_pitch = runtime.axis[FD_PITCH].dAdjustment;

    // Defer EEPROM write to disarm to avoid blocking the PID loop
    runtime.pendingSave = true;

    runtime.gainsModified = false;
}

void ffAutotuneOnDisarm(void)
{
    if (runtime.pendingSave) {
        writeEEPROM();
        runtime.pendingSave = false;
        beeper(BEEPER_READY_BEEP);
    }
}

void ffAutotuneReset(void)
{
    ffAutotuneInit();
    runtime.gainsModified = true;
}

// ============================================================================
// INTELLIGENT HISTORY MANAGEMENT (Phase 1)
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
// GAIN ADJUSTMENT (Phase 1)
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
// PHASE 2: RINGING ANALYSIS
// ============================================================================

static void ringWindowReset(ffAxisState_t *state)
{
    state->ringPeakPos = 0.0f;
    state->ringPeakNeg = 0.0f;
    state->ringZeroCrossings = 0;
    state->ringSampleCount = 0;
    state->ringPrevError = 0.0f;
    state->ringFirstPeakPassed = false;
    state->ringWindowActive = false;
    state->ringWindowStartTime = 0;
    state->ringFirstPeakValue = 0.0f;
}

static void ringWindowOpen(ffAxisState_t *state, float initialError, timeUs_t currentTimeUs)
{
    ringWindowReset(state);
    state->ringWindowActive = true;
    state->ringWindowStartTime = currentTimeUs;
    state->ringPrevError = initialError;
    state->ringFirstPeakSign = (initialError >= 0);
    state->ringFirstPeakValue = fabsf(initialError);
}

static void ringWindowAccumulate(ffAxisState_t *state, float trackingError)
{
    if (!state->ringWindowActive) {
        return;
    }

    const float deadband = (float)ffAutotuneConfig()->ring_deadband;

    state->ringSampleCount++;

    // Zero-crossing detection with deadband
    if (state->ringPrevError != 0.0f) {
        // Check if error crossed zero (with deadband filtering)
        bool prevPositive = (state->ringPrevError > deadband);
        bool prevNegative = (state->ringPrevError < -deadband);
        bool currPositive = (trackingError > deadband);
        bool currNegative = (trackingError < -deadband);

        if ((prevPositive && currNegative) || (prevNegative && currPositive)) {
            if (!state->ringFirstPeakPassed) {
                // First zero-crossing after the initial overshoot - mark it
                state->ringFirstPeakPassed = true;
            } else {
                // Subsequent crossings are ringing
                state->ringZeroCrossings++;
            }
        }
    }

    // Track peak amplitude (only after first peak has passed)
    if (state->ringFirstPeakPassed) {
        if (trackingError > state->ringPeakPos) {
            state->ringPeakPos = trackingError;
        }
        if (trackingError < state->ringPeakNeg) {
            state->ringPeakNeg = trackingError;
        }
    } else {
        // Track first peak value to skip it
        float absErr = fabsf(trackingError);
        if (absErr > state->ringFirstPeakValue) {
            state->ringFirstPeakValue = absErr;
        }
    }

    state->ringPrevError = trackingError;
}

static uint16_t ringWindowClose(ffAxisState_t *state)
{
    state->ringWindowActive = false;

    if (state->ringSampleCount < 10) {
        // Not enough samples for meaningful analysis
        return 0;
    }

    // Calculate peak-to-peak amplitude after first overshoot
    float amplitude = state->ringPeakPos - state->ringPeakNeg;

    // Calculate average peak amplitude for scoring
    float avgPeakAmplitude = amplitude / 2.0f;

    // Combined ringing score: zero_crossings * avg_peak_amplitude
    // Scale by 10 for integer storage
    float score = (float)state->ringZeroCrossings * avgPeakAmplitude;
    uint16_t ringingScore = (uint16_t)MIN(score, 65535);

    return ringingScore;
}

static ffRingAssessment_e assessRinging(uint16_t ringingScore)
{
    const uint8_t threshold = ffAutotuneConfig()->ring_threshold;
    const uint8_t thresholdLow = threshold / 2;

    if (ringingScore > threshold) {
        return FF_RING_RINGING;
    } else if (ringingScore > thresholdLow) {
        return FF_RING_MILD;
    }
    return FF_RING_WELL_DAMPED;
}

// ============================================================================
// PHASE 2: P/D ADJUSTMENT
// ============================================================================

static void processPhase2Ringing(ffAxisState_t *state)
{
    uint16_t ringingScore = ringWindowClose(state);
    state->lastRingingScore = ringingScore;
    state->lastRingAssessment = assessRinging(ringingScore);

    // Store in ring history
    if (state->ringHistoryCount < FF_AUTOTUNE_HISTORY_SIZE) {
        int idx = state->ringHistoryCount++;
        state->ringHistory[idx].pAdjustment = state->pAdjustment;
        state->ringHistory[idx].dAdjustment = state->dAdjustment;
        state->ringHistory[idx].ringingScore = ringingScore;
        state->ringHistory[idx].zeroCrossings = state->ringZeroCrossings;
        state->ringHistory[idx].amplitude = (uint16_t)(state->ringPeakPos - state->ringPeakNeg);
        state->ringHistory[idx].valid = true;
    }

    // Skip adjustment if bracket already converged
    if (state->ringBracketState == FF_BRACKET_CONVERGED) {
        return;
    }

    const int16_t pStep = ffAutotuneConfig()->p_step;
    const int16_t dStep = ffAutotuneConfig()->d_step;
    const int16_t pAdjMax = -(int16_t)ffAutotuneConfig()->p_adjust_max;  // Negative: P reduction
    const int16_t dAdjMax = (int16_t)ffAutotuneConfig()->d_adjust_max;

    if (!state->adjustingD) {
        // Phase 2a: P adjustment
        if (state->lastRingAssessment == FF_RING_RINGING) {
            // Ringing detected: this P value has ringing (too high)
            // Record as lower bound of P bracket (less negative = higher P = ringing)
            if (state->ringLowerP == 1 || state->pAdjustment > state->ringLowerP) {
                state->ringLowerP = state->pAdjustment;
            }

            // Decrease P (make adjustment more negative)
            int16_t newP = state->pAdjustment - pStep;
            if (newP < pAdjMax) {
                // Hit P reduction limit, switch to Phase 2b (D adjustment)
                state->pAdjustment = pAdjMax;
                state->adjustingD = true;
            } else {
                state->pAdjustment = newP;
            }
            runtime.gainsModified = true;

        } else if (state->lastRingAssessment == FF_RING_WELL_DAMPED) {
            // Well damped: this P value is good (low enough)
            // Record as upper bound (more negative = lower P = well-damped)
            if (state->ringUpperP == -100 || state->pAdjustment < state->ringUpperP) {
                state->ringUpperP = state->pAdjustment;
            }

            // Check if we have a bracket
            if (state->ringLowerP != 1 && state->ringUpperP != -100 &&
                state->ringLowerP > state->ringUpperP) {
                // Bracket established
                if (state->ringBracketState == FF_BRACKET_SEARCHING) {
                    state->ringBracketState = FF_BRACKET_BRACKETED;
                }

                // Check convergence
                int16_t bracketWidth = state->ringLowerP - state->ringUpperP;
                if (bracketWidth <= pStep) {
                    // Converged: use the well-damped value
                    state->pAdjustment = state->ringUpperP;
                    state->ringBracketState = FF_BRACKET_CONVERGED;
                    runtime.gainsModified = true;
                    // Transition to Phase 3
                    state->phase = FF_AUTOTUNE_PHASE3_RECHECK;
                    state->recheckErrorAccum = 0;
                    state->recheckManeuverCount = 0;
                    // Store current Phase 1 error for comparison
                    state->recheckAvgError = state->lastAvgError;
                } else {
                    // Binary search: try midpoint
                    state->pAdjustment = (state->ringLowerP + state->ringUpperP) / 2;
                    runtime.gainsModified = true;
                }
            }
            // If no bracket yet, P is already good - may converge

        } else {
            // MILD ringing - could go either way
            // If we have a bracket, do binary search
            if (state->ringBracketState == FF_BRACKET_BRACKETED) {
                state->pAdjustment = (state->ringLowerP + state->ringUpperP) / 2;
                runtime.gainsModified = true;
            }
            // If no bracket, continue decreasing P cautiously
            else if (state->ringLowerP != 1) {
                int16_t newP = state->pAdjustment - pStep;
                if (newP >= pAdjMax) {
                    state->pAdjustment = newP;
                    runtime.gainsModified = true;
                }
            }
        }
    } else {
        // Phase 2b: D adjustment (fallback when P reduction alone insufficient)
        if (state->lastRingAssessment >= FF_RING_MILD) {
            // Still ringing or mild: increase D
            int16_t newD = state->dAdjustment + dStep;
            if (newD <= dAdjMax) {
                state->dAdjustment = newD;
                runtime.gainsModified = true;
            } else {
                // Hit D limit, converge at current values
                state->ringBracketState = FF_BRACKET_CONVERGED;
                state->phase = FF_AUTOTUNE_PHASE3_RECHECK;
                state->recheckErrorAccum = 0;
                state->recheckManeuverCount = 0;
                state->recheckAvgError = state->lastAvgError;
            }
        } else {
            // Well damped with D increase
            state->ringBracketState = FF_BRACKET_CONVERGED;
            runtime.gainsModified = true;
            state->phase = FF_AUTOTUNE_PHASE3_RECHECK;
            state->recheckErrorAccum = 0;
            state->recheckManeuverCount = 0;
            state->recheckAvgError = state->lastAvgError;
        }
    }
}

// ============================================================================
// PHASE 3: F-TERM SPOT CHECK
// ============================================================================

static void processPhase3Recheck(ffAxisState_t *state, int16_t avgError)
{
    state->recheckErrorAccum += (float)avgError;
    state->recheckManeuverCount++;

    if (state->recheckManeuverCount >= FF_PHASE3_RECHECK_COUNT) {
        // Calculate average error across recheck maneuvers
        int16_t recheckAvg = lrintf(state->recheckErrorAccum / (float)state->recheckManeuverCount);
        const int16_t deadband = ffAutotuneConfig()->error_deadband * 10;

        // Compare with Phase 1 converged error
        int16_t errorDrift = recheckAvg - state->recheckAvgError;

        if (ABS(errorDrift) <= deadband) {
            // F-term still valid, all phases complete
            state->phase = FF_AUTOTUNE_COMPLETE;
        } else {
            // F-term drifted, re-open Phase 1 bracket near current gain
            // Narrow bracket around current value for fast re-convergence
            uint8_t currentGain = state->gain;
            uint8_t step = ffAutotuneConfig()->gain_step;

            if (errorDrift < -deadband) {
                // Lag: F too low after P reduction
                state->lowerGain = currentGain;
                state->lowerError = recheckAvg;
                state->upperGain = currentGain + step * 2;
            } else {
                // Lead: F too high
                state->upperGain = currentGain;
                state->upperError = recheckAvg;
                state->lowerGain = (currentGain > step * 2) ? currentGain - step * 2 : 0;
            }

            state->bracketState = FF_BRACKET_BRACKETED;
            state->phase = FF_AUTOTUNE_PHASE1_FF;
        }

        state->recheckErrorAccum = 0;
        state->recheckManeuverCount = 0;
    }
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

    // Calculate average error (x10 for precision in storage)
    const float avgErrorF = state->errorAccumulator / (float)state->sampleCount;
    const int16_t avgError = lrintf(avgErrorF * 10.0f);

    // Store results
    state->lastAvgError = avgError;
    state->lastAssessment = classifyError(avgError, ffAutotuneConfig()->error_deadband * 10);

    switch (state->phase) {
        case FF_AUTOTUNE_PHASE1_FF:
            // Phase 1: F-term tuning
            addHistoryEntry(state, state->gain, avgError, state->sampleCount);

            if (state->bracketState != FF_BRACKET_CONVERGED) {
                uint8_t nextGain = calculateNextGain(state, avgError);
                if (nextGain != state->gain) {
                    state->gain = nextGain;
                    state->adjustmentCount++;
                    runtime.gainsModified = true;
                    if (nextGain > 0) {
                        runtime.gainsLearned = true;
                    }
                }
            } else {
                // Phase 1 converged - transition to Phase 2 if enabled
                if (ffAutotuneConfig()->pd_enabled) {
                    state->phase = FF_AUTOTUNE_PHASE2_PD;
                    ringWindowReset(state);
                } else {
                    state->phase = FF_AUTOTUNE_COMPLETE;
                }
            }
            break;

        case FF_AUTOTUNE_PHASE2_PD:
            // Phase 2: Process ringing data (collected during RISING->ADJUSTING transition)
            // The ringing window data is processed separately in processPhase2Ringing()
            // Here we just track the F-term error for reference
            break;

        case FF_AUTOTUNE_PHASE3_RECHECK:
            // Phase 3: F-term spot check
            processPhase3Recheck(state, avgError);
            break;

        case FF_AUTOTUNE_COMPLETE:
            // All done, no processing needed
            break;
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

    // Calculate setpoint acceleration (deg/s^2)
    const float setpointAccel = setpointDelta * pidFrequency;
    const float absAccel = fabsf(setpointAccel);

    // Tracking error using magnitude comparison (works for both positive and negative maneuvers)
    // Positive = gyro magnitude ahead of setpoint magnitude (LEAD)
    // Negative = gyro magnitude behind setpoint magnitude (LAG)
    const float trackingError = fabsf(gyroRate) - fabsf(setpoint);

    // Signed tracking error for ringing analysis (gyro - setpoint)
    const float signedError = gyroRate - setpoint;

    // Determine if setpoint magnitude is increasing (stick moving away from center)
    const bool magnitudeIncreasing = (setpoint * setpointDelta) > 0;

    // Zone checks
    const bool isNeutral = (absSetpoint < setpointLow);
    const bool inSetpointWindow = (absSetpoint >= setpointLow) && (absSetpoint <= setpointHigh);
    const bool hasSignificantAccel = (absAccel >= minAccel);

    // Phase 2: update ringing window if active
    if (state->ringWindowActive) {
        const uint32_t ringWindowUs = (uint32_t)ffAutotuneConfig()->ring_window_ms * 1000;
        if (cmpTimeUs(currentTimeUs, state->ringWindowStartTime) >= (timeDelta_t)ringWindowUs) {
            // Window duration expired
            if (state->phase == FF_AUTOTUNE_PHASE2_PD) {
                processPhase2Ringing(state);
            } else {
                ringWindowClose(state);
            }
        } else {
            ringWindowAccumulate(state, signedError);
        }
    }

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

                // Phase 2: Open ringing analysis window at end of rise
                if (state->phase == FF_AUTOTUNE_PHASE2_PD && !state->ringWindowActive) {
                    ringWindowOpen(state, signedError, currentTimeUs);
                }
            }
            break;

        case FF_WINDOW_ADJUSTING:
            // ADJUSTING: wait 100ms, then process and adjust F term, then -> WAITING
            // Phase 2: continue collecting ringing data during this delay
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
                // Close ringing window if still open
                if (state->ringWindowActive) {
                    if (state->phase == FF_AUTOTUNE_PHASE2_PD) {
                        processPhase2Ringing(state);
                    } else {
                        ringWindowClose(state);
                    }
                }
            }
            break;
    }

    state->prevAccel = absAccel;
    state->prevSetpoint = setpoint;

    // Debug output - Phase 1
    if (axis == gyro.gyroDebugAxis) {
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 0, state->gain);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 1, lrintf(trackingError));
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 2, state->windowState);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 3, state->lastAssessment);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 4, state->lastAvgError);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 5, state->sampleCount > 255 ? 255 : state->sampleCount);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 6, state->historyCount);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 7, state->bracketState);

        // Debug output - Phase 2
        // ch0: ringing score, ch1: ring amplitude, ch2: P adj, ch3: D adj
        // ch4: ring window active, ch5: autotune phase, ch6: ring assessment
        // ch7: recheck status (0=not started, 1=collecting, 2=passed, 3=re-tuning)
        DEBUG_SET(DEBUG_FF_AUTOTUNE_PD, 0, state->lastRingingScore);
        DEBUG_SET(DEBUG_FF_AUTOTUNE_PD, 1, lrintf(state->ringPeakPos - state->ringPeakNeg));
        DEBUG_SET(DEBUG_FF_AUTOTUNE_PD, 2, state->pAdjustment);
        DEBUG_SET(DEBUG_FF_AUTOTUNE_PD, 3, state->dAdjustment);
        DEBUG_SET(DEBUG_FF_AUTOTUNE_PD, 4, state->ringWindowActive ? 1 : 0);
        DEBUG_SET(DEBUG_FF_AUTOTUNE_PD, 5, state->phase);
        DEBUG_SET(DEBUG_FF_AUTOTUNE_PD, 6, state->lastRingAssessment);
        {
            int recheckStatus = 0;
            if (state->phase == FF_AUTOTUNE_PHASE3_RECHECK) {
                recheckStatus = 1;  // collecting
            } else if (state->phase == FF_AUTOTUNE_COMPLETE) {
                recheckStatus = 2;  // passed
            } else if (state->phase == FF_AUTOTUNE_PHASE1_FF && state->recheckManeuverCount > 0) {
                recheckStatus = 3;  // re-tuning (went back to Phase 1 from Phase 3)
            }
            DEBUG_SET(DEBUG_FF_AUTOTUNE_PD, 7, recheckStatus);
        }
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
        // Mark gains as learned if we have non-zero gains
        if (runtime.axis[FD_ROLL].gain > 0 || runtime.axis[FD_PITCH].gain > 0) {
            runtime.gainsLearned = true;
        }
        beeper(BEEPER_RX_SET);

    } else if (!modeActive && runtime.wasActive) {
        // Just deactivated - stop learning but keep gains applied
        runtime.active = false;
        // gainsLearned stays true — learned F/P/D values persist until disarm

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
