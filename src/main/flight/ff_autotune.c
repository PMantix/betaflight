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
 * FF Autotune v4: Concurrent Metric Assessment
 * =============================================
 *
 * All three metrics are measured concurrently on every maneuver:
 *   - Tracking error (F-term quality)
 *   - Ringing score (P/D ratio quality)
 *   - Noise score (D-term noise level)
 *
 * A priority-based decision engine picks which adjustment to apply:
 *   1. Severe tracking error -> adjust F
 *   2. Ringing above threshold -> adjust P/D ratio
 *   3. Noise above baseline -> scale down P+D
 *   4. Mild tracking error -> fine-tune F
 *   5. Mild ringing -> fine-tune P
 *   6. All within deadbands -> COMPLETE (after N consecutive)
 *
 * Bootstrap: If no prior F gains exist, Phase 1 F-tuning runs first.
 *            Once F converges, concurrent mode takes over.
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

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"

#include "pg/rpm_filter.h"

#include "ff_autotune.h"

// ============================================================================
// CONSTANTS
// ============================================================================

#define FF_AUTOTUNE_HISTORY_SIZE    8       // Ring buffer size per axis
#define FF_AUTOTUNE_MIN_SAMPLES     50      // Minimum samples for valid maneuver
#define FF_AUTOTUNE_ADJUST_DELAY_US 100000  // 100ms delay before adjusting F term
#define FF_AUTOTUNE_MIN_IDLE_US     200000  // 200ms minimum IDLE dwell before next RISING
#define FF_CONVERGENCE_COUNT        3       // Consecutive all-good maneuvers to declare COMPLETE
// FF_NOISE_FLOOR_SCORE replaced by configurable noise_floor (default 600)
#define FF_WIGGLE_AMPLITUDE         40.0f   // Wiggle amplitude (deg/s) for COMPLETE notification
#define FF_WIGGLE_DURATION_US       400000  // 400ms wiggle (1.5 sine cycles, ~3.75Hz)
#define FF_WIGGLE_DELAY_US          500000  // 500ms delay after IDLE entry before wiggle

// Noise tool identifiers (for revert tracking)
#define NOISE_TOOL_NONE       0
#define NOISE_TOOL_DTERM_LPF2 1
#define NOISE_TOOL_GYRO_LPF2  2
#define NOISE_TOOL_SCALE      3

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
// CONCURRENT METRIC SNAPSHOT
// ============================================================================

typedef struct {
    int16_t  trackingError;         // avgError x10
    int8_t   trackingAssessment;    // -1=lag, 0=optimal, +1=lead
    uint16_t ringingScore;
    ffDampingAssessment_e dampingAssessment;
    uint16_t noiseScore;            // avg|D| x10
    bool     valid;
} ffMetricSnapshot_t;

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
    float expectedLagAccumulator;   // Sum of expected filter-delay lag per sample
    uint32_t sampleCount;           // Samples in current maneuver
    float prevSetpoint;             // For acceleration calc
    float prevAccel;                // Previous acceleration (for peak detection)

    // Post-maneuver processing
    timeUs_t maneuverEndTime;       // When maneuver ended
    timeUs_t idleEntryTime;         // When axis entered IDLE (for dwell time check)
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

    // Phase 2: Ringing/damping measurement
    float ringPeakPos;              // Max positive error in window (after 1st overshoot)
    float ringPeakNeg;              // Max negative error in window
    uint16_t ringZeroCrossings;     // Zero-crossing count in window
    uint16_t ringSampleCount;       // Samples in current analysis window
    int8_t ringLastSide;            // Schmitt trigger: last side of deadband (-1=neg, 0=neutral, 1=pos)
    bool ringFirstPeakPassed;       // Have we passed the first overshoot?
    bool ringWindowActive;          // Currently in analysis window
    timeUs_t ringWindowStartTime;   // When analysis window opened
    float ringFirstPeakValue;       // Tracks the first peak for skipping
    float ringSignedErrorAccum;     // Sum of signed errors during ringing window
    uint32_t ringSignedErrorCount;  // Sample count for mean signed error

    // Phase 2: P/D adjustment state
    int16_t pAdjustment;            // Cumulative P adjustment (negative = decreased)
    int16_t dAdjustment;            // Cumulative D adjustment (positive = increased)
    uint16_t lastRingingScore;      // Last measured ringing score (x10)
    ffDampingAssessment_e lastDampingAssessment; // Last damping assessment

    // Phase 2: Ringing history for bracketing
    ffRingHistoryEntry_t ringHistory[FF_AUTOTUNE_HISTORY_SIZE];
    uint8_t ringHistoryCount;

    // Phase 2: P bracket state
    int16_t ringLowerP;             // P adjustment with ringing (less negative = higher P)
    int16_t ringUpperP;             // P adjustment well-damped (more negative = lower P)
    ffBracketState_e ringBracketState;

    // Phase 2b: Noise measurement (during RISING phase)
    float noiseAccumulator;             // Sum of |D-term| values during rise
    uint32_t noiseSampleCount;          // Samples accumulated during rise
    uint16_t lastNoiseScore;            // Last measured noise (×10)
    uint16_t noiseBaseline;             // Baseline noise before scaling started (×10)

    // Phase 2b: Gyro noise measurement (during RISING phase)
    float gyroNoiseAccumulator;         // Sum of |gyroADC - gyroADCf| during rise
    uint32_t gyroNoiseSampleCount;      // Samples accumulated during rise
    uint16_t lastGyroNoiseScore;        // Last measured gyro noise (×10)
    ffNoiseSrc_e lastNoiseSrc;          // Last diagnosed noise source

    // Phase 2b: Gain noise — ratio-preserving scale
    uint8_t gainScalePercent;           // 100 = no scaling, decreased for noise
    uint8_t lastNoiseTool;              // Which tool was last applied (for revert)

    // D noise ceiling learning
    uint8_t dNoiseTriggerCount;         // Times noise triggered near current D level
    int16_t lastNoiseDLevel;            // Effective D when noise last triggered
    uint16_t maneuversSinceNoise;       // Consecutive maneuvers without noise (for reassessment)

    // Concurrent metric snapshot (all three metrics from last maneuver)
    ffMetricSnapshot_t lastMetrics;
    uint8_t convergenceCount;       // Consecutive all-good maneuvers
    bool    bootstrapComplete;      // F-term initial convergence done

    // COMPLETE notification wiggle
    bool wigglePending;             // Waiting to start wiggle (set on convergence)
    bool wiggleActive;              // Currently playing wiggle
    timeUs_t wiggleStartTime;       // When wiggle started
    float wiggleOffset;             // Current setpoint offset (deg/s)

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
    float filterGroupDelay;         // Estimated gyro filter chain group delay (seconds)
    uint16_t lpf2BaseCutoff;        // Original gyro LPF2 cutoff at init (Hz)
    int16_t lpf2Adjustment;         // Current gyro LPF2 cutoff adjustment (Hz, negative = reduced)
    uint16_t dtermLpf2BaseCutoff;   // Original D-term LPF2 cutoff at init (Hz)
    int16_t dtermLpf2Adjustment;    // Current D-term LPF2 cutoff adjustment (Hz, negative = reduced)
    uint8_t dtermLpf2Type;          // Cached D-term LPF2 filter type
    ffAxisState_t axis[2];          // Roll and Pitch only
} runtime;

// ============================================================================
// FILTER GROUP DELAY ESTIMATION
// ============================================================================

static uint16_t getEffectiveLpf2Hz(void)
{
    int16_t effective = (int16_t)gyroConfig()->gyro_lpf2_static_hz + runtime.lpf2Adjustment;
    return (effective > 0) ? (uint16_t)effective : 0;
}

static float estimateFilterGroupDelay(void)
{
    float delay = 0.0f;

    // Gyro lowpass filter 1
    const uint16_t lpf1Hz = gyroConfig()->gyro_lpf1_static_hz;
    if (lpf1Hz > 0) {
        switch (gyroConfig()->gyro_lpf1_type) {
            case FILTER_PT1:    delay += 0.159f / lpf1Hz; break;
            case FILTER_BIQUAD: delay += 0.225f / lpf1Hz; break;
            case FILTER_PT2:    delay += 0.318f / lpf1Hz; break;
            case FILTER_PT3:    delay += 0.477f / lpf1Hz; break;
        }
    }

    // Gyro lowpass filter 2 (uses effective cutoff including autotune adjustment)
    const uint16_t lpf2Hz = getEffectiveLpf2Hz();
    if (lpf2Hz > 0) {
        switch (gyroConfig()->gyro_lpf2_type) {
            case FILTER_PT1:    delay += 0.159f / lpf2Hz; break;
            case FILTER_BIQUAD: delay += 0.225f / lpf2Hz; break;
            case FILTER_PT2:    delay += 0.318f / lpf2Hz; break;
            case FILTER_PT3:    delay += 0.477f / lpf2Hz; break;
        }
    }

    // Static notch filters (biquad-based)
    if (gyroConfig()->gyro_soft_notch_hz_1 > 0) {
        delay += 0.225f / gyroConfig()->gyro_soft_notch_hz_1;
    }
    if (gyroConfig()->gyro_soft_notch_hz_2 > 0) {
        delay += 0.225f / gyroConfig()->gyro_soft_notch_hz_2;
    }

#ifdef USE_RPM_FILTER
    // RPM notch filters: center frequencies at motor RPM harmonics (200-800Hz typically)
    // At stick-input frequencies (~10Hz), biquad notch group delay approaches zero
    // because the signal frequency is far below the notch center.
    // Only add a small residual contribution per notch.
    if (isRpmFilterEnabled()) {
        const int harmonics = rpmFilterConfig()->rpm_filter_harmonics;
        delay += (float)(getMotorCount() * harmonics) * 0.00002f;
    }
#endif

    return delay;
}

// ============================================================================
// RUNTIME LPF2 CUTOFF UPDATE
// ============================================================================

static void applyLpf2Cutoff(uint16_t newCutoffHz)
{
    if (newCutoffHz == 0) {
        return;
    }

    const float gyroDt = gyro.sampleLooptime * 1e-6f;

    switch (gyroConfig()->gyro_lpf2_type) {
        case FILTER_PT1:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&gyro.lowpass2Filter[axis].pt1FilterState, pt1FilterGain(newCutoffHz, gyroDt));
            }
            break;
        case FILTER_BIQUAD:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&gyro.lowpass2Filter[axis].biquadFilterState, newCutoffHz, gyro.sampleLooptime);
            }
            break;
        case FILTER_PT2:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt2FilterUpdateCutoff(&gyro.lowpass2Filter[axis].pt2FilterState, pt2FilterGain(newCutoffHz, gyroDt));
            }
            break;
        case FILTER_PT3:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt3FilterUpdateCutoff(&gyro.lowpass2Filter[axis].pt3FilterState, pt3FilterGain(newCutoffHz, gyroDt));
            }
            break;
    }
}

// ============================================================================
// RUNTIME D-TERM LPF2 CUTOFF UPDATE
// ============================================================================

static uint16_t getEffectiveDtermLpf2Hz(void)
{
    int16_t effective = (int16_t)runtime.dtermLpf2BaseCutoff + runtime.dtermLpf2Adjustment;
    return (effective > 0) ? (uint16_t)effective : 0;
}

static void applyDtermLpf2Cutoff(uint16_t newCutoffHz)
{
    if (newCutoffHz == 0) {
        return;
    }

    switch (runtime.dtermLpf2Type) {
        case FILTER_PT1:
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterUpdateCutoff(&pidRuntime.dtermLowpass2[axis].pt1Filter, pt1FilterGain(newCutoffHz, pidRuntime.dT));
            }
            break;
        case FILTER_BIQUAD:
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                biquadFilterUpdateLPF(&pidRuntime.dtermLowpass2[axis].biquadFilter, newCutoffHz, targetPidLooptime);
            }
            break;
        case FILTER_PT2:
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt2FilterUpdateCutoff(&pidRuntime.dtermLowpass2[axis].pt2Filter, pt2FilterGain(newCutoffHz, pidRuntime.dT));
            }
            break;
        case FILTER_PT3:
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt3FilterUpdateCutoff(&pidRuntime.dtermLowpass2[axis].pt3Filter, pt3FilterGain(newCutoffHz, pidRuntime.dT));
            }
            break;
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void ffAutotuneInit(void)
{
    memset(&runtime, 0, sizeof(runtime));

    // Load gyro LPF2 base cutoff and persisted adjustment
    runtime.lpf2BaseCutoff = gyroConfig()->gyro_lpf2_static_hz;
    runtime.lpf2Adjustment = ffAutotuneConfig()->lpf2_adj;

    // Apply persisted gyro LPF2 adjustment to runtime filter
    if (runtime.lpf2Adjustment != 0 && runtime.lpf2BaseCutoff > 0) {
        const uint16_t effectiveHz = getEffectiveLpf2Hz();
        if (effectiveHz > 0) {
            applyLpf2Cutoff(effectiveHz);
        }
    }

    // Load D-term LPF2 base cutoff, type, and persisted adjustment
    runtime.dtermLpf2BaseCutoff = currentPidProfile->dterm_lpf2_static_hz;
    runtime.dtermLpf2Type = currentPidProfile->dterm_lpf2_type;
    runtime.dtermLpf2Adjustment = ffAutotuneConfig()->dterm_lpf2_adj;

    // Apply persisted D-term LPF2 adjustment to runtime filter
    if (runtime.dtermLpf2Adjustment != 0 && runtime.dtermLpf2BaseCutoff > 0) {
        const uint16_t effectiveHz = getEffectiveDtermLpf2Hz();
        if (effectiveHz > 0) {
            applyDtermLpf2Cutoff(effectiveHz);
        }
    }

    // Estimate gyro filter chain group delay (uses effective LPF2 cutoff)
    runtime.filterGroupDelay = estimateFilterGroupDelay();

    // Load F gains: start from configured F, apply learned adjustment
    for (int i = 0; i < 2; i++) {
        const int16_t fAdj = (i == 0) ? ffAutotuneConfig()->f_adj_roll : ffAutotuneConfig()->f_adj_pitch;
        int16_t gain = (int16_t)currentPidProfile->pid[i].F + fAdj;
        if (gain < ffAutotuneConfig()->gain_min) gain = ffAutotuneConfig()->gain_min;
        if (gain > ffAutotuneConfig()->gain_max) gain = ffAutotuneConfig()->gain_max;
        runtime.axis[i].gain = (uint8_t)gain;
    }

    // Load Phase 2 adjustments from config
    runtime.axis[FD_ROLL].pAdjustment = ffAutotuneConfig()->p_adj_roll;
    runtime.axis[FD_PITCH].pAdjustment = ffAutotuneConfig()->p_adj_pitch;
    runtime.axis[FD_ROLL].dAdjustment = ffAutotuneConfig()->d_adj_roll;
    runtime.axis[FD_PITCH].dAdjustment = ffAutotuneConfig()->d_adj_pitch;

    // Load gain scale percent from config
    runtime.axis[FD_ROLL].gainScalePercent = ffAutotuneConfig()->gain_scale_roll;
    runtime.axis[FD_PITCH].gainScalePercent = ffAutotuneConfig()->gain_scale_pitch;

    // Initialize bracket bounds to extremes
    for (int i = 0; i < 2; i++) {
        runtime.axis[i].lowerGain = 0;
        runtime.axis[i].upperGain = 255;
        runtime.axis[i].bracketState = FF_BRACKET_SEARCHING;
        runtime.axis[i].phase = FF_AUTOTUNE_PHASE1_FF;
        runtime.axis[i].ringBracketState = FF_BRACKET_SEARCHING;
        runtime.axis[i].ringLowerP = 1;     // Sentinel: no bracket yet
        runtime.axis[i].ringUpperP = -100;  // Sentinel: no bracket yet

        // Bootstrap: if persisted gains exist from prior flight, skip Phase 1
        if (runtime.axis[i].gain > 0) {
            runtime.axis[i].bootstrapComplete = true;
        }
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
        if (runtime.axis[i].phase >= FF_AUTOTUNE_PHASE2_UNDERDAMPED) {
            return true;
        }
    }
    // Also return true if persisted adjustments exist (even if not actively tuning)
    if (ffAutotuneConfig()->pd_enabled) {
        for (int i = 0; i < 2; i++) {
            if (runtime.axis[i].pAdjustment != 0 || runtime.axis[i].dAdjustment != 0 ||
                runtime.axis[i].gainScalePercent < 100) {
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
    const int16_t baseP = (int16_t)currentPidProfile->pid[axis].P;
    const int16_t pAdj = runtime.axis[axis].pAdjustment;
    int16_t effectiveP = (int16_t)(((int32_t)(baseP + pAdj) * runtime.axis[axis].gainScalePercent) / 100);
    int16_t adj = effectiveP - baseP;
    const int16_t minAdj = 1 - baseP;
    return (adj < minAdj) ? minAdj : adj;
}

int16_t ffAutotuneGetDAdjustment(int axis)
{
    if (axis > FD_PITCH) {
        return 0;
    }
    const int16_t baseD = (int16_t)currentPidProfile->pid[axis].D;
    const int16_t dAdj = runtime.axis[axis].dAdjustment;
    int16_t effectiveD = (int16_t)(((int32_t)(baseD + dAdj) * runtime.axis[axis].gainScalePercent) / 100);
    int16_t adj = effectiveD - baseD;
    const int16_t minAdj = 1 - baseD;
    return (adj < minAdj) ? minAdj : adj;
}

int16_t ffAutotuneGetFAdjustment(int axis)
{
    if (axis > FD_PITCH) {
        return 0;
    }
    return (int16_t)runtime.axis[axis].gain - (int16_t)currentPidProfile->pid[axis].F;
}

int16_t ffAutotuneGetLpf2Adjustment(void)
{
    return runtime.lpf2Adjustment;
}

int16_t ffAutotuneGetDtermLpf2Adjustment(void)
{
    return runtime.dtermLpf2Adjustment;
}

float ffAutotuneGetWiggleOffset(int axis)
{
    if (axis > FD_PITCH) {
        return 0.0f;
    }
    return runtime.axis[axis].wiggleOffset;
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

    // Save F adjustments (delta from configured F) to RAM config
    ffAutotuneConfigMutable()->f_adj_roll = (int8_t)((int16_t)runtime.axis[FD_ROLL].gain - (int16_t)currentPidProfile->pid[FD_ROLL].F);
    ffAutotuneConfigMutable()->f_adj_pitch = (int8_t)((int16_t)runtime.axis[FD_PITCH].gain - (int16_t)currentPidProfile->pid[FD_PITCH].F);

    // Save Phase 2 adjustments to RAM config
    ffAutotuneConfigMutable()->p_adj_roll = runtime.axis[FD_ROLL].pAdjustment;
    ffAutotuneConfigMutable()->p_adj_pitch = runtime.axis[FD_PITCH].pAdjustment;
    ffAutotuneConfigMutable()->d_adj_roll = runtime.axis[FD_ROLL].dAdjustment;
    ffAutotuneConfigMutable()->d_adj_pitch = runtime.axis[FD_PITCH].dAdjustment;

    // Save gain scale percent to RAM config
    ffAutotuneConfigMutable()->gain_scale_roll = runtime.axis[FD_ROLL].gainScalePercent;
    ffAutotuneConfigMutable()->gain_scale_pitch = runtime.axis[FD_PITCH].gainScalePercent;

    // Save D noise ceilings
    ffAutotuneConfigMutable()->d_noise_ceiling_roll = runtime.axis[FD_ROLL].lastNoiseDLevel > 0 ?
        (int8_t)runtime.axis[FD_ROLL].lastNoiseDLevel : ffAutotuneConfig()->d_noise_ceiling_roll;
    ffAutotuneConfigMutable()->d_noise_ceiling_pitch = runtime.axis[FD_PITCH].lastNoiseDLevel > 0 ?
        (int8_t)runtime.axis[FD_PITCH].lastNoiseDLevel : ffAutotuneConfig()->d_noise_ceiling_pitch;

    // Save gyro LPF2 adjustment
    ffAutotuneConfigMutable()->lpf2_adj = runtime.lpf2Adjustment;

    // Save D-term LPF2 adjustment
    ffAutotuneConfigMutable()->dterm_lpf2_adj = runtime.dtermLpf2Adjustment;

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
    state->ringLastSide = 0;
    state->ringFirstPeakPassed = false;
    state->ringWindowActive = false;
    state->ringWindowStartTime = 0;
    state->ringFirstPeakValue = 0.0f;
    state->ringSignedErrorAccum = 0.0f;
    state->ringSignedErrorCount = 0;
}

static void ringWindowOpen(ffAxisState_t *state, float initialError, timeUs_t currentTimeUs)
{
    ringWindowReset(state);
    state->ringWindowActive = true;
    state->ringWindowStartTime = currentTimeUs;

    // Initialize Schmitt trigger with the side of the initial error
    const float deadband = (float)ffAutotuneConfig()->ring_deadband;
    if (initialError > deadband) {
        state->ringLastSide = 1;
    } else if (initialError < -deadband) {
        state->ringLastSide = -1;
    } else {
        state->ringLastSide = 0;
    }
    state->ringFirstPeakValue = fabsf(initialError);
    state->ringSignedErrorAccum = initialError;
    state->ringSignedErrorCount = 1;
}

static void ringWindowAccumulate(ffAxisState_t *state, float trackingError)
{
    if (!state->ringWindowActive) {
        return;
    }

    const float deadband = (float)ffAutotuneConfig()->ring_deadband;

    state->ringSampleCount++;
    state->ringSignedErrorAccum += trackingError;
    state->ringSignedErrorCount++;

    // Schmitt trigger zero-crossing detection with deadband hysteresis
    // Latch the last side the signal was on; only trigger a crossing when
    // the signal reaches the OPPOSITE threshold, avoiding false negatives
    // from gradual transitions through the deadband zone.
    if (trackingError > deadband) {
        if (state->ringLastSide == -1) {
            // Signal was on negative side, now crossed to positive
            if (!state->ringFirstPeakPassed) {
                state->ringFirstPeakPassed = true;
            } else {
                state->ringZeroCrossings++;
            }
        }
        state->ringLastSide = 1;
    } else if (trackingError < -deadband) {
        if (state->ringLastSide == 1) {
            // Signal was on positive side, now crossed to negative
            if (!state->ringFirstPeakPassed) {
                state->ringFirstPeakPassed = true;
            } else {
                state->ringZeroCrossings++;
            }
        }
        state->ringLastSide = -1;
    }
    // When inside [-deadband, +deadband], ringLastSide retains its value (hysteresis)

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

static ffDampingAssessment_e assessDamping(ffAxisState_t *state, uint16_t ringingScore)
{
    const uint8_t ringThreshold = ffAutotuneConfig()->ring_threshold;

    if (ringingScore > ringThreshold) {
        return FF_DAMPING_UNDERDAMPED;
    }

    // Overdamped: low oscillation + persistent lag during settling
    if (state->ringSignedErrorCount >= 10) {
        float meanSignedError = state->ringSignedErrorAccum / (float)state->ringSignedErrorCount;
        const float overdampedThreshold = -(float)ffAutotuneConfig()->error_deadband;
        if (meanSignedError < overdampedThreshold && state->ringZeroCrossings <= 1) {
            return FF_DAMPING_OVERDAMPED;
        }
    }

    return FF_DAMPING_GOOD;
}

static int8_t getDNoiseCeiling(int axis)
{
    return (axis == FD_ROLL) ? ffAutotuneConfig()->d_noise_ceiling_roll
                             : ffAutotuneConfig()->d_noise_ceiling_pitch;
}

// ============================================================================
// PHASE 2: P/D ADJUSTMENT
// ============================================================================

static void processDamping(ffAxisState_t *state, int axis)
{
    uint16_t ringingScore = ringWindowClose(state);
    state->lastRingingScore = ringingScore;
    state->lastDampingAssessment = assessDamping(state, ringingScore);

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
    const int16_t pAdjMax = -(int16_t)ffAutotuneConfig()->p_adjust_max;  // Negative: P reduction limit

    if (state->lastDampingAssessment == FF_DAMPING_UNDERDAMPED) {
        // Underdamped: decrease P AND increase D (lower P/D ratio → more damping)
        if (state->ringLowerP == 1 || state->pAdjustment > state->ringLowerP) {
            state->ringLowerP = state->pAdjustment;
        }

        // Decrease P
        int16_t newP = state->pAdjustment - pStep;
        if (newP < pAdjMax) {
            state->pAdjustment = pAdjMax;
            state->ringBracketState = FF_BRACKET_CONVERGED;
        } else {
            state->pAdjustment = newP;
        }

        // Increase D (but respect D noise ceiling)
        int16_t effectiveD = (int16_t)currentPidProfile->pid[axis].D + state->dAdjustment;
        int8_t ceiling = getDNoiseCeiling(axis);
        if (ceiling == 0 || effectiveD + dStep <= ceiling) {
            state->dAdjustment += dStep;
            if (state->dAdjustment > (int16_t)ffAutotuneConfig()->d_adjust_max) {
                state->dAdjustment = (int16_t)ffAutotuneConfig()->d_adjust_max;
            }
        }

        runtime.gainsModified = true;

    } else if (state->lastDampingAssessment == FF_DAMPING_OVERDAMPED) {
        // Overdamped: increase P, decrease D (raise P/D ratio)
        state->pAdjustment += pStep;
        if (state->pAdjustment > (int16_t)ffAutotuneConfig()->p_adjust_max) {
            state->pAdjustment = (int16_t)ffAutotuneConfig()->p_adjust_max;
        }

        state->dAdjustment -= dStep;
        if (state->dAdjustment < -(int16_t)ffAutotuneConfig()->d_adjust_max) {
            state->dAdjustment = -(int16_t)ffAutotuneConfig()->d_adjust_max;
        }

        runtime.gainsModified = true;

    } else {
        // FF_DAMPING_GOOD
        if (state->ringUpperP == -100 || state->pAdjustment < state->ringUpperP) {
            state->ringUpperP = state->pAdjustment;
        }

        if (state->ringLowerP != 1 && state->ringUpperP != -100 &&
            state->ringLowerP > state->ringUpperP) {
            if (state->ringBracketState == FF_BRACKET_SEARCHING) {
                state->ringBracketState = FF_BRACKET_BRACKETED;
            }

            int16_t bracketWidth = state->ringLowerP - state->ringUpperP;
            if (bracketWidth <= pStep) {
                state->pAdjustment = state->ringUpperP;
                state->ringBracketState = FF_BRACKET_CONVERGED;
                runtime.gainsModified = true;
            } else {
                state->pAdjustment = (state->ringLowerP + state->ringUpperP) / 2;
                runtime.gainsModified = true;
            }
        }
    }
}

// ============================================================================
// PHASE 2B: NOISE MEASUREMENT AND P/D SCALE-DOWN WITH DIAGNOSTIC
// ============================================================================

static void noiseReset(ffAxisState_t *state)
{
    state->noiseAccumulator = 0.0f;
    state->noiseSampleCount = 0;
    state->gyroNoiseAccumulator = 0.0f;
    state->gyroNoiseSampleCount = 0;
}

// ── D-term LPF2 tool ──

static bool tryReduceDtermLpf2(void)
{
    if (runtime.dtermLpf2BaseCutoff == 0) {
        return false;  // No D-term LPF2 configured
    }

    const uint8_t step = ffAutotuneConfig()->dterm_lpf2_step;
    const uint16_t minHz = ffAutotuneConfig()->dterm_lpf2_min;

    const uint16_t currentHz = getEffectiveDtermLpf2Hz();
    if (currentHz <= minHz) {
        return false;  // Already at minimum
    }

    int16_t newAdj = runtime.dtermLpf2Adjustment - (int16_t)step;
    uint16_t newHz = (int16_t)runtime.dtermLpf2BaseCutoff + newAdj;
    if (newHz < minHz) {
        newHz = minHz;
        newAdj = (int16_t)minHz - (int16_t)runtime.dtermLpf2BaseCutoff;
    }

    runtime.dtermLpf2Adjustment = newAdj;
    applyDtermLpf2Cutoff(newHz);
    runtime.gainsModified = true;

    return true;
}

static void revertDtermLpf2Step(void)
{
    if (runtime.dtermLpf2BaseCutoff == 0) {
        return;
    }

    const uint8_t step = ffAutotuneConfig()->dterm_lpf2_step;
    runtime.dtermLpf2Adjustment += (int16_t)step;
    if (runtime.dtermLpf2Adjustment > 0) {
        runtime.dtermLpf2Adjustment = 0;
    }

    const uint16_t newHz = getEffectiveDtermLpf2Hz();
    if (newHz > 0) {
        applyDtermLpf2Cutoff(newHz);
    }
    runtime.gainsModified = true;
}

// ── Gyro LPF2 tool ──

static bool tryReduceGyroLpf2(void)
{
    if (runtime.lpf2BaseCutoff == 0) {
        return false;  // No gyro LPF2 configured
    }

    const uint8_t lpf2Step = ffAutotuneConfig()->lpf2_step;
    const uint16_t lpf2Min = ffAutotuneConfig()->lpf2_min;

    const uint16_t currentHz = getEffectiveLpf2Hz();
    if (currentHz <= lpf2Min) {
        return false;  // Already at minimum
    }

    int16_t newAdj = runtime.lpf2Adjustment - (int16_t)lpf2Step;
    uint16_t newHz = (int16_t)runtime.lpf2BaseCutoff + newAdj;
    if (newHz < lpf2Min) {
        newHz = lpf2Min;
        newAdj = (int16_t)lpf2Min - (int16_t)runtime.lpf2BaseCutoff;
    }

    runtime.lpf2Adjustment = newAdj;
    applyLpf2Cutoff(newHz);

    // Recalculate filter group delay with new LPF2
    runtime.filterGroupDelay = estimateFilterGroupDelay();
    runtime.gainsModified = true;

    return true;
}

static void revertGyroLpf2Step(void)
{
    if (runtime.lpf2BaseCutoff == 0) {
        return;
    }

    const uint8_t lpf2Step = ffAutotuneConfig()->lpf2_step;
    runtime.lpf2Adjustment += (int16_t)lpf2Step;
    if (runtime.lpf2Adjustment > 0) {
        runtime.lpf2Adjustment = 0;
    }

    const uint16_t newHz = getEffectiveLpf2Hz();
    if (newHz > 0) {
        applyLpf2Cutoff(newHz);
    }

    runtime.filterGroupDelay = estimateFilterGroupDelay();
    runtime.gainsModified = true;
}

// ── Noise source diagnosis ──

static ffNoiseSrc_e diagnoseNoiseSource(uint16_t dtermNoiseScore, uint16_t gyroNoiseScore)
{
    const uint16_t noiseFloor = ffAutotuneConfig()->noise_floor;
    const uint16_t gyroThreshold = ffAutotuneConfig()->gyro_noise_threshold;

    if (dtermNoiseScore <= noiseFloor) {
        return FF_NOISE_SRC_NONE;
    }

    if (gyroNoiseScore > gyroThreshold) {
        return FF_NOISE_SRC_FILTER;
    }

    return FF_NOISE_SRC_GAIN;
}

// ── D noise ceiling learning ──

static void recordDNoiseTrigger(ffAxisState_t *state, int axis)
{
    int16_t effectiveD = (int16_t)currentPidProfile->pid[axis].D + state->dAdjustment;

    if (ABS(effectiveD - state->lastNoiseDLevel) <= 3) {
        state->dNoiseTriggerCount++;
    } else {
        state->dNoiseTriggerCount = 1;
    }
    state->lastNoiseDLevel = effectiveD;
    state->maneuversSinceNoise = 0;

    // After 3 triggers at similar D level: establish ceiling
    if (state->dNoiseTriggerCount >= 3) {
        int8_t ceiling = (int8_t)(effectiveD - 2);
        if (axis == FD_ROLL) {
            ffAutotuneConfigMutable()->d_noise_ceiling_roll = ceiling;
        } else {
            ffAutotuneConfigMutable()->d_noise_ceiling_pitch = ceiling;
        }
        runtime.gainsModified = true;
    }
}

// ── Tool application based on diagnosis ──

static void applyNoiseReduction(ffAxisState_t *state, int axis, ffNoiseSrc_e src)
{
    const uint8_t gainScaleStep = ffAutotuneConfig()->gain_scale_step;
    const uint8_t gainScaleMin = ffAutotuneConfig()->gain_scale_min;

    if (src == FF_NOISE_SRC_FILTER) {
        // Filter noise: D-term LPF2 first, then gyro LPF2, then gain scale
        if (tryReduceDtermLpf2()) {
            state->lastNoiseTool = NOISE_TOOL_DTERM_LPF2;
            return;
        }
        if (tryReduceGyroLpf2()) {
            state->lastNoiseTool = NOISE_TOOL_GYRO_LPF2;
            return;
        }
    }

    // Gain noise (or filter tools exhausted): ratio-preserving gain scale-down
    if (state->gainScalePercent > gainScaleMin) {
        state->gainScalePercent -= gainScaleStep;
        if (state->gainScalePercent < gainScaleMin) {
            state->gainScalePercent = gainScaleMin;
        }
        recordDNoiseTrigger(state, axis);
        state->lastNoiseTool = NOISE_TOOL_SCALE;
        runtime.gainsModified = true;
    } else {
        state->lastNoiseTool = NOISE_TOOL_NONE;
    }
}

static void revertLastNoiseTool(ffAxisState_t *state)
{
    switch (state->lastNoiseTool) {
        case NOISE_TOOL_DTERM_LPF2:
            revertDtermLpf2Step();
            break;
        case NOISE_TOOL_GYRO_LPF2:
            revertGyroLpf2Step();
            break;
        case NOISE_TOOL_SCALE: {
            const uint8_t gainScaleStep = ffAutotuneConfig()->gain_scale_step;
            state->gainScalePercent += gainScaleStep;
            if (state->gainScalePercent > 100) {
                state->gainScalePercent = 100;
            }
            runtime.gainsModified = true;
            break;
        }
        default:
            break;
    }
    state->lastNoiseTool = NOISE_TOOL_NONE;
}

// ── Main Phase 2b processing ──

static void processPhase2bNoise(ffAxisState_t *state, int axis)
{
    // Compute D-term noise score
    uint16_t noiseScore = 0;
    if (state->noiseSampleCount >= 3) {
        float avgNoise = state->noiseAccumulator / (float)state->noiseSampleCount;
        noiseScore = (uint16_t)MIN(avgNoise * 10.0f, 65535);
    }

    // Compute gyro noise score
    uint16_t gyroNoiseScore = 0;
    if (state->gyroNoiseSampleCount >= 3) {
        float avgGyroNoise = state->gyroNoiseAccumulator / (float)state->gyroNoiseSampleCount;
        gyroNoiseScore = (uint16_t)MIN(avgGyroNoise * 10.0f, 65535);
    }

    noiseReset(state);
    state->lastNoiseScore = noiseScore;
    state->lastGyroNoiseScore = gyroNoiseScore;

    // Diagnose noise source
    ffNoiseSrc_e src = diagnoseNoiseSource(noiseScore, gyroNoiseScore);
    state->lastNoiseSrc = src;

    // Below noise floor — noise is acceptable
    if (src == FF_NOISE_SRC_NONE) {
        return;
    }

    if (state->noiseBaseline == 0) {
        // First measurement above floor: establish baseline, apply first tool
        state->noiseBaseline = noiseScore;
        applyNoiseReduction(state, axis, src);
        return;
    }

    // Compare noise to baseline
    const uint8_t noiseThreshold = ffAutotuneConfig()->noise_threshold;

    uint16_t improvement = 0;
    if (noiseScore < state->noiseBaseline) {
        improvement = ((state->noiseBaseline - noiseScore) * 100) / state->noiseBaseline;
    }

    if (noiseScore > state->noiseBaseline) {
        // Noise increased — revert last adjustment
        revertLastNoiseTool(state);
    } else if (improvement >= noiseThreshold) {
        // Noise improving — update baseline and apply another step
        state->noiseBaseline = noiseScore;
        applyNoiseReduction(state, axis, src);
    }
    // else: improvement below threshold — converged, no action
}

// ============================================================================
// CONCURRENT ASSESSMENT: PRIORITY DECISION
// ============================================================================

static ffAutotunePhase_e priorityDecision(ffAxisState_t *state)
{
    const ffMetricSnapshot_t *m = &state->lastMetrics;
    if (!m->valid) {
        return state->phase;
    }

    const uint16_t noiseFloor = ffAutotuneConfig()->noise_floor;
    const int16_t deadband = ffAutotuneConfig()->error_deadband * 10;
    const uint8_t ringThreshold = ffAutotuneConfig()->ring_threshold;

    // Priority 0: Catastrophic noise — PID loop unstable, tracking error is unreliable.
    if (m->noiseScore > noiseFloor * 2) {
        // Exponential gain scale reduction
        state->gainScalePercent = state->gainScalePercent / 2;
        if (state->gainScalePercent < ffAutotuneConfig()->gain_scale_min) {
            state->gainScalePercent = ffAutotuneConfig()->gain_scale_min;
        }
        runtime.gainsModified = true;
        return FF_AUTOTUNE_PHASE2B_GAIN_NOISE;
    }

    int16_t trackSeverity_x10 = (ABS(m->trackingError) * 10) / MAX(deadband, 1);
    int16_t ringSeverity_x10 = (m->ringingScore * 10) / MAX(ringThreshold, 1);

    // Priority 0.5: Catastrophic ringing with gainScale < 100 — gains were over-reduced.
    if (ringSeverity_x10 > 20 && state->gainScalePercent < 100) {
        state->gainScalePercent = (100 + state->gainScalePercent) / 2;
        runtime.gainsModified = true;
        return FF_AUTOTUNE_PHASE2B_GAIN_NOISE;
    }

    // Priority 1: Severe tracking error (>2x deadband) -> fix F first
    if (trackSeverity_x10 > 20) {
        return FF_AUTOTUNE_PHASE1_FF;
    }

    // Priority 2: Underdamped (ringing > threshold)
    if (m->dampingAssessment == FF_DAMPING_UNDERDAMPED) {
        return FF_AUTOTUNE_PHASE2_UNDERDAMPED;
    }

    // Priority 2.5: Overdamped
    if (m->dampingAssessment == FF_DAMPING_OVERDAMPED) {
        return FF_AUTOTUNE_PHASE2_OVERDAMPED;
    }

    // Priority 3: Noise above absolute floor
    if (m->noiseScore > noiseFloor) {
        return FF_AUTOTUNE_PHASE2B_GAIN_NOISE;
    }

    // Priority 4: Mild tracking error (>1x deadband)
    if (trackSeverity_x10 > 10) {
        return FF_AUTOTUNE_PHASE1_FF;
    }

    // All within deadbands
    return FF_AUTOTUNE_COMPLETE;
}

static void checkConvergence(ffAxisState_t *state, int axis)
{
    const ffMetricSnapshot_t *m = &state->lastMetrics;
    if (!m->valid) {
        return;
    }

    const int16_t deadband = ffAutotuneConfig()->error_deadband * 10;
    const uint8_t ringThreshold = ffAutotuneConfig()->ring_threshold;

    bool trackingOk = ABS(m->trackingError) <= deadband;
    bool ringingOk = m->ringingScore <= ringThreshold;
    bool noiseOk = m->noiseScore <= ffAutotuneConfig()->noise_floor;

    if (noiseOk) {
        state->maneuversSinceNoise++;
        // D ceiling reassessment: after 20 quiet maneuvers, bump ceiling up by d_step
        int8_t ceiling = getDNoiseCeiling(axis);
        if (ceiling > 0 && state->maneuversSinceNoise >= 20) {
            ceiling += ffAutotuneConfig()->d_step;
            if (axis == FD_ROLL) {
                ffAutotuneConfigMutable()->d_noise_ceiling_roll = ceiling;
            } else {
                ffAutotuneConfigMutable()->d_noise_ceiling_pitch = ceiling;
            }
            state->dNoiseTriggerCount = 0;
            state->maneuversSinceNoise = 0;
            runtime.gainsModified = true;
        }
    }

    if (trackingOk && ringingOk && noiseOk) {
        state->convergenceCount++;
    } else {
        state->convergenceCount = 0;
    }

    if (state->convergenceCount >= FF_CONVERGENCE_COUNT) {
        state->phase = FF_AUTOTUNE_COMPLETE;
        state->wigglePending = true;
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
        state->expectedLagAccumulator = 0.0f;
        state->sampleCount = 0;
        noiseReset(state);
        return;
    }

    // ── Step 1: Compute tracking error with filter delay compensation ──
    const float avgErrorRaw = state->errorAccumulator / (float)state->sampleCount;
    // Expected lag from filter group delay: positive value representing expected |gyro| deficit
    const float avgExpectedLag = state->expectedLagAccumulator / (float)state->sampleCount;
    // Compensate: raw error is negative for lag, expected lag shifts target so lag is expected
    const float avgErrorCompensated = avgErrorRaw + avgExpectedLag;
    const int16_t avgError = lrintf(avgErrorCompensated * 10.0f);
    const int16_t deadband = ffAutotuneConfig()->error_deadband * 10;

    state->lastAvgError = avgError;
    state->lastAssessment = classifyError(avgError, deadband);

    // ── Step 2: Store all metrics in snapshot ──
    state->lastMetrics.trackingError = avgError;
    state->lastMetrics.trackingAssessment = state->lastAssessment;
    state->lastMetrics.ringingScore = state->lastRingingScore;
    state->lastMetrics.dampingAssessment = state->lastDampingAssessment;
    state->lastMetrics.noiseScore = state->lastNoiseScore;
    state->lastMetrics.valid = true;

    // Priority 0: Catastrophic noise overrides bootstrap — PID loop unstable,
    // tuning F is pointless when gains are this far off
    if (!state->bootstrapComplete && state->lastNoiseScore > ffAutotuneConfig()->noise_floor * 2) {
        state->bootstrapComplete = true;
        state->noiseBaseline = state->lastNoiseScore;
    }

    // ── Step 3: Bootstrap check ──
    if (!state->bootstrapComplete) {
        // During bootstrap, only do Phase 1 F-adjustment
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
            // F-term converged, bootstrap complete
            state->bootstrapComplete = true;
            // Establish noise baseline from this maneuver
            state->noiseBaseline = state->lastNoiseScore;
        }
    } else {
        // ── Step 4: Concurrent mode — priority-based adjustment ──
        ffAutotunePhase_e nextPhase = priorityDecision(state);

        // Apply adjustment for the chosen phase
        switch (nextPhase) {
            case FF_AUTOTUNE_PHASE1_FF:
                // F-term drifted — adjust like Phase 1
                // In concurrent mode, always allow gain refinement even after
                // bracket convergence. New maneuvers provide information that
                // can shift or narrow the bracket further.
                state->phase = FF_AUTOTUNE_PHASE1_FF;
                addHistoryEntry(state, state->gain, avgError, state->sampleCount);
                {
                    uint8_t nextGain = calculateNextGain(state, avgError);
                    if (nextGain != state->gain) {
                        state->gain = nextGain;
                        state->adjustmentCount++;
                        runtime.gainsModified = true;
                    }
                }
                break;

            case FF_AUTOTUNE_PHASE2_UNDERDAMPED:
                state->phase = FF_AUTOTUNE_PHASE2_UNDERDAMPED;
                break;

            case FF_AUTOTUNE_PHASE2_OVERDAMPED:
                state->phase = FF_AUTOTUNE_PHASE2_OVERDAMPED;
                break;

            case FF_AUTOTUNE_PHASE2B_GAIN_NOISE:
                state->phase = FF_AUTOTUNE_PHASE2B_GAIN_NOISE;
                break;

            case FF_AUTOTUNE_COMPLETE:
                state->phase = FF_AUTOTUNE_COMPLETE;
                break;
        }

        // ── Step 5: Check convergence ──
        checkConvergence(state, axis);
    }

    state->maneuverCount++;

    // Reset accumulators for next maneuver (state transition handled by caller)
    state->errorAccumulator = 0.0f;
    state->sampleCount = 0;
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

    // Update COMPLETE notification wiggle
    if (state->wiggleActive) {
        const timeDelta_t elapsed = cmpTimeUs(currentTimeUs, state->wiggleStartTime);
        if (elapsed >= (timeDelta_t)FF_WIGGLE_DURATION_US) {
            state->wiggleActive = false;
            state->wiggleOffset = 0.0f;
        } else {
            // 1.5-cycle sine: -A*sin(3*pi*t/T) gives pattern 0→-A→+A→-A→0
            const float phase = (float)elapsed / (float)FF_WIGGLE_DURATION_US;
            state->wiggleOffset = -FF_WIGGLE_AMPLITUDE * sin_approx(3.0f * M_PIf * phase);
        }
    }

    // Update ringing window if active (unconditional — concurrent measurement)
    if (state->ringWindowActive) {
        const uint32_t ringWindowUs = (uint32_t)ffAutotuneConfig()->ring_window_ms * 1000;
        if (cmpTimeUs(currentTimeUs, state->ringWindowStartTime) >= (timeDelta_t)ringWindowUs) {
            // Window duration expired — always process ringing
            processDamping(state, axis);
        } else {
            ringWindowAccumulate(state, signedError);
        }
    }

    // Simple state machine:
    // IDLE -> RISING -> ADJUSTING -> WAITING -> IDLE
    switch (state->windowState) {
        case FF_WINDOW_IDLE:
            // Start COMPLETE notification wiggle after delay
            if (state->wigglePending && !state->wiggleActive) {
                if (cmpTimeUs(currentTimeUs, state->idleEntryTime) >= (timeDelta_t)FF_WIGGLE_DELAY_US) {
                    state->wiggleActive = true;
                    state->wigglePending = false;
                    state->wiggleStartTime = currentTimeUs;
                    state->wiggleOffset = 0.0f;
                }
            }
            // IDLE: waiting for rise - neutral setpoint and low accel
            // Transition to RISING when setpoint enters window with increasing magnitude
            // Require minimum dwell time in IDLE to avoid catching recovery from prior maneuver
            // Don't start RISING during notification wiggle
            if (inSetpointWindow && magnitudeIncreasing && hasSignificantAccel
                && !state->wiggleActive
                && cmpTimeUs(currentTimeUs, state->idleEntryTime) >= FF_AUTOTUNE_MIN_IDLE_US) {
                state->windowState = FF_WINDOW_RISING;
                state->errorAccumulator = trackingError;
                // setpointDelta from getFeedforward() is already in deg/s² (angular accel)
                // Expected lag in deg/s = |angular_accel| × filter_delay_seconds
                state->expectedLagAccumulator = fabsf(setpointDelta) * runtime.filterGroupDelay;
                state->sampleCount = 1;
                // Always reset noise accumulators at start of rise (concurrent measurement)
                noiseReset(state);
                state->noiseAccumulator = fabsf(pidData[axis].D);
                state->noiseSampleCount = 1;
                state->gyroNoiseAccumulator = fabsf(gyro.gyroADC[axis] - gyro.gyroADCf[axis]);
                state->gyroNoiseSampleCount = 1;
            }
            break;

        case FF_WINDOW_RISING:
            // RISING: setpoint in range, accel away from center - MEASURE HERE
            // Stay in RISING as long as magnitude is increasing with significant accel
            if (magnitudeIncreasing && hasSignificantAccel) {
                // Still rising - accumulate tracking error and expected filter delay lag
                state->errorAccumulator += trackingError;
                state->expectedLagAccumulator += fabsf(setpointDelta) * runtime.filterGroupDelay;
                state->sampleCount++;
                // Always accumulate |D-term| noise and gyro noise during the rise (concurrent)
                state->noiseAccumulator += fabsf(pidData[axis].D);
                state->noiseSampleCount++;
                state->gyroNoiseAccumulator += fabsf(gyro.gyroADC[axis] - gyro.gyroADCf[axis]);
                state->gyroNoiseSampleCount++;
            } else {
                // No longer rising (accel dropped or direction changed) -> ADJUSTING
                state->windowState = FF_WINDOW_ADJUSTING;
                state->maneuverEndTime = currentTimeUs;

                // Always open ringing analysis window at end of rise (concurrent)
                if (!state->ringWindowActive) {
                    ringWindowOpen(state, signedError, currentTimeUs);
                }
                // Always finalize noise measured during the rise (concurrent)
                if (state->noiseSampleCount >= 3) {
                    processPhase2bNoise(state, axis);
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
                state->idleEntryTime = currentTimeUs;
                // Close ringing window if still open (always process — concurrent)
                if (state->ringWindowActive) {
                    processDamping(state, axis);
                }
            }
            break;
    }

    state->prevAccel = absAccel;
    state->prevSetpoint = setpoint;

    // Debug output: always-visible, not phase-multiplexed
    // ch0: Roll P+D+F adj packed | ch1: Pitch P+D+F adj packed
    // ch2: gainScale*100 + dtermLpf2Adj+50 | ch3: phase*10 + windowState
    // ch4: tracking error (x10) | ch5: damping*10000 + ringing/overdamped score
    // ch6: noise score (D-term) | ch7: dCeiling*100 + convergenceCount
    if (axis == gyro.gyroDebugAxis) {
        // Pack P+D+F adjustments per axis into single int16
        // Encoding: (pAdj+20)*525 + (dAdj+10)*25 + (fAdj/8+12)
        // Max: 40*525 + 20*25 + 24 = 21524, fits int16
        for (int i = 0; i < 2; i++) {
            ffAxisState_t *s = &runtime.axis[i];
            int16_t pEnc = constrain(s->pAdjustment, -20, 20) + 20;
            int16_t dEnc = constrain(s->dAdjustment, -10, 10) + 10;
            int16_t fAdj = (int16_t)s->gain - (int16_t)currentPidProfile->pid[i].F;
            int16_t fEnc = constrain(fAdj / 8, -12, 12) + 12;
            DEBUG_SET(DEBUG_FF_AUTOTUNE, i, pEnc * 525 + dEnc * 25 + fEnc);
        }

        DEBUG_SET(DEBUG_FF_AUTOTUNE, 2,
            (int16_t)state->gainScalePercent * 100 + constrain(runtime.dtermLpf2Adjustment + 50, 0, 99));
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 3, state->phase * 10 + state->windowState);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 4, state->lastMetrics.valid ? state->lastMetrics.trackingError : lrintf(trackingError * 10.0f));
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 5,
            (int16_t)state->lastDampingAssessment * 10000 + (int16_t)MIN(state->lastRingingScore, 9999));
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 6, state->lastNoiseScore);
        DEBUG_SET(DEBUG_FF_AUTOTUNE, 7,
            getDNoiseCeiling(axis) * 100 + MIN(state->convergenceCount, 99));
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
        // Just activated — all overrides become active (additive, safe at 0)
        runtime.active = true;
        runtime.gainsLearned = true;
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
