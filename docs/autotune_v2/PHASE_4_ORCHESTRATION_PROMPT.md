# Phase 4 Orchestration Prompt

**Copy this entire prompt to a new chat session to execute Phase 4 implementation.**

---

## System Context

You are an orchestrator agent implementing Phase 4 (PD Scale Up & F Tune) for Betaflight Autotune V2. You will use subagents to complete individual tasks while tracking overall progress.

### Project Location
- **Workspace:** `c:\Users\pmant\source\repos\betaflight\betaflight`
- **Branch:** `autotune-v2-clean`
- **Build command:** `make CONFIG=BETAFPVG473`
- **Target:** STM32G4 flight controller

### Completed Phases
- ✅ Phase 0: Foundation (types, skeletons, PG, debug)
- ✅ Phase 1: Filter Characterization (hover lock, throttle sweep, noise confirm)
- ✅ Phase 2: Event Detection (stick deflection, quality gates, metrics)
- ✅ Phase 3: PD Ratio Seek (bracket+Newton algorithm for P gain)

### Key Files
- `src/main/flight/autotune_v2/autotune_types.h` - Type definitions
- `src/main/flight/autotune_v2/autotune_core.c` - Main state machine
- `src/main/flight/autotune_v2/autotune_metrics.c` - Metrics computation
- `src/main/flight/autotune_v2/autotune_rollback.c` - Rollback system
- `src/main/flight/autotune_v2/autotune_feedback.c` - Pilot feedback
- `src/main/flight/autotune_v2/autotune_debug.h` - Reason codes
- `docs/autotune_v2/phases/PHASE_4_PD_SCALE_UP.md` - Detailed spec

---

## Phase 4 Overview

Phase 4 implements three substates:
1. **PD_SCALE_UP** - Scale P and D together while maintaining ratio from Phase 3
2. **F_TUNE** - Tune feedforward to reduce lag (if enabled)
3. **PD_RETUNE_AFTER_F** - Quick validation that P/D is still good after F changes

After tuning one axis, loop to next axis (Roll → Pitch → Yaw) until all complete.

---

## Task Breakdown

Execute these tasks in order using subagents:

### Task 4.1: Add Scale History Type
**File:** `autotune_types.h`

Add after `bracketState_t`:
```c
// ============================================================================
// Scale History (for PD Scale Up Newton estimation)
// ============================================================================

#define MAX_SCALE_HISTORY 4

typedef struct {
    float scale[MAX_SCALE_HISTORY];     // Scale factor at each step
    float lag[MAX_SCALE_HISTORY];       // Lag measured at each scale
    uint8_t count;                      // Number of entries
} scaleHistory_t;

// ============================================================================
// F Tune History
// ============================================================================

#define MAX_F_HISTORY 3

typedef struct {
    float fValues[MAX_F_HISTORY];       // F value at each step
    float lagValues[MAX_F_HISTORY];     // Lag measured at each F
    uint8_t count;                      // Number of entries
} fHistory_t;
```

Add these fields to `axisTuneState_t`:
```c
    float ratioSeekP;                   // P value at end of ratio seek
    float ratioSeekD;                   // D value at end of ratio seek
    float currentScale;                 // Current scale factor (1.0 = ratio seek values)
    scaleHistory_t scaleHistory;        // History for Newton estimation
    fHistory_t fHistory;                // History for F tune
    float preFTuneOvershoot;            // Overshoot before F tuning
    float preFTuneLag;                  // Lag before F tuning
    bool pdRetuneNeeded;                // Flag for PD retune after F
```

---

### Task 4.2: Update statePdScaleUpEnter()
**File:** `autotune_core.c`

Modify the existing `statePdScaleUpEnter()` function to:
1. Capture `ratioSeekP` and `ratioSeekD` from current values
2. Calculate and store `pdRatio = P / D`
3. Initialize `currentScale = 1.0f`
4. Clear `scaleHistory` (count = 0)
5. Initialize D state for rollback tracking
6. Set `lagTargetMs` based on aggressiveness: `100.0f - (aggressiveness * 75.0f)`

---

### Task 4.3: Implement pdScaleUpDecision()
**File:** `autotune_core.c`

Implement the Newton-primary scale up algorithm (place before `statePdScaleUpEnter`):
```c
static autotuneDecision_e pdScaleUpDecision(
    const eventMetrics_t *metrics,
    axisTuneState_t *axisState,
    float lagTargetMs,
    float *newScale)
{
    // 1. SAFETY: If hasRebound, rollback to previous scale (or 1.0)
    // 2. Record current scale/lag in history (shift if full)
    // 3. OVERSHOOT CHECK: If overshoot > target * 1.5, rollback
    // 4. CONVERGENCE: If lag <= lagTargetMs, return ADVANCE
    // 5. GAIN LIMIT: If P >= AUTOTUNE_GAIN_MAX, return ADVANCE
    // 6. NEWTON: If history >= 2 points, estimate d(lag)/d(scale) and compute Newton step
    //    - Clamp step to 0.8x-1.3x of current scale
    //    - Clamp to minimum 1.0 (never go below ratio seek values)
    // 7. FALLBACK: Use fixed step from rollback system
}
```

Also implement helper:
```c
static void applyScale(axisTuneState_t *axis, float scale)
{
    // Apply scale to both P and D
    float newP = axis->ratioSeekP * scale;
    float newD = axis->ratioSeekD * scale;
    
    // Clamp and apply
    newP = constrainf(newP, AUTOTUNE_GAIN_MIN, AUTOTUNE_GAIN_MAX);
    newD = constrainf(newD, AUTOTUNE_GAIN_MIN, AUTOTUNE_GAIN_MAX);
    
    currentPidProfile->pid[runtime.currentAxisIndex].P = lrintf(newP);
    currentPidProfile->pid[runtime.currentAxisIndex].D = lrintf(newD);
    
    axis->pState.currentValue = newP;
    axis->dState.currentValue = newD;
    axis->currentScale = scale;
}
```

---

### Task 4.4: Update statePdScaleUpUpdate()
**File:** `autotune_core.c`

Wire up the decision function in `statePdScaleUpUpdate()`:
- In `AXIS_SUBSTATE_ANALYZING`:
  1. After computing metrics, call `pdScaleUpDecision()`
  2. If INCREASE/DECREASE: call `applyScale()`, save rollback state
  3. If ADVANCE: transition to `AUTOTUNE_STATE_F_TUNE` (or skip if disabled)
  4. If ROLLBACK: call `applyScale()` with previous scale

Check `autotuneConfig()->tuneFeedforward` to decide whether to enter F_TUNE or skip to next axis.

---

### Task 4.5: Implement stateFTuneEnter()
**File:** `autotune_core.c`

Modify existing stub:
1. Save `preFTuneOvershoot` and `preFTuneLag` from lastMetrics
2. Initialize F rollback state from `currentPidProfile->pid[axis].F`
3. Clear `fHistory` (count = 0)
4. Set substate to `AXIS_SUBSTATE_WAIT_EVENT`

---

### Task 4.6: Implement fTuneDecision()
**File:** `autotune_core.c`

Implement cautious Newton for F tuning:
```c
#define F_MAX 200
#define F_MAX_STEP_PCT 0.15f  // Max 15% per step

static autotuneDecision_e fTuneDecision(
    const eventMetrics_t *metrics,
    axisTuneState_t *axisState,
    float lagTargetMs,
    float *newF)
{
    // 1. SAFETY: If overshoot > target * 1.3, rollback to previous F
    // 2. Record current F/lag in history
    // 3. CONVERGENCE: If lag <= lagTargetMs, return ADVANCE
    // 4. LIMIT: If F >= F_MAX, return ADVANCE
    // 5. CAUTIOUS NEWTON: Only if history >= 2 AND trend is clear (dF > 0 && dLag < 0)
    //    - Cap step to F_MAX_STEP_PCT of current F
    //    - Ensure increase only (never decrease F during tune)
    // 6. FALLBACK: Fixed step (10% or min 10)
}
```

---

### Task 4.7: Implement stateFTuneUpdate()
**File:** `autotune_core.c`

Wire up F tune decision:
- In `AXIS_SUBSTATE_ANALYZING`:
  1. After metrics, call `fTuneDecision()`
  2. If INCREASE: apply F value, save rollback
  3. If ADVANCE: check if PD retune needed, transition appropriately
  4. If ROLLBACK: apply previous F value

---

### Task 4.8: Implement statePdRetuneAfterFEnter() and Update()
**File:** `autotune_core.c`

Quick validation phase:
- **Enter:** Compare current overshoot to `preFTuneOvershoot`. If > 3% higher, set `pdRetuneNeeded = true`
- **Update:** Process 2-3 events max. If overshoot acceptable (< target * 1.2), advance. If rebound, slight P decrease.

---

### Task 4.9: Implement advanceToNextAxisOrComplete()
**File:** `autotune_core.c`

Axis loop logic:
```c
static void advanceToNextAxisOrComplete(timeUs_t currentTimeUs)
{
    // Mark current axis complete
    runtime.axisState[runtime.currentAxisIndex].complete = true;
    
    // Feedback: 1 wiggle per axis complete
    autotuneFeedbackWiggle(1);
    
    // Find next enabled axis
    for (int i = runtime.currentAxisIndex + 1; i < AUTOTUNE_AXIS_COUNT; i++) {
        if (autotuneConfig()->axes & (1 << i)) {
            runtime.currentAxisIndex = i;
            transitionToState(AUTOTUNE_STATE_PD_RATIO_SEEK, currentTimeUs);
            return;
        }
    }
    
    // All axes complete!
    transitionToState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
    autotuneFeedbackWiggle(4);  // 4 wiggles = all done
}
```

---

### Task 4.10: Build and Verify
Run `make CONFIG=BETAFPVG473` and fix any compile errors.

---

### Task 4.11: Update PROGRESS.md
Mark Phase 4 complete and update status table.

---

## Existing Infrastructure Reference

### State Transition
```c
static void transitionToState(autotuneState_e newState, timeUs_t currentTimeUs);
```

### Rollback API
```c
void autotuneRollbackInit(parameterState_t *param, float initialValue, float stepSize);
void autotuneRollbackSaveState(parameterState_t *param);
void autotuneRollbackReportGood(parameterState_t *param);
void autotuneRollbackReportBad(parameterState_t *param);
float autotuneRollbackGetStepSize(parameterState_t *param, float baseStep);
```

### Feedback API
```c
void autotuneFeedbackWiggle(uint8_t count);
```

### Config Access
```c
autotuneConfig()->tuneFeedforward  // bool
autotuneConfig()->axes             // bitmask
autotuneConfig()->pStepPercent     // uint8_t
runtime.aggressiveness             // float 0.0-1.0
runtime.targetOvershootPct         // float
runtime.lagTargetMs                // float
```

### Debug Output
```c
AUTOTUNE_DEBUG_SET(channel, value)
// Channels: DEBUG_STATE, DEBUG_AXIS, DEBUG_REASON, DEBUG_OVERSHOOT, DEBUG_GAIN_P, DEBUG_GAIN_D
```

### Reason Codes (already defined)
- `AUTOTUNE_REASON_PHASE_COMPLETE`
- `AUTOTUNE_REASON_AXIS_COMPLETE`  
- `AUTOTUNE_REASON_GAIN_LIMIT_MAX`
- `AUTOTUNE_REASON_ROLLBACK_OSCILLATION`

---

## Execution Instructions

1. **Start with Task 4.1** - Add types first as other tasks depend on them
2. **Build after each task** to catch errors early
3. **Use subagents** for each task with specific file/function context
4. **Update todo list** after completing each task
5. **Final build** must pass with no errors
6. **Update PROGRESS.md** as final step

## Subagent Prompt Template

When invoking subagents, include:
1. The specific task number and description
2. The exact file path
3. Any relevant existing code context
4. The expected function signature or struct definition
5. Integration points (what calls it, what it calls)

---

## Success Criteria

- [ ] `scaleHistory_t` and `fHistory_t` types added
- [ ] `axisTuneState_t` has new fields for scale tracking
- [ ] `pdScaleUpDecision()` implements Newton-primary scaling
- [ ] `fTuneDecision()` implements cautious Newton for F
- [ ] `PD_RETUNE_AFTER_F` quick validates P/D after F changes
- [ ] Axis loop works: Roll → Pitch → Yaw → Complete
- [ ] 1 wiggle per axis complete, 4 wiggles for all done
- [ ] Build passes cleanly for BETAFPVG473
- [ ] PROGRESS.md shows Phase 4 complete

---

*End of Phase 4 Orchestration Prompt*
