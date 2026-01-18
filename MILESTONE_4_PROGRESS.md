# Milestone 4 Progress Tracking

## Status: IN PROGRESS - FIXES REQUIRED

**Last Updated:** 2026-01-17  
**Roadmap:** [MILESTONE_4_ROADMAP.md](MILESTONE_4_ROADMAP.md)

---

## ⚠️ Critical Issue Discovered (Flight Test Analysis)

**Log:** `btfl__mk4___subagent.bbl.csv`

### Problem: PID Tuning is F-Biased
Analysis of flight log shows:
- **F gain changed 21 times** (bouncing between increases/decreases)
- **P gain changed only ONCE** (+6 at t=62s)
- **D gain never changed** (0 adjustments)
- **I gain never changed** (0 adjustments)

### Root Cause:
In `autotune_analysis.c`, the attribution logic checks F-term issues (Phase 2) BEFORE P/D response classification (Phase 3). Any `velocityWeightedLag > 5.0` during fast stick movement immediately triggers F attribution, preventing P/D tuning.

### Fix Required: **P0 - Fix Attribution Order**
See new P0 task below.

---

## Feature Progress

| Task | Feature | Status | Assignee | Notes |
|------|---------|--------|----------|-------|
| **P0** | **Fix F-Bias in Attribution** | ✅ COMPLETE | Subagent | P/D now evaluated before F |
| P1 | Multi-Variable DOE | ✅ PARTIAL | Subagent | Hover mode only, needs PID mode |
| P2 | Bidirectional Exploration | ✅ COMPLETE | Subagent | Relaxation phases added |
| P3 | Critical Damping Detection | ✅ COMPLETE | Subagent | Damping ratio calculation added |
| P4 | P/D Push-Up Logic | ❌ NOT STARTED | - | Depends on P3 ✅ |
| P5 | Integrated Filter+PID | ❌ NOT STARTED | - | Depends on P1 |
| P6 | Notch Configuration | ❌ NOT STARTED | - | |

---

## P0: Fix Attribution Order ✅ COMPLETE
**Implemented:** 2026-01-17  
**Build Status:** PASS

### Changes Made:
**File: `src/main/flight/autotune_analysis.c`**

Reordered `autotuneAttributeGains()` function:
1. **Phase 0:** Check oscillation first (skip I-term drift if oscillating)
2. **Phase 1:** I-term issues (unchanged - bounceback, slow oscillation)
3. **Phase 2:** P/D response class (MOVED UP - was Phase 3)
   - UNDERDAMPED → P/D attribution, then **return** (no F check)
   - OVERDAMPED → P/D attribution, then **return** (no F check)
   - NOISY → D/filter attribution, then **return**
   - EXCELLENT/CRITICAL → **fall through** to F-term checks
4. **Phase 3:** F-term optimization (MOVED DOWN - only when P/D are good)
5. **Phase 4:** Fine-tuning for excellent response

### Key Fix:
Added `return` statements after UNDERDAMPED, OVERDAMPED, and NOISY cases to prevent falling through to F-term attribution.

### Acceptance Criteria:
- [x] P/D attributed for UNDERDAMPED/OVERDAMPED responses
- [x] F attributed only when P/D response is already EXCELLENT or CRITICAL
- [x] Build passes
- [ ] Flight test shows balanced P/D/F adjustments (pending)

---

## P1: Multi-Variable DOE ✅ PARTIAL
**Implemented:** 2026-01-17  
**Build Status:** PASS

### What's Working:
- `applyMultiVariableFix()` function exists in autotune.c
- Used in HOVER mode diagnostic analyzing phase
- Sensitivities calculated and applied with 90% damping

### What's Missing:
- Multi-variable approach NOT used in PID tuning mode (ROLL/PITCH)
- PID mode still uses single-parameter `autotuneApplyGainAdjustment()`
- Types (`multiVariableResult_t`, etc.) not added to autotune_types.h

### Status: PARTIAL - Works for Hover, not for PID tuning

---

## P2: Bidirectional Exploration ✅ COMPLETE
**Implemented:** 2026-01-17  
**Build Status:** PASS

### Changes Made:
1. **autotune_types.h:**
   - Added relaxation phases to `hoverDiagPhase_e` enum:
     - `HOVER_DIAG_RELAX_ROLL` (test 1.5× roll gains)
     - `HOVER_DIAG_RELAX_PITCH` (test 1.5× pitch gains)  
     - `HOVER_DIAG_RELAX_GYRO_LPF1` (test +50Hz)
     - `HOVER_DIAG_RELAX_DTERM_LPF1` (test +25Hz)

2. **autotune.c:**
   - Added `shouldAttemptRelaxation()` - checks if baseline RMS < 8.0
   - Added `applyRelaxationTest()` - applies 1.5× gains or +Hz filters
   - Added `evaluateRelaxationResult()` - keeps changes if noise stays < 15
   - State machine transitions to relaxation phases after standard DOE
   - Uses reason codes 1140-1143 for relaxation actions

### Acceptance Criteria:
- [x] Relaxation phases added to enum
- [x] Triggered when baseline RMS < MOTOR_RMS_EXCELLENT
- [x] Tests 1.5× gain increases
- [x] Tests filter frequency increases (+50Hz gyro, +25Hz dterm)
- [x] Keeps changes only if noise stays acceptable (< 15)
- [x] Reason codes output for debug

---

## P3: Critical Damping Detection ✅ COMPLETE
**Implemented:** 2026-01-17  
**Build Status:** PASS

### Changes Made:
1. **autotune_types.h:**
   - Added `dampingRatio` field to `autotuneMetrics_t` (line 309)
   - Added `RESPONSE_CRITICAL` to response classification enum

2. **autotune_analysis.c:**
   - Added `calculateDampingFromOvershoot(float overshootPercent)` (line 39):
     ```c
     if (overshootPercent <= 0) return 1.0f;  // Critically damped
     float os = overshootPercent / 100.0f;
     float lnOs = logf(os);
     return -lnOs / sqrtf(M_PI * M_PI + lnOs * lnOs);
     ```
   - Integrated into `autotuneAnalyzeResponse()` (line 243) - calculates and stores damping ratio
   - Updated `autotuneClassifyResponse()` to use damping ratio thresholds

3. **Response Classification Updated:**
   - ζ < 0.5: `RESPONSE_UNDERDAMPED` (bouncy)
   - ζ 0.5-0.7: `RESPONSE_CRITICAL` (snappy, slight overshoot OK)
   - ζ 0.7-1.0: `RESPONSE_EXCELLENT` (optimal)
   - ζ > 1.0: `RESPONSE_OVERDAMPED` (sluggish)

### Acceptance Criteria:
- [x] Damping ratio calculated from overshoot percentage
- [x] Standard control theory formula implemented
- [x] `dampingRatio` field added to metrics struct
- [x] Response classification uses damping ratio
- [x] Foundation ready for P4 (P/D Push-Up Logic)

---

## Infrastructure (Complete)  
**Priority:** P1 (Critical)  
**Files:** `src/main/flight/autotune.c`

**Implementation (already in code):**
1. ANALYZING phase (lines 1196-1221):
   - `resetAdjustmentQueue()` called (line 1198)
   - Loop calls `queueAdjustment()` for ALL improvements > threshold (lines 1199-1203)
   - `sortAdjustmentQueue()` called to prioritize (line 1215)
   - `runtime.adjState = ADJ_STATE_APPLY_NEXT` set (line 1217)
2. ARMED state (lines 1800-1810):
   - Check `runtime.adjState != ADJ_STATE_IDLE` (line 1800)
   - Call `updateSequentialAdjustment(currentTimeUs)` (line 1802)
3. Sequential adjustment state machine exists (lines 651-749):
   - `ADJ_STATE_APPLY_NEXT` → `ADJ_STATE_MEASURING` → `ADJ_STATE_VERIFY`
   - Reverts adjustments that make things worse (`ADJ_STATE_REVERT`)
   - Triggers wiggle signal when complete

---

### T2: Remove `__attribute__((unused))` Markers
**Status:** ⚠️ DEFERRED  
**Priority:** P2

**Remaining functions with unused markers:**
| Function | Line | Reason |
|----------|------|--------|
| `getLatestHistoryEntry()` | 365 | Helper for future Newton enhancements |
| `getMetricTarget()` | 406 | Helper for future metric targeting |

**Already used (no unused markers):**
- `calculateNewtonAdjustment()` - Used in sequential adjustment
- `resetNewtonHistory()` - Called on mode change (line 1851)
- Queue functions - All integrated and used

**Decision:** Keep unused markers on helper functions. They're useful utilities but not critical path. Removing markers causes `-Werror` build failures.

---

### T3: Populate Newton History from PID Tune Mode
**Status:** ✅ COMPLETE  
**Priority:** P2  
**Files:** `src/main/flight/autotune.c`

**Location:** ADJUSTING state (lines ~2092-2130) after `autotuneApplyGainAdjustment()`

**Add:**
```c
recordToHistory(runtime.currentAxis, TUNE_PARAM_P, runtime.currentP, runtime.metrics.overshootPercent);
recordToHistory(runtime.currentAxis, TUNE_PARAM_D, runtime.currentD, runtime.metrics.noiseRms);
recordToHistory(runtime.currentAxis, TUNE_PARAM_F, runtime.currentF, runtime.metrics.velocityWeightedLag);
recordToHistory(runtime.currentAxis, TUNE_PARAM_I, runtime.currentI, runtime.metrics.steadyStateError);
```

---

### T4: Mode-Change History Reset
**Status:** ✅ COMPLETE  
**Priority:** P3

**Implementation:** Line 1854 calls `resetNewtonHistory()` when `runtime.tuneMode` changes.

---

### T5: Add Measurement Functions for Verification
**Status:** ✅ COMPLETE  
**Priority:** P3  
**Files:** `src/main/flight/autotune.c`

**Implementation:** `updateSequentialAdjustment()` uses `computeWindowedMotorRms()` for hover diagnostic verification. This is correct since hover mode only measures motor RMS.

---

## Completed Tasks

### Newton's Method Functions ✅
- `calculateOptimalF_Newton()` - autotune_gains.c#L203
- `calculateOptimalP_Newton()` - autotune_gains.c#L264
- `calculateOptimalD_Newton()` - autotune_gains.c#L354
- `calculateOptimalI_Newton()` - autotune_gains.c#L407

### History Tracking Infrastructure ✅
- `newtonHistory_t`, `axisNewtonHistory_t`, `tuneNewtonHistory_t` in autotune_types.h
- `recordToHistory()`, `getAxisHistory()`, `getParameterHistory()` in autotune.c

### Sensitivity Calculations ✅
- `calculateSensitivity()` - autotune.c#L389
- `calculateProportionalAdjustment()` - autotune.c#L411
- `calculateNewtonAdjustment()` - autotune.c#L429
- `recordDiagnosticSensitivities()` - autotune.c#L858

### Sequential Queue Infrastructure ✅ (INTEGRATED)
- `pendingAdjustment_t`, `adjustmentQueue_t` in autotune_types.h
- `queueAdjustment()` - autotune.c#L508
- `sortAdjustmentQueue()` - autotune.c#L547
- `applyNextQueuedAdjustment()` - autotune.c#L563
- `updateSequentialAdjustment()` state machine - autotune.c#L651

### Reason Codes ✅
- All codes defined in autotune_types.h
- Output to debug[7] in autotune.c

---

## Test Results

### Flight Test: btfl__mk4___subagent.bbl.csv (2026-01-17)

**Summary:**
- Hover tune: Worked well, converged to low noise
- PID tune: F-biased, P/D not adjusting properly

**Detailed Analysis:**
| Metric | Value | Assessment |
|--------|-------|------------|
| F changes | 21 | Too many - bouncing |
| P changes | 1 | Too few |
| D changes | 0 | Should have adjusted |
| I changes | 0 | Expected (I is last) |
| Direction changes (D) | 9 | Bouncing detected |

**Reason Code Distribution:**
- 3510 (F_LAG_UP): 22,793 samples - dominant
- 3520 (F_LEAD_DOWN): 3,328 samples
- 3410 (D noise): 1,590 samples (but D wasn't changed!)
- P/D reason codes: minimal

---

## Next Actions (Priority Order)

1. ~~**P0: Fix Attribution Order**~~ ✅ COMPLETE
2. ~~**P3: Critical Damping Detection**~~ ✅ COMPLETE
3. ~~**P2: Bidirectional Exploration**~~ ✅ COMPLETE
4. **Flight test to validate fixes** 🔜
5. **P4: P/D Push-Up Logic** (can now proceed - P3 complete)
6. **P5: Integrated Filter+PID** (can now proceed)
7. **P6: Notch Configuration**
