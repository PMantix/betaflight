# Autotune V2 Consolidated Issue Tracker

**Created:** January 23, 2026  
**Updated:** January 24, 2026  
**Status:** Active Development  
**Sources:** CRITERIA_ASSESSMENT.md, STATE_FLOW_ASSESSMENT.md, LOGGING_OBSERVABILITY_ASSESSMENT.md, SPEC_DRIFT_ASSESSMENT.md

---

## Issue Summary

### Previously Identified (Criteria & StateFlow)

| ID | Source | Severity | Status | Description |
|----|--------|----------|--------|-------------|
| CRIT-011 | Criteria | ❌ Blocker | ✅ FIXED | Event quality gate units mismatch (normalized vs degrees) |
| CRIT-004 | Criteria | ❌ Blocker | ✅ FIXED | Filter application not implemented |
| CRIT-008 | Criteria | 🟠 Major | ✅ FIXED | Hardcoded 4000µs sample period |
| CRIT-010 | Criteria | 🟠 Major | ✅ FIXED | Rebound threshold uses wrong reference |
| SFA-001 | StateFlow | 🟡 Medium | ✅ FIXED | autotuneInit() not called at startup |
| SFA-002 | StateFlow | 🟡 Medium | ✅ FIXED | NOISE_CONFIRM filter not actually tightened |
| CRIT-005 | Criteria | 🟢 Minor | ✅ FIXED | Static variable noiseConfirmIterations state leak |
| CRIT-009 | Criteria | 🟢 Minor | ✅ FIXED | Lag target formula mismatch with PRD |
| CRIT-001 | Criteria | 🟢 Minor | ✅ FIXED | Throttle normalization assumes specific range |
| SFA-003 | StateFlow | 🟡 Medium | 📋 DEFERRED | PRD safety behaviors not fully implemented |
| SFA-004 | StateFlow | 🟢 Low | 📋 DEFERRED | No per-state inactivity timeout |
| SFA-005 | StateFlow | 🟢 Low | 📋 DEFERRED | Unused substates APPLYING/CONFIRMING |

### Observability Issues (January 24, 2026)

| ID | Source | Severity | Status | Description |
|----|--------|----------|--------|-------------|
| OBS-002 | Observability | 🔴 Blocker | ✅ FIXED | Abort does not restore original gains |
| OBS-003 | Observability | 🔴 Blocker | 📋 PARTIAL | Parameter old→new values not logged |
| OBS-004 | Observability | 🟡 Major | ✅ FIXED | Reason code docs don't match code |
| OBS-005 | Observability | 🟡 Major | ✅ FIXED | Lag metric not logged |
| OBS-007 | Observability | 🟡 Major | 📋 TODO | Transition reason codes inconsistent |
| OBS-001 | Observability | 🔴 Blocker | 📋 DEFERRED | No save-to-memory command |

### Spec Drift Issues (January 24, 2026)

| ID | Source | Severity | Status | Description |
|----|--------|----------|--------|-------------|
| DRIFT-001 | SpecDrift | 🔴 Blocker | ✅ FIXED | ABORTED state (9) in docs but not in code |
| DRIFT-002 | SpecDrift | 🟡 Medium | ✅ FIXED | Reason code scheme 1xxx vs 0-35 |
| DRIFT-003 | SpecDrift | 🟡 Medium | ✅ FIXED | Debug channel mapping unclear |
| DRIFT-008 | SpecDrift | 🔴 Blocker | ✅ FIXED | VERIFICATION.md describes non-existent tests |
| DRIFT-004 | SpecDrift | 🟢 Low | 📋 TODO | MIN_STEP/MAX_STEP location unclear |
| DRIFT-005 | SpecDrift | 🟢 Low | 📋 TODO | Script location mismatch |
| DRIFT-006 | SpecDrift | 🟡 Medium | 📋 TODO | analyze_quick.py channel interpretation |

---

## Detailed Fix Log

### CRIT-011: Event Quality Gate Units Mismatch ❌→✅

**Problem:** `stickDeflection` stored as normalized float (0-1), compared against 15.0 degrees.

**Fix:** Changed thresholds from degrees to normalized values:
- `AUTOTUNE_STICK_DEFLECTION_MIN` = 0.15 (15% stick travel)
- `AUTOTUNE_CROSS_AXIS_MAX` = 0.10 (10% cross-axis)

**Files Changed:**
- `autotune_debug.h` - Changed constant names and values
- `autotune_event.c` - Updated function to use new constants

---

### CRIT-004: Filter Application Not Implemented ❌→✅

**Problem:** `autotuneFilterApplyRecommendations()` was a TODO stub.

**Fix:** Implemented filter application using Betaflight's gyro/PID filter configuration APIs.

**Files Changed:**
- `autotune_filter.c` - Implemented `autotuneFilterApplyRecommendations()` and `autotuneFilterRollback()`
- `autotune_filter.c` - Added `autotuneFilterSetRecommendedLpf()` for NOISE_CONFIRM iteration

---

### CRIT-008: Hardcoded Sample Period 🟠→✅

**Problem:** Metrics used hardcoded 4000µs (250Hz), breaking on 4K/8K systems.

**Fix:** Pass actual PID loop period from `gyro.targetLooptime` to metrics functions.

**Files Changed:**
- `autotune_metrics.h` - Added `loopTimeUs` parameter to `autotuneMetricsAnalyze()`
- `autotune_metrics.c` - Updated to use passed loop time
- `autotune_core.c` - Pass `gyro.targetLooptime` to metrics analysis

---

### CRIT-010: Rebound Threshold Wrong Reference 🟠→✅

**Problem:** Rebound threshold was relative to first peak, not setpoint.

**Fix:** Changed rebound detection to use setpoint as reference, pass setpoint to detection function.

**Files Changed:**
- `autotune_metrics.h` - Added setpoint parameter to rebound detection
- `autotune_metrics.c` - Use setpoint-relative threshold

---

### SFA-001: autotuneInit() Not Called 🟡→✅

**Problem:** Task definition did not call autotuneInit during initialization.

**Fix:** Added `autotuneInit()` call in `tasksInit()` before enabling the AUTOTUNE task.

**Files Changed:**
- `tasks.c` - Added `autotuneInit();` call in `tasksInit()` where `setTaskEnabled(TASK_AUTOTUNE, true)` is called

---

### SFA-002: NOISE_CONFIRM Filter Not Tightened 🟡→✅

**Problem:** `tighterLpf` computed but never stored to filterState.

**Fix:** Added `autotuneFilterSetRecommendedLpf()` function and call it in NOISE_CONFIRM.

**Files Changed:**
- `autotune_filter.h` - Added `autotuneFilterSetRecommendedLpf()` declaration
- `autotune_filter.c` - Implemented setter function
- `autotune_core.c` - Call setter before reapplying recommendations

---

### CRIT-005: Static Variable State Leak 🟢→✅

**Problem:** `noiseConfirmIterations` not reset on abort/restart.

**Fix:** Reset counter in `stateNoiseConfirmEnter()` instead of relying on exit.

**Files Changed:**
- `autotune_core.c` - Reset counter in Enter function

---

### CRIT-009: Lag Target Formula Mismatch 🟢→✅

**Problem:** Default lag target 100ms too sluggish (PRD says 10ms).

**Fix:** Changed formula to `25 - (aggressiveness × 15)` giving 10-25ms range.

**Files Changed:**
- `autotune_core.c` - Updated lag target calculation in `statePdScaleUpEnter()`

---

### CRIT-001: Throttle Normalization 🟢→✅

**Problem:** Assumed rcCommand[THROTTLE] is 0-1000.

**Fix:** Use proper PWM range constants for normalization.

**Files Changed:**
- `autotune_core.c` - Updated throttle normalization to use PWM_RANGE constants

---

## Observability Fixes (January 24, 2026)

### OBS-002: Abort Does Not Restore Original Gains 🔴→✅

**Problem:** `autotuneAbort()` had TODO stub, gains not restored on abort.

**Fix:** Implemented gain restoration loop for all enabled axes, restores originalP/D/F.

**Files Changed:**
- `autotune_core.c` - Implemented `autotuneAbort()` with gain restoration loop

---

### OBS-004/DRIFT-002: Reason Code Documentation 🟡→✅

**Problem:** LOG_ANALYSIS.md used 1xxx reason codes, code uses 0-35 enum.

**Fix:** Updated LOG_ANALYSIS.md §3.4 to reflect actual `autotuneReason_e` enum values.

**Files Changed:**
- `LOG_ANALYSIS.md` - Replaced reason code table with actual enum values

---

### OBS-005: Lag Metric Not Logged 🟡→✅

**Problem:** Lag (t50) computed but never logged, critical for PD_SCALE_UP and F_TUNE.

**Fix:** Added lag×10 logging after metrics computation.

**Files Changed:**
- `autotune_core.c` - Added `AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_GAIN_F, lagMs*10)` in PD_SCALE_UP
- `autotune_core.c` - Added `AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_DECISION, lagMs*10)` in F_TUNE

---

### DRIFT-001: ABORTED State Missing from Code 🔴→✅

**Problem:** LOG_ANALYSIS.md documented state 9 as ABORTED, but code only has states 0-8.

**Fix:** Updated LOG_ANALYSIS.md to clarify abort transitions to IDLE (0) with reason code 30-35.

**Files Changed:**
- `LOG_ANALYSIS.md` - Updated state table, removed state 9, added comment about abort behavior

---

### DRIFT-003: Debug Channel Mapping Unclear 🟡→✅

**Problem:** No explicit documentation of which debug channel index maps to which value.

**Fix:** Added comprehensive channel mapping table in autotune_debug.h header.

**Files Changed:**
- `autotune_debug.h` - Added explicit channel number comments for all AUTOTUNE_DEBUG_* defines

---

### DRIFT-008: VERIFICATION.md Describes Non-Existent Tests 🔴→✅

**Problem:** Document described unit tests, simulation tests that don't exist.

**Fix:** Added disclaimer at top noting tests are planned but not yet implemented.

**Files Changed:**
- `VERIFICATION.md` - Added implementation status box with current vs planned coverage

---

## Deferred Items

### SFA-003: PRD Safety Behaviors

**Rationale:** Requires PRD clarification on exact thresholds for:
- "Sustained oscillation" detection
- "Excessive overshoot" abort trigger

**Action:** Create follow-up task after PRD review.

---

### SFA-004: Per-State Inactivity Timeout

**Rationale:** Global 3-minute timeout exists. Per-state timeouts are nice-to-have.

**Action:** Consider for Phase 5 polish.

---

### SFA-005: Unused Substates

**Rationale:** APPLYING and CONFIRMING substates may be used in future refinements.

**Action:** Keep for now, document as reserved.

---

## Validation Checklist

- [x] Build passes with `make CONFIG=BETAFPVG473`
- [ ] Events pass quality gates during normal stick movements
- [ ] Filter phase actually changes filter values (verify in blackbox)
- [ ] Lag values correct on 8K system (~30x smaller than before)
- [ ] NOISE_CONFIRM doesn't get stuck
- [ ] Rebound detection consistent across overshoot levels
- [ ] autotuneInit() called before first use

---

*Last Updated: January 23, 2026*
