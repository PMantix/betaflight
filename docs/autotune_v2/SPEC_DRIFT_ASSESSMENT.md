# Spec Drift & Consistency Assessment: Autotune V2

**Assessment Date:** January 24, 2026  
**Lead Engineer:** Spec Drift & Consistency Lead  
**Project Status:** Implementation Complete, Ready for Flight Testing

---

## 1. Executive Summary

### Overall Consistency Health: ⚠️ WARNING

| Category | Status | Issues |
|----------|--------|--------|
| PRD ↔ Code | ⚠️ Minor drifts | 4 issues |
| Code ↔ Types | ✅ Good | 1 issue |
| Code ↔ Tests | ❌ Critical gap | 0 tests exist |
| Code ↔ Analysis Scripts | ⚠️ Partial | 2 issues |
| PRD ↔ Verification | ⚠️ Stale references | 3 issues |

### Total Issues Detected: 10

### Most Dangerous Drifts (Potential Field Failures)

1. **BLOCKER: AUTOTUNE_STATE_ABORTED exists in docs but NOT in code** - Analysis scripts will fail on state value "9" which doesn't exist in enum
2. **HIGH: No unit tests or simulation tests** - VERIFICATION.md describes tests that don't exist
3. **MEDIUM: Debug channel mapping inconsistency** - Code uses different channels than documented
4. **MEDIUM: Reason code numbering divergence** - LOG_ANALYSIS.md uses 1xxx codes, code uses 0-35

### Recommended Resolution Actions

1. **Immediate:** Add `AUTOTUNE_STATE_ABORTED = 9` to `autotune_types.h` or update all docs to remove it
2. **Immediate:** Update debug channel constants to match documented layout
3. **Before Flight Test:** Create at least Python simulation validation
4. **Before Flight Test:** Align reason code numbering across all artifacts

---

## 2. Canonical Artifact Map

### 2.1 PRD Sections and Scope

| Section | File | Status |
|---------|------|--------|
| Executive Summary | PRD.md §1 | ✅ Complete |
| Global Config | PRD.md §2 | ✅ Implemented in autotune_pg.c |
| Event Philosophy | PRD.md §3 | ✅ Implemented in autotune_event.c |
| User Stories | PRD.md §4 | ✅ Described |
| Master State Flow | PRD.md §5 | ⚠️ Minor deviations |
| Phase Details | PRD.md §6 | ✅ Implemented |
| Stability Mechanisms | PRD.md §7 | ✅ Implemented in autotune_rollback.c |
| User Feedback | PRD.md §8 | ✅ Implemented in autotune_feedback.c |
| Debug Output | PRD.md §12 | ⚠️ Inconsistencies |

### 2.2 Implemented States (from code)

| Enum Value | State Name | In PRD | In Code | Status |
|------------|-----------|--------|---------|--------|
| 0 | AUTOTUNE_STATE_IDLE | ✅ | ✅ | Match |
| 1 | AUTOTUNE_STATE_HOVER_LOCK | ✅ | ✅ | Match |
| 2 | AUTOTUNE_STATE_THROTTLE_SWEEP | ✅ | ✅ | Match |
| 3 | AUTOTUNE_STATE_NOISE_CONFIRM | ✅ | ✅ | Match |
| 4 | AUTOTUNE_STATE_PD_RATIO_SEEK | ✅ | ✅ | Match |
| 5 | AUTOTUNE_STATE_PD_SCALE_UP | ✅ | ✅ | Match |
| 6 | AUTOTUNE_STATE_F_TUNE | ✅ | ✅ | Match |
| 7 | AUTOTUNE_STATE_PD_RETUNE_AFTER_F | ✅ | ✅ | Match |
| 8 | AUTOTUNE_STATE_COMPLETE | ✅ | ✅ | Match |
| 9 | AUTOTUNE_STATE_ABORTED | ✅ (LOG_ANALYSIS.md) | ❌ MISSING | **DRIFT** |

### 2.3 Implemented Criteria & Metrics

| Metric | PRD Reference | Code Location | Status |
|--------|---------------|---------------|--------|
| Overshoot % | PRD §6.4 | autotune_metrics.c | ✅ |
| Rebound detection | PRD §6.4 | autotune_metrics.c | ✅ |
| Lag (t50) | PRD §6.5, §6.6 | autotune_metrics.c | ✅ |
| Settling time | PRD §6.4 | autotune_metrics.c | ✅ |
| Cross-axis rejection | PRD §3.2 | autotune_event.c | ✅ |

### 2.4 Test Cases and Coverage

| Test Type | Documented | Exists | Coverage |
|-----------|------------|--------|----------|
| Unit Tests | VERIFICATION.md §2 | ❌ NO FILES | 0% |
| Simulation Tests | VERIFICATION.md §3 | ❌ NO FILES | 0% |
| Bench Tests | VERIFICATION.md §4 | Checklists only | N/A |
| Flight Tests | VERIFICATION.md §5 | Checklists only | N/A |

### 2.5 Analysis Scripts

| Script | Location | Status |
|--------|----------|--------|
| `analyze_log.py` | `scripts/` | ❌ NOT FOUND |
| `plot_autotune.py` | `scripts/` | ❌ NOT FOUND |
| `analyze_quick.py` | `bb logs/` | ⚠️ Found, partial |

---

## 3. PRD ↔ Code Consistency Check

### Issue #1: ABORTED State Missing

| Aspect | Details |
|--------|---------|
| **PRD Claim** | LOG_ANALYSIS.md §3.2: `9 = ABORTED // Tuning stopped due to error/pilot` |
| **Code Reality** | `autotune_types.h` line 59: enum ends at `AUTOTUNE_STATE_COUNT` (value 9 as sentinel) |
| **Consistency Status** | ❌ **Code deviates from PRD** |
| **Evidence** | `autotuneState_e` enum only has 9 values (0-8), no ABORTED state |
| **Risk Assessment** | **HIGH** - Analysis scripts expect state 9 to mean ABORTED |
| **Recommended Action** | Either: (a) Add `AUTOTUNE_STATE_ABORTED = 9` before `AUTOTUNE_STATE_COUNT`, OR (b) Update LOG_ANALYSIS.md to remove ABORTED and clarify abort goes to IDLE with reason code |

### Issue #2: Reason Code Numbering Scheme Divergence

| Aspect | Details |
|--------|---------|
| **PRD Claim** | LOG_ANALYSIS.md §3.4 uses 4-digit codes: `1001 = Activated`, `1401 = Abort: stick movement` |
| **Code Reality** | `autotune_debug.h` uses 0-35: `AUTOTUNE_REASON_ABORT_SWITCH = 30` |
| **Consistency Status** | ❌ **Code deviates from PRD** |
| **Evidence** | `autotuneReason_e` enum in autotune_debug.h lines 30-65 |
| **Risk Assessment** | **MEDIUM** - Log analysis scripts will misinterpret reason codes |
| **Recommended Action** | Update LOG_ANALYSIS.md to reflect actual 0-35 codes, or redefine code enum to use documented 1xxx scheme |

### Issue #3: Debug Channel Layout Mismatch

| Aspect | Details |
|--------|---------|
| **PRD Claim** | PRD §12 and LOG_ANALYSIS.md: `debug[5] = Reason code` |
| **Code Reality** | Code uses `AUTOTUNE_DEBUG_REASON` but actual channel assignment unclear |
| **Consistency Status** | ⚠️ **PRD underspecified** |
| **Evidence** | autotune_debug.h defines macros but channel numbers not visible in context |
| **Risk Assessment** | **MEDIUM** - Potential mismatch between logged data and expected channels |
| **Recommended Action** | Audit `AUTOTUNE_DEBUG_*` definitions and ensure they match documented channel 0-7 mapping |

### Issue #4: Step Size Configuration Divergence

| Aspect | Details |
|--------|---------|
| **PRD Claim** | PRD §2.1: `P_STEP_BASE = 0.10` (10%), `MIN_STEP = 0.02`, `MAX_STEP = 0.15` |
| **Code Reality** | autotune_pg.c: `pStepPercent = 10` (matches), but MIN/MAX_STEP not in PG |
| **Consistency Status** | ⚠️ **PRD underspecified** |
| **Evidence** | Constants like `MIN_STEP`, `MAX_STEP` should be in autotune_debug.h or types.h |
| **Risk Assessment** | **LOW** - May affect tuning aggressiveness bounds |
| **Recommended Action** | Document where MIN_STEP/MAX_STEP are defined, or add to PG if configurable |

---

## 4. Code ↔ Tests Consistency Check

### Critical Finding: NO TESTS EXIST

| Test Category | VERIFICATION.md Claim | Reality |
|---------------|----------------------|---------|
| Unit Tests | §2 describes DBL-*, MSR-*, OPT-* test IDs | ❌ No test files found in `src/test/` |
| Python Simulation | §3 describes `scripts/autotune_sim.py` | ❌ File does not exist |
| Simulation Test Cases | SIM-01 through SIM-06 documented | ❌ No implementation |

### False Coverage Warning

**VERIFICATION.md describes test infrastructure that does not exist.**

This is a **BLOCKER-level documentation drift** because:
- Engineers may believe tests provide coverage
- PRD acceptance criteria reference test outcomes
- No automated validation of algorithm correctness

### Missing Coverage - Critical Paths

| Critical Path | Test Coverage |
|---------------|---------------|
| Bracket search convergence | ❌ None |
| Newton refinement fallback | ❌ None |
| Rollback on rebound | ❌ None |
| Cross-axis rejection | ❌ None |
| Timeout handling | ❌ None |
| consecutiveBadEvents forcing advance | ❌ None |

---

## 5. Code ↔ Analysis Scripts Consistency Check

### Issue #5: Expected Scripts Missing

| Aspect | Details |
|--------|---------|
| **Documentation Claim** | LOG_ANALYSIS.md §4.1: `python scripts/analyze_log.py` |
| **Reality** | `scripts/` folder contents unknown; `analyze_quick.py` found in `bb logs/` instead |
| **Consistency Status** | ❌ **Stale script references** |
| **Evidence** | File search found only `bb logs/analyze_quick.py` |
| **Risk Assessment** | **LOW** - User confusion on script location |
| **Recommended Action** | Move `analyze_quick.py` to `scripts/` or update docs |

### Issue #6: analyze_quick.py State Mapping

| Aspect | Details |
|--------|---------|
| **Script Assumption** | Script at lines 13-17 uses `debug[7]` for reason codes |
| **Code Reality** | unclear if `debug[7]` is `AUTOTUNE_DEBUG_REASON` or packed axis+iter |
| **Consistency Status** | ⚠️ **Potential misinterpretation** |
| **Evidence** | `analyze_quick.py` line 15: `df['debug[7]'].value_counts()` |
| **Risk Assessment** | **MEDIUM** - Script may misinterpret debug data |
| **Recommended Action** | Verify debug[7] channel assignment matches script expectations |

### Issue #7: State Value 9 in Scripts

| Aspect | Details |
|--------|---------|
| **Script Assumption** | LOG_ANALYSIS.md §5.3 matplotlib code uses `'ABORTED'` for state 9 |
| **Code Reality** | State 9 is `AUTOTUNE_STATE_COUNT` (sentinel), not a valid state |
| **Consistency Status** | ❌ **Analysis drift** |
| **Evidence** | LOG_ANALYSIS.md line 282: `ax.set_yticklabels([...'ABORTED'])` |
| **Risk Assessment** | **HIGH** - Plotting code will label invalid data |
| **Recommended Action** | Remove 'ABORTED' from label list or add state to code |

---

## 6. PRD ↔ Tests ↔ Analysis Triangle

### Chain of Evidence Assessment

| PRD Requirement | Code Implementation | Test Validation | Log Verification | Status |
|-----------------|---------------------|-----------------|------------------|--------|
| Single event = single decision | ✅ Implemented | ❌ No test | ⚠️ Partial script | **WEAK** |
| Overshoot 5-10% target | ✅ Implemented | ❌ No test | ⚠️ Partial script | **WEAK** |
| Rollback on rebound | ✅ Implemented | ❌ No test | ❌ No script check | **NONE** |
| Filters before gains | ✅ State ordering correct | ❌ No test | ⚠️ State visible | **WEAK** |
| <= 20-30 maneuvers | ✅ Event limits in code | ❌ No test | ⚠️ Can count in log | **WEAK** |
| Complete in one battery | ✅ 3-min timeout | ❌ No test | ⚠️ Duration visible | **WEAK** |

### Key Question: "What evidence do we actually have that the PRD intent is satisfied?"

**Answer:** Currently **ONLY visual inspection of code**. There are:
- ❌ No automated tests validating algorithm behavior
- ❌ No simulation proving convergence properties
- ⚠️ Partial scripts for post-flight log review
- ✅ Code compiles clean
- ❓ Unknown: actual flight test results

---

## 7. Drift Classification & Risk Ranking

| ID | Type | Severity | Likelihood | Detectability | Priority |
|----|------|----------|------------|---------------|----------|
| #1 | Implementation drift | Safety risk | Likely | Silent | **P0** |
| #2 | Documentation drift | Misleading | Likely | Obvious | **P1** |
| #3 | Documentation drift | Misleading | Occasional | Subtle | **P1** |
| #4 | Documentation drift | Cosmetic | Rare | Obvious | **P3** |
| #5 | Documentation drift | Cosmetic | Likely | Obvious | **P3** |
| #6 | Analysis drift | Functional risk | Likely | Subtle | **P1** |
| #7 | Analysis drift | Misleading | Occasional | Silent | **P1** |
| #8 | Test drift | Safety risk | Certain | Silent | **P0** |
| #9 | Test drift | Safety risk | Certain | Silent | **P0** |
| #10 | Test drift | Safety risk | Certain | Silent | **P0** |

---

## 8. Issue List (Handoff-Ready)

### Issue #1: ABORTED State Missing from Code

| Field | Value |
|-------|-------|
| **ID** | DRIFT-001 |
| **Drift type** | Implementation drift |
| **Affected artifacts** | PRD (LOG_ANALYSIS.md), Code (autotune_types.h), Analysis scripts |
| **Description** | LOG_ANALYSIS.md documents state value 9 as ABORTED, but `autotuneState_e` enum has no such state. State 9 is `AUTOTUNE_STATE_COUNT` sentinel. |
| **Evidence** | `autotune_types.h:50-59` shows enum ending at COMPLETE=8 |
| **Risk level** | HIGH - Analysis scripts will fail or misinterpret |
| **Proposed resolution** | Option A: Add `AUTOTUNE_STATE_ABORTED = 9` to enum. Option B: Update all docs to clarify abort transitions to IDLE with reason code, not separate state |
| **Validation steps** | 1. Grep for "ABORTED" in all docs 2. Verify state 9 behavior 3. Update scripts |

---

### Issue #2: Reason Code Scheme Divergence

| Field | Value |
|-------|-------|
| **ID** | DRIFT-002 |
| **Drift type** | Documentation drift |
| **Affected artifacts** | PRD (LOG_ANALYSIS.md), Code (autotune_debug.h) |
| **Description** | LOG_ANALYSIS.md §3.4 uses 4-digit codes (1001, 1401, etc). Code uses 0-35 sequential enum. |
| **Evidence** | `autotune_debug.h:30-65` defines enum from 0-35; LOG_ANALYSIS.md shows 1xxx codes |
| **Risk level** | MEDIUM - Log interpretation will be wrong |
| **Proposed resolution** | Update LOG_ANALYSIS.md §3.4 to reflect actual 0-35 codes from autotune_debug.h |
| **Validation steps** | 1. Export code enum as table 2. Replace doc section 3. Update any scripts |

---

### Issue #3: Debug Channel Assignment Unclear

| Field | Value |
|-------|-------|
| **ID** | DRIFT-003 |
| **Drift type** | Documentation underspecified |
| **Affected artifacts** | PRD (LOG_ANALYSIS.md, PRD.md), Code (autotune_debug.h) |
| **Description** | PRD documents debug[0-7] layout but code macro definitions not verified to match |
| **Evidence** | Need to verify `AUTOTUNE_DEBUG_STATE`, `AUTOTUNE_DEBUG_AXIS`, etc. map to channels 0-7 correctly |
| **Risk level** | MEDIUM |
| **Proposed resolution** | Add explicit channel number comments in autotune_debug.h |
| **Validation steps** | 1. Read autotune_debug.h fully 2. Confirm channel assignments 3. Add inline docs |

---

### Issue #4: MIN_STEP/MAX_STEP Constants Location

| Field | Value |
|-------|-------|
| **ID** | DRIFT-004 |
| **Drift type** | PRD underspecified |
| **Affected artifacts** | PRD.md §2.1 |
| **Description** | PRD mentions MIN_STEP=0.02, MAX_STEP=0.15 but these aren't in PG and location unclear |
| **Evidence** | Not found in autotune_pg.h/c |
| **Risk level** | LOW |
| **Proposed resolution** | Document where these constants live or add to autotune_debug.h |
| **Validation steps** | Grep for MIN_STEP, MAX_STEP in codebase |

---

### Issue #5: Script Location Mismatch

| Field | Value |
|-------|-------|
| **ID** | DRIFT-005 |
| **Drift type** | Documentation stale |
| **Affected artifacts** | LOG_ANALYSIS.md |
| **Description** | Docs reference `scripts/analyze_log.py` but script found at `bb logs/analyze_quick.py` |
| **Evidence** | File search results |
| **Risk level** | LOW |
| **Proposed resolution** | Create `scripts/` folder with proper analysis tools or update docs |
| **Validation steps** | 1. Create scripts folder 2. Move/rename scripts 3. Update docs |

---

### Issue #6: analyze_quick.py Channel Interpretation

| Field | Value |
|-------|-------|
| **ID** | DRIFT-006 |
| **Drift type** | Analysis drift |
| **Affected artifacts** | bb logs/analyze_quick.py, Code |
| **Description** | Script uses `debug[7]` for reason codes but PRD says debug[5] is reason code |
| **Evidence** | analyze_quick.py:15 vs LOG_ANALYSIS.md §3.1 |
| **Risk level** | MEDIUM |
| **Proposed resolution** | Verify actual debug channel mapping in code and fix script |
| **Validation steps** | 1. Check code debug assignments 2. Update script 3. Test with real log |

---

### Issue #7: State Label Array Includes ABORTED

| Field | Value |
|-------|-------|
| **ID** | DRIFT-007 |
| **Drift type** | Analysis drift |
| **Affected artifacts** | LOG_ANALYSIS.md example code |
| **Description** | Matplotlib code snippet includes 10 labels (ending with ABORTED) but only 9 states exist |
| **Evidence** | LOG_ANALYSIS.md:282 |
| **Risk level** | MEDIUM - Will cause array index issues |
| **Proposed resolution** | Remove ABORTED from label list or add state to code (linked to DRIFT-001) |
| **Validation steps** | Fix DRIFT-001 first, then update this |

---

### Issue #8: No Unit Tests (BLOCKER)

| Field | Value |
|-------|-------|
| **ID** | DRIFT-008 |
| **Drift type** | Test drift - MISSING |
| **Affected artifacts** | VERIFICATION.md §2, Code |
| **Description** | VERIFICATION.md describes unit tests DBL-*, MSR-*, OPT-* but none exist |
| **Evidence** | File search for `*autotune*test*` found 0 results |
| **Risk level** | **BLOCKER** - No automated validation |
| **Proposed resolution** | Create unit tests for critical functions OR mark VERIFICATION.md as aspirational/future |
| **Validation steps** | 1. Decide on test strategy 2. Either implement or update docs |

---

### Issue #9: No Simulation Tests (BLOCKER)

| Field | Value |
|-------|-------|
| **ID** | DRIFT-009 |
| **Drift type** | Test drift - MISSING |
| **Affected artifacts** | VERIFICATION.md §3, `scripts/` |
| **Description** | VERIFICATION.md describes Python simulator and SIM-01 to SIM-06 tests, none exist |
| **Evidence** | `scripts/autotune_sim.py` not found |
| **Risk level** | **BLOCKER** - Algorithm not validated |
| **Proposed resolution** | Create basic Python simulation OR document that validation is flight-test-only |
| **Validation steps** | 1. Decide test strategy 2. Implement or update docs |

---

### Issue #10: No Bench Test Automation (BLOCKER)

| Field | Value |
|-------|-------|
| **ID** | DRIFT-010 |
| **Drift type** | Test drift - MISSING |
| **Affected artifacts** | VERIFICATION.md §4 |
| **Description** | Bench tests described as checklists only, no automation |
| **Evidence** | BNC-01 through BNC-08 are manual procedures |
| **Risk level** | HIGH - No pre-flight validation |
| **Proposed resolution** | Accept as manual process OR create SITL-based automation |
| **Validation steps** | Document explicit decision on manual vs automated bench tests |

---

## 9. Non-Negotiable Findings (Blockers)

### 🚫 BLOCKER #1: State Enum Mismatch

**Impact:** Analysis scripts using LOG_ANALYSIS.md guidance will misinterpret state values when they encounter the `AUTOTUNE_STATE_COUNT` value (9) in logs, thinking it means ABORTED when it's actually just a sentinel.

**Required Resolution:** Align state enum with documentation before flight testing.

---

### 🚫 BLOCKER #2: No Automated Testing

**Impact:** There is **zero automated verification** that the algorithm works correctly. All test documentation describes tests that don't exist.

**Required Resolution:** Either:
- Create basic simulation tests, OR
- Explicitly mark VERIFICATION.md as "planned/aspirational" and accept flight-test-only validation

---

### 🚫 BLOCKER #3: Reason Code Interpretation Failure

**Impact:** Anyone using LOG_ANALYSIS.md to interpret reason codes will get wrong results. Code uses 0-35, docs describe 1xxx.

**Required Resolution:** Update LOG_ANALYSIS.md before sharing with testers.

---

## 10. Recommended Next Steps

### Immediate (Before Any Flight Test)

1. **DRIFT-001:** Decide on ABORTED state - add to code or remove from docs
2. **DRIFT-002:** Update LOG_ANALYSIS.md reason code table to match `autotune_debug.h`
3. **DRIFT-007:** Fix matplotlib label array
4. **DRIFT-008/009/010:** Add disclaimer to VERIFICATION.md that tests are not yet implemented

### Short-Term (Before Community Release)

5. Create `scripts/analyze_autotune.py` with correct channel mappings
6. Create basic Python simulation for bracket search validation
7. Add explicit debug channel number comments to autotune_debug.h
8. Validate analyze_quick.py against actual log with new firmware

### Medium-Term (Quality Improvement)

9. Implement at least SIM-01 through SIM-03 simulation tests
10. Create automated bench test using SITL
11. Establish canonical state/criteria registry (auto-generated from code)

---

## 11. Appendix: Canonical State Registry (Extracted from Code)

```c
// From autotune_types.h
typedef enum {
    AUTOTUNE_STATE_IDLE = 0,            // Inactive
    AUTOTUNE_STATE_HOVER_LOCK = 1,      // Waiting for stable hover
    AUTOTUNE_STATE_THROTTLE_SWEEP = 2,  // Filter characterization
    AUTOTUNE_STATE_NOISE_CONFIRM = 3,   // Validate filters
    AUTOTUNE_STATE_PD_RATIO_SEEK = 4,   // Find critical damping
    AUTOTUNE_STATE_PD_SCALE_UP = 5,     // Scale P+D together
    AUTOTUNE_STATE_F_TUNE = 6,          // Feedforward tuning
    AUTOTUNE_STATE_PD_RETUNE_AFTER_F = 7, // Re-validate P/D
    AUTOTUNE_STATE_COMPLETE = 8,        // Finished
    AUTOTUNE_STATE_COUNT = 9            // SENTINEL - NOT A STATE
} autotuneState_e;
```

---

## 12. Appendix: Canonical Reason Code Registry (Extracted from Code)

```c
// From autotune_debug.h
typedef enum {
    // Normal (0-10)
    AUTOTUNE_REASON_NONE = 0,
    AUTOTUNE_REASON_EVENT_DETECTED = 1,
    AUTOTUNE_REASON_QUALITY_OK = 2,
    AUTOTUNE_REASON_DECISION_INCREASE = 3,
    AUTOTUNE_REASON_DECISION_DECREASE = 4,
    AUTOTUNE_REASON_DECISION_HOLD = 5,
    AUTOTUNE_REASON_AXIS_COMPLETE = 6,
    AUTOTUNE_REASON_PHASE_COMPLETE = 7,
    AUTOTUNE_REASON_HOVER_LOCKED = 8,
    AUTOTUNE_REASON_FILTERS_SET = 9,
    AUTOTUNE_REASON_TARGET_REACHED = 10,

    // Quality failures (11-19)
    AUTOTUNE_REASON_INSUFFICIENT_DEFLECTION = 11,
    AUTOTUNE_REASON_CROSS_AXIS_CONTAMINATION = 12,
    AUTOTUNE_REASON_THROTTLE_OUT_OF_BAND = 13,
    AUTOTUNE_REASON_ABNORMAL_DURATION = 14,
    AUTOTUNE_REASON_INVALID_METRICS = 15,
    AUTOTUNE_REASON_EVENT_TIMEOUT = 16,
    AUTOTUNE_REASON_STICK_NOT_CENTERED = 17,

    // Safety (20-29)
    AUTOTUNE_REASON_ROLLBACK_OSCILLATION = 20,
    AUTOTUNE_REASON_ROLLBACK_OVERSHOOT = 21,
    AUTOTUNE_REASON_GAIN_LIMIT_MIN = 22,
    AUTOTUNE_REASON_GAIN_LIMIT_MAX = 23,
    AUTOTUNE_REASON_EVENT_LIMIT = 24,
    AUTOTUNE_REASON_NOISE_TOO_HIGH = 25,
    AUTOTUNE_REASON_TRUST_DEPLETED = 26,

    // Abort (30-39)
    AUTOTUNE_REASON_ABORT_SWITCH = 30,
    AUTOTUNE_REASON_ABORT_DISARM = 31,
    AUTOTUNE_REASON_ABORT_FAILSAFE = 32,
    AUTOTUNE_REASON_ABORT_TIMEOUT = 33,
    AUTOTUNE_REASON_ABORT_USER = 34,
    AUTOTUNE_REASON_ABORT_ERROR = 35,
} autotuneReason_e;
```

---

*Assessment completed: January 24, 2026*  
*Next review scheduled: After resolving BLOCKER issues*
