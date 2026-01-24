# Autotune V2 Logging & Observability Assessment

**Version:** 1.0  
**Date:** January 24, 2026  
**Author:** Logging & Observability Lead Engineer  
**Status:** Initial Assessment

---

## 1. Executive Summary

### Observability Health: ⚠️ PARTIAL

The Autotune V2 implementation has a solid foundation for debug logging via blackbox, but **critical gaps exist** that prevent full verification of the autotune workflow from logs alone.

### Top 5 Missing Signals/Events That Block Verification

| Priority | Gap | Impact |
|----------|-----|--------|
| 🔴 **1** | **No save-to-memory event logging** | Cannot prove parameters were saved only when commanded |
| 🔴 **2** | **No abort/revert event with original values** | Cannot verify full parameter restoration on abort |
| 🔴 **3** | **Parameter old→new values not logged** | Cannot reconstruct exact gain history |
| 🟡 **4** | **Criteria evaluation not logged at decision time** | Cannot audit why specific decisions were made |
| 🟡 **5** | **Transition reason codes inconsistent with PRD** | Mismatch between code enum and documented codes |

### Quick Wins vs Structural Changes

| Quick Wins (< 1 day each) | Structural Changes (> 1 day) |
|---------------------------|------------------------------|
| Add parameter update event with old/new values | Implement save-to-memory command with audit log |
| Log abort event with original gain snapshot | Create criteria evaluation snapshot logging |
| Align reason code enums with PRD documentation | Add event-rate throttling infrastructure |
| Add iteration counter to debug output | Implement log versioning/schema |

---

## 2. Observability Requirements (What Must Be Answerable)

### 2.1 Workflow & Timing

| Question | Currently Answerable? | Evidence |
|----------|----------------------|----------|
| What state was the system in at every time? | ✅ Yes | `debug[0]` = master state enum |
| When did it enter/exit each state? | ⚠️ Partial | State changes visible, but no explicit entry timestamp log |
| What triggered each transition (reason code + key metric values)? | ⚠️ Partial | `debug[5]/debug[7]` has reason, but not always paired with metrics |
| How many iterations/phases occurred and where? | ❌ No | `axis×100 + iter` packed value not implemented in current code |

### 2.2 Decision Audit

| Question | Currently Answerable? | Evidence |
|----------|----------------------|----------|
| What criteria were evaluated in each state? | ⚠️ Partial | Overshoot logged, but not thresholds or gating conditions |
| What were the metric values at decision time? | ⚠️ Partial | `debug[4]` = overshoot×10, but lag/rebound not logged |
| Was decision due to success, failure, timeout, or safety? | ⚠️ Partial | Reason codes exist but not logged at every decision |
| Why was a specific gain direction chosen? | ❌ No | Bracket state, Newton step logic not logged |

### 2.3 Tuning Actions

| Question | Currently Answerable? | Evidence |
|----------|----------------------|----------|
| When were parameters changed? | ✅ Yes | `debug[5]` (GAIN_P) and `debug[6]` (GAIN_D) update on change |
| What changed (old → new values)? | ❌ No | Only new value logged, not previous |
| Why was the change applied? | ⚠️ Partial | Reason code logged but not always at exact change moment |
| Did the change improve the quality metric? | ❌ No | No before/after comparison logged |

### 2.4 Safety

| Question | Currently Answerable? | Evidence |
|----------|----------------------|----------|
| Did an abort occur? Why? | ⚠️ Partial | Reason codes 30-39 exist, but `autotuneAbort()` has TODO for gain restore |
| Did parameters revert fully? | ❌ No | Original values stored but not logged, revert not confirmed |
| Did any safety clamp activate? | ⚠️ Partial | `AUTOTUNE_REASON_GAIN_LIMIT_MIN/MAX` exists but not always logged |

### 2.5 Persistence

| Question | Currently Answerable? | Evidence |
|----------|----------------------|----------|
| Was "save to memory" commanded? | ❌ No | **No save-to-memory functionality implemented** |
| What exactly was saved? | ❌ No | N/A - not implemented |
| Confirm nothing saved without command? | ❌ No | Gains applied to `currentPidProfile` directly, risk of accidental persist |

---

## 3. Event Log Schema (Canonical)

### 3.1 Current Debug Channel Mapping

From [autotune_debug.h](../../src/main/flight/autotune_v2/autotune_debug.h#L163-L172):

| Channel | Index | Content | Scale | Notes |
|---------|-------|---------|-------|-------|
| `debug[0]` | AUTOTUNE_DEBUG_STATE | Master state | 0-8 enum | ✅ Correct |
| `debug[1]` | AUTOTUNE_DEBUG_AXIS | Current axis | 0/1/2 | ✅ Correct |
| `debug[2]` | AUTOTUNE_DEBUG_REASON | Reason code | enum | ⚠️ Used inconsistently |
| `debug[3]` | AUTOTUNE_DEBUG_DECISION | Decision/progress | varies | ⚠️ Overloaded meaning |
| `debug[4]` | AUTOTUNE_DEBUG_OVERSHOOT | Overshoot | ×10 (%) | ✅ Correct |
| `debug[5]` | AUTOTUNE_DEBUG_GAIN_P | P gain value | direct | ✅ Correct |
| `debug[6]` | AUTOTUNE_DEBUG_GAIN_D | D gain value | direct | ✅ Correct |
| `debug[7]` | AUTOTUNE_DEBUG_GAIN_F | F gain value | direct | ✅ Correct |

**DISCREPANCY:** The PRD (Section 12) specifies:
- `debug[5]` = Reason code
- `debug[6]` = Event validity
- `debug[7]` = Axis×100 + iter

But the code uses:
- `debug[2]` = Reason code (AUTOTUNE_DEBUG_REASON)
- `debug[5-7]` = P/D/F gains

**Recommendation:** Align PRD documentation with actual implementation OR update code.

### 3.2 Required Event Schema (Not Currently Implemented)

#### A) State Transition Event (Required) ❌ NOT LOGGED

```c
typedef struct {
    timeUs_t timestamp;
    autotuneState_e previousState;
    autotuneState_e nextState;
    autotuneReason_e transitionReason;
    uint8_t phaseIndex;           // For iteration tracking
    uint8_t axis;
    int16_t keyMetricSnapshot[4]; // [overshoot, lag, rebound, noise]
    uint8_t gatingFlags;          // Bit flags for metric_valid, maneuver_detected, etc.
} stateTransitionEvent_t;
```

**Current Gap:** Only `AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_STATE, newState)` is called in `transitionToState()`. Previous state, reason, and key metrics at transition not logged.

#### B) Criteria Evaluation Snapshot (Strongly Recommended) ❌ NOT LOGGED

```c
typedef struct {
    autotuneState_e state;
    uint8_t criterionId;          // Which check: overshoot/rebound/lag/noise
    int16_t metricValue;          // Actual measured value
    int16_t threshold;            // Threshold used for comparison
    bool decisionOutcome;         // Pass/fail
    float windowMean;             // If windowed stats used
    float windowStd;
} criteriaEvalEvent_t;
```

**Current Gap:** Decision logic in `pdRatioSeekDecision()`, `pdScaleUpDecision()`, `fTuneDecision()` computes internally but doesn't log threshold comparisons.

#### C) Parameter Update Event (Required) ⚠️ PARTIAL

```c
typedef struct {
    timeUs_t timestamp;
    autotuneState_e state;
    uint8_t axis;
    uint8_t paramType;            // P/D/F
    float oldValue;
    float newValue;
    bool clampApplied;
    float clampLimit;
    float qualityBefore;          // Optional: metric before
    float qualityAfter;           // Optional: metric after
} parameterUpdateEvent_t;
```

**Current State:** New P/D/F values logged via `AUTOTUNE_DEBUG_SET`, but:
- ❌ Old value not captured
- ❌ Clamp application not logged
- ❌ Quality metric before/after not logged

#### D) Abort/Revert Event (Required) ❌ NOT LOGGED

```c
typedef struct {
    timeUs_t timestamp;
    autotuneReason_e abortReason;
    autotuneState_e stateAtAbort;
    float originalP[3];           // Original values for R/P/Y
    float originalD[3];
    float originalF[3];
    float revertedP[3];           // Values after revert
    float revertedD[3];
    float revertedF[3];
    bool revertComplete;          // Confirm match
} abortRevertEvent_t;
```

**Current State:** `autotuneAbort()` contains `// TODO: Restore original gains if needed` - **revert not implemented**.

#### E) Save-to-Memory Event (Required) ❌ NOT IMPLEMENTED

```c
typedef struct {
    timeUs_t timestamp;
    bool explicitCommandReceived; // Must be true
    float savedP[3];
    float savedD[3];
    float savedF[3];
    uint32_t checksum;
    uint8_t firmwareVersion;
} saveToMemoryEvent_t;
```

**Current State:** **No save-to-memory command exists.** Gains are applied directly to `currentPidProfile` without explicit save. This is a **BLOCKER** for verifying commanded-only persistence.

---

## 4. Signal Coverage Checklist

### 4.1 State & Control Signals

| Signal | Logged? | Channel | Notes |
|--------|---------|---------|-------|
| State enum | ✅ | debug[0] | AUTOTUNE_DEBUG_STATE |
| Reason code | ⚠️ | debug[2] | Not always set at decision point |
| Phase/iteration index | ❌ | - | eventCount tracked internally but not logged |
| Axis identifier | ✅ | debug[1] | AUTOTUNE_DEBUG_AXIS |
| Loop time (dt) | ❌ | - | Available in gyro.targetLooptime, not logged |
| Substate (WAIT/ANALYZE/APPLY) | ❌ | - | axisTuneState.substate not logged |

### 4.2 Pilot Inputs & Vehicle Response

| Signal | Logged? | Channel | Notes |
|--------|---------|---------|-------|
| Stick commands | ⚠️ | blackbox | Standard blackbox, not autotune-specific |
| Gyro rates | ⚠️ | blackbox | Standard blackbox |
| Setpoint rates | ⚠️ | blackbox | Standard blackbox |
| Throttle | ⚠️ | blackbox | Standard blackbox |
| Motor outputs | ⚠️ | blackbox | Standard blackbox |
| Battery voltage | ⚠️ | blackbox | Standard blackbox |

**Note:** Pilot inputs are available in standard blackbox but not synchronized with autotune debug channels. Cross-referencing requires timestamp alignment.

### 4.3 Metrics Used in Criteria

| Metric | Logged? | Channel | Notes |
|--------|---------|---------|-------|
| Overshoot % | ✅ | debug[4] | ×10 scaling |
| Lag (t50) ms | ❌ | - | Computed in metricsAnalyze but not logged |
| Rebound detected | ❌ | - | hasRebound bool not logged |
| Settling time | ❌ | - | Computed but not logged |
| Noise level | ⚠️ | - | Logged during HOVER tune only |
| Trust score | ❌ | - | trustScore in parameterState not logged |
| Bracket bounds (pLow/pHigh) | ❌ | - | Internal state not logged |

### 4.4 Event Quality Gates

| Gate | Logged? | Evidence |
|------|---------|----------|
| Stick deflection sufficient | ⚠️ | Reason code 11 exists |
| Cross-axis below threshold | ⚠️ | Reason code 12 exists |
| Throttle in band | ⚠️ | Reason code 13 exists |
| Duration valid | ⚠️ | Reason code 14 exists |
| Event validity (summary) | ❌ | PRD expects debug[6]=0/1, not implemented |

### 4.5 Safety Signals

| Signal | Logged? | Channel | Notes |
|--------|---------|---------|-------|
| Saturation/clipping | ❌ | - | Not checked or logged |
| Integrator windup | ❌ | - | Not relevant (I not tuned) |
| Gain clamp activation | ⚠️ | debug[2] | Reason codes 22/23 exist |
| Rebound (oscillation) flag | ⚠️ | debug[2] | Reason code 20 exists |
| Trust depleted | ⚠️ | debug[2] | Reason code 26 exists |

---

## 5. Log Rate & Performance Budget

### 5.1 Current Logging Approach

| Category | Rate | Method | Overhead |
|----------|------|--------|----------|
| Debug channels | Loop rate (4-8kHz) | `DEBUG_SET` macro | ~10 cycles per channel |
| State changes | Event-based | Logged on transition | Negligible |
| Metrics | Per-event | After event detection | Negligible |
| Parameter updates | Per-change | After gain applied | Negligible |

### 5.2 Recommendations

| Category | Recommended Rate | Rationale |
|----------|-----------------|-----------|
| State transitions | Always (event) | Critical for workflow reconstruction |
| Parameter updates | Always (event) | Critical for audit trail |
| Criteria evaluations | Per-decision | Required for decision audit |
| Time series metrics | 50-100 Hz | Sufficient for post-analysis |
| High-verbosity mode | Opt-in only | Prevent loop destabilization |

### 5.3 Performance Safety

**Current implementation is safe:**
- `DEBUG_SET` macro compiles to single store instruction
- 8 debug channels × 2 bytes × 4kHz = 64KB/s blackbox bandwidth
- No additional allocation or computation in hot path

**Proposed additions must:**
- Use existing debug channels or blackbox fields
- Avoid string formatting in PID loop
- Use event-based logging for new structured events
- Gate any expensive logging behind config flag

---

## 6. Python Script Compatibility Review

### 6.1 Existing Scripts

| Script | Location | Purpose |
|--------|----------|---------|
| `analyze_quick.py` | [bb logs/analyze_quick.py](../../bb%20logs/analyze_quick.py) | Quick CSV analysis |
| `plot_hover_effectiveness.py` | [bb logs/plot_hover_effectiveness.py](../../bb%20logs/plot_hover_effectiveness.py) | Hover tune visualization |

### 6.2 Field Assumptions in Scripts

From [analyze_quick.py](../../bb%20logs/analyze_quick.py#L1-L30):

```python
df = pd.read_csv('...csv', skiprows=147, low_memory=False)
df['time_s'] = pd.to_numeric(df['time'], errors='coerce') / 1e6
```

**Assumptions:**
- ✅ 147-row header skip (correct per PRD)
- ✅ `time` column exists and is in microseconds
- ✅ `debug[0]` through `debug[7]` columns exist

**Field usage:**
- `debug[0]` = State
- `debug[7]` = Reason (but code uses `debug[2]`)

**DISCREPANCY:** Scripts expect `debug[7]` for reason codes, but code puts reason in `debug[2]`.

From [plot_hover_effectiveness.py](../../bb%20logs/plot_hover_effectiveness.py#L12-L23):

```python
hover['dterm_lpf1'] = hover['debug[2]']
hover['motor_rms'] = hover['debug[3]'] / 10.0
hover['gyro_rms'] = hover['debug[4]'] / 10.0
hover['p_rms'] = hover['debug[5]'] / 10.0
hover['d_rms'] = hover['debug[6]'] / 10.0
hover['reason'] = hover['debug[7]']
```

**Problem:** This script assumes state 1 (HOVER_LOCK) uses different channel meanings:
- debug[2] = D-term LPF1 (Hz)
- debug[3-6] = RMS values

But code uses persistent channel meanings across all states. **This script will produce garbage for current implementation.**

### 6.3 Recommendations for Scripts

1. **Add schema version detection:**
   ```python
   def detect_log_version(df):
       # Check for expected value ranges to infer schema
       if df['debug[0]'].max() <= 10:  # State enum
           return "v2"
       return "unknown"
   ```

2. **Validate required fields before analysis:**
   ```python
   REQUIRED_FIELDS = ['time', 'debug[0]', 'debug[1]', 'debug[2]', ...]
   for field in REQUIRED_FIELDS:
       if field not in df.columns:
           raise ValueError(f"Missing required field: {field}")
   ```

3. **Update scripts to use correct channel mapping:**
   ```python
   # Correct mapping per autotune_debug.h
   STATE = 'debug[0]'
   AXIS = 'debug[1]'
   REASON = 'debug[2]'  # NOT debug[7]
   DECISION = 'debug[3]'
   OVERSHOOT = 'debug[4]'
   GAIN_P = 'debug[5]'
   GAIN_D = 'debug[6]'
   GAIN_F = 'debug[7]'
   ```

4. **Produce state-annotated time series:**
   ```python
   def plot_state_annotated(df):
       fig, axes = plt.subplots(4, 1, sharex=True)
       # Annotate with state background colors
       for state_val, state_name in STATE_NAMES.items():
           mask = df['debug[0]'] == state_val
           for ax in axes:
               ax.fill_between(df.loc[mask, 'time_rel'], 
                              ax.get_ylim()[0], ax.get_ylim()[1],
                              alpha=0.2, label=state_name)
   ```

---

## 7. Gap Analysis & Recommendations

### 7.1 Critical Gaps (Blockers)

| ID | Gap | Impact | Fix Location | Overhead | Validation |
|----|-----|--------|--------------|----------|------------|
| G-001 | No save-to-memory implementation | Cannot verify commanded-only persistence | `autotune_core.c`, CLI | Medium | Add CLI "autotune save" command with log event |
| G-002 | Abort does not restore original gains | Cannot prove safety revert | `autotuneAbort()` in `autotune_core.c` | Low | Implement TODO, log restore event |
| G-003 | Reason codes not documented to match code | Analysis scripts will misinterpret | PRD Section 12 vs `autotune_debug.h` | Low | Update PRD or code |
| G-004 | Parameter old value not logged | Cannot reconstruct exact gain history | `applyPGain()`, `applyScale()`, `applyF()` | Low | Add old value to debug event |

### 7.2 Major Gaps

| ID | Gap | Impact | Fix Location | Overhead | Validation |
|----|-----|--------|--------------|----------|------------|
| G-005 | Lag metric not logged | Cannot analyze response speed tuning | `statePdScaleUpUpdate()` | Low | Add debug channel or pack with overshoot |
| G-006 | Rebound flag not logged | Cannot audit oscillation detection | `pdRatioSeekDecision()` | Low | Add to reason code or decision field |
| G-007 | Bracket bounds not logged | Cannot debug Newton algorithm | `pdRatioSeekDecision()` | Medium | Add debug mode for verbose bracket log |
| G-008 | Event validity not in debug[6] | PRD mismatch, scripts expect it | All state updates | Low | Add `AUTOTUNE_DEBUG_SET(6, eventValid)` |

### 7.3 Minor Gaps

| ID | Gap | Impact | Fix Location | Overhead | Validation |
|----|-----|--------|--------------|----------|------------|
| G-009 | Iteration counter not logged | Cannot track phase progress | State handlers | Low | Use packed `axis*100+iter` in debug[7] |
| G-010 | Trust score not logged | Cannot analyze step size adaptation | Rollback module | Low | Optional verbose mode |
| G-011 | Substate not logged | Cannot debug event detection | State handlers | Low | Pack with state or use reason codes |
| G-012 | No log schema version | Future compatibility risk | Debug header | Low | Add version byte to first debug sample |

---

## 8. Issue List (Handoff-Ready)

### Issue OBS-001: Save-to-Memory Command Missing

| Field | Value |
|-------|-------|
| **ID** | OBS-001 |
| **Severity** | 🔴 Blocker |
| **Missing Item** | Save-to-memory command with audit logging |
| **Impact** | Cannot prove parameters saved only when explicitly commanded. Risk of accidental persistence. |
| **Proposed Fix** | 1. Add CLI command `autotune save` or mode box action<br>2. Log save event: timestamp, params saved, explicit flag<br>3. Only call `saveConfigAndNotify()` on explicit command |
| **Code Location** | New: `autotune_core.c::autotuneSave()`, CLI command in `cli/cli.c` |
| **Example Log** | `AT_SAVE: t=12345678, cmd=EXPLICIT, P=[52,54,45], D=[28,32,30], F=[100,100,0]` |
| **Validation** | 1. Complete autotune, power cycle - gains should NOT persist<br>2. Run `autotune save`, power cycle - gains SHOULD persist<br>3. Log shows SAVE event with explicit=true |

---

### Issue OBS-002: Abort Does Not Restore Original Gains

| Field | Value |
|-------|-------|
| **ID** | OBS-002 |
| **Severity** | 🔴 Blocker |
| **Missing Item** | Gain restoration on abort/revert |
| **Impact** | Cannot prove safety revert occurred. Pilot may fly with unstable gains after abort. |
| **Proposed Fix** | 1. Implement TODO in `autotuneAbort()`<br>2. Restore original P/D/F for all axes<br>3. Log revert event with before/after values |
| **Code Location** | `autotune_core.c::autotuneAbort()` line ~1740 |
| **Example Log** | `AT_ABORT: t=12345678, reason=ABORT_SWITCH, restored=[P:45,D:30,F:100]` |
| **Validation** | 1. Start autotune, let P change, abort<br>2. Verify `currentPidProfile->pid[x].P` equals original<br>3. Log shows ABORT with restored values matching pre-autotune |

---

### Issue OBS-003: Parameter Update Missing Old Value

| Field | Value |
|-------|-------|
| **ID** | OBS-003 |
| **Severity** | 🔴 Blocker |
| **Missing Item** | Old parameter value in update log |
| **Impact** | Cannot reconstruct exact gain history. Cannot compute step sizes from log. |
| **Proposed Fix** | Modify `applyPGain()`, `applyScale()`, `applyF()` to log old→new values |
| **Code Location** | `autotune_core.c` lines 363-366, 684-691, 1025 |
| **Example Log** | Use debug channels: `debug[5]=newP, debug[6]=oldP` during update events |
| **Validation** | Log analysis can show "P changed from 45 to 48" not just "P is now 48" |

---

### Issue OBS-004: Reason Code Discrepancy PRD vs Code

| Field | Value |
|-------|-------|
| **ID** | OBS-004 |
| **Severity** | 🟡 Major |
| **Missing Item** | Consistent reason code definitions |
| **Impact** | Scripts use wrong channel. LOG_ANALYSIS.md documents codes that don't exist. |
| **Proposed Fix** | 1. Update PRD Section 12 debug format to match `autotune_debug.h`<br>2. Update LOG_ANALYSIS.md reason codes to match `autotuneReason_e` enum<br>3. Fix Python scripts to use `debug[2]` for reason |
| **Code Location** | PRD.md Section 12, LOG_ANALYSIS.md Section 3.4, Python scripts |
| **Example** | PRD says "1301 = P increased", code uses `AUTOTUNE_REASON_DECISION_INCREASE = 3` |
| **Validation** | Scripts correctly parse and report reason codes |

---

### Issue OBS-005: Lag Metric Not Logged

| Field | Value |
|-------|-------|
| **ID** | OBS-005 |
| **Severity** | 🟡 Major |
| **Missing Item** | Lag (t50) metric in debug output |
| **Impact** | Cannot analyze PD_SCALE_UP or F_TUNE effectiveness |
| **Proposed Fix** | Pack lag into debug channel: `debug[4] = overshoot*10 + (lag*10 << 8)` or use debug[3] |
| **Code Location** | `statePdScaleUpUpdate()`, `stateFTuneUpdate()` after metrics compute |
| **Example Log** | `debug[4] = lagMs * 10` (separate from overshoot) |
| **Validation** | Plot shows lag decreasing during SCALE_UP phase |

---

### Issue OBS-006: Event Validity Not Logged

| Field | Value |
|-------|-------|
| **ID** | OBS-006 |
| **Severity** | 🟡 Major |
| **Missing Item** | Event validity flag in expected channel |
| **Impact** | PRD expects `debug[6]=0/1`, scripts may assume this |
| **Proposed Fix** | Current code uses `debug[6]` for D gain. Either update PRD or move event validity elsewhere. |
| **Code Location** | All state handlers after `qualityGatesPassed` check |
| **Example Log** | Log `eventData->qualityGatesPassed` to agreed channel |
| **Validation** | Count of valid vs invalid events matches algorithm behavior |

---

### Issue OBS-007: Transition Reason Not Always Set

| Field | Value |
|-------|-------|
| **ID** | OBS-007 |
| **Severity** | 🟡 Major |
| **Missing Item** | Reason code set at every state transition |
| **Impact** | Some transitions have stale reason codes |
| **Proposed Fix** | Ensure `runtime.reasonCode` set before every `transitionToState()` call |
| **Code Location** | All `transitionToState()` call sites in `autotune_core.c` |
| **Example Log** | Every state change shows meaningful reason code |
| **Validation** | No consecutive samples with different state but same reason |

---

### Issue OBS-008: Rebound Flag Not Logged

| Field | Value |
|-------|-------|
| **ID** | OBS-008 |
| **Severity** | 🟡 Major |
| **Missing Item** | `hasRebound` metric in debug output |
| **Impact** | Cannot verify oscillation detection triggered rollback |
| **Proposed Fix** | Add rebound flag to reason code or decision field when detected |
| **Code Location** | `pdRatioSeekDecision()` and `pdScaleUpDecision()` when `metrics->hasRebound` |
| **Example Log** | Reason code includes ROLLBACK_OSCILLATION (20) when rebound detected |
| **Validation** | Log shows rollback reason correlates with visible oscillation in gyro trace |

---

### Issue OBS-009: Iteration Counter Not Logged

| Field | Value |
|-------|-------|
| **ID** | OBS-009 |
| **Severity** | 🟢 Minor |
| **Missing Item** | Phase iteration count in debug output |
| **Impact** | Cannot track convergence progress per-axis |
| **Proposed Fix** | Use `debug[3]` for packed `axis*100 + eventCount` as PRD suggests |
| **Code Location** | State handler update functions after eventCount increment |
| **Example Log** | `debug[3] = currentAxisIndex * 100 + axis->eventCount` |
| **Validation** | Can count events per axis from log |

---

### Issue OBS-010: Script Channel Mapping Incorrect

| Field | Value |
|-------|-------|
| **ID** | OBS-010 |
| **Severity** | 🟢 Minor |
| **Missing Item** | Scripts aligned with actual debug channel layout |
| **Impact** | `plot_hover_effectiveness.py` produces incorrect output |
| **Proposed Fix** | Update scripts to use `REASON=debug[2]`, `GAIN_P=debug[5]`, etc. |
| **Code Location** | `bb logs/analyze_quick.py`, `bb logs/plot_hover_effectiveness.py` |
| **Example** | Change `hover['reason'] = hover['debug[7]']` to `hover['debug[2]']` |
| **Validation** | Scripts produce correct state/reason correlation plots |

---

## 9. Non-Negotiable Standards Verification

| Standard | Status | Evidence |
|----------|--------|----------|
| State transitions can be reconstructed with timestamps and reason codes | ⚠️ Partial | State logged, reason sometimes stale |
| Parameter updates can be tied to state/phase and explained | ❌ Fail | Old value not logged, phase/iteration not logged |
| Abort/revert behavior can be proven from logs | ❌ Fail | Revert not implemented, no revert log event |
| Save-to-memory actions can be proven to be commanded-only | ❌ Fail | No save command exists |
| Criteria used for transitions are auditable at decision time | ⚠️ Partial | Overshoot logged, thresholds/gates not logged |

---

## 10. Appendix: Recommended Debug Channel Layout

### Proposed Final Layout (Preserving Backward Compatibility Where Possible)

| Channel | Name | Content | Scale | Notes |
|---------|------|---------|-------|-------|
| debug[0] | STATE | Master state | 0-9 enum | Keep as-is |
| debug[1] | AXIS | Current axis | 0/1/2 | Keep as-is |
| debug[2] | REASON | Reason code | enum | Keep as-is, ensure always current |
| debug[3] | PROGRESS | Axis×100 + eventCount | packed | Update to match PRD |
| debug[4] | OVERSHOOT | Overshoot % ×10 | scaled | Keep as-is |
| debug[5] | GAIN_P | Current P gain | direct | Keep as-is |
| debug[6] | GAIN_D | Current D gain | direct | Keep as-is |
| debug[7] | METRIC2 | Lag ×10 OR F gain | scaled/direct | Context-dependent |

**Alternative: Add Transition Event Channel**

For major transitions, briefly override channels 3-7 with structured transition data:
- debug[3] = previousState × 16 + newState
- debug[4] = reasonCode
- debug[5] = keyMetric1 (overshoot)
- debug[6] = keyMetric2 (lag or noise)
- debug[7] = gatingFlags

Then resume normal logging. Scripts detect transitions by state change in debug[0].

---

*End of Logging & Observability Assessment*

*Document Checksum: Review required before implementation begins*
