# Autotune V2 Test Results Validation Report

**Version:** 1.2  
**Created:** January 24, 2026  
**Updated:** January 25, 2026  
**Role:** Test Validation Expert Engineer  
**Status:** 🔄 In Progress - Core Bugs Fixed, Tuning Working

---

## 1. Executive Summary

### 1.1 What Was Tested
| Field | Value |
|-------|-------|
| Test Date | January 25, 2026 |
| Test Batch IDs | phase_1_baseline, phase_2_hover_then_arm_autotune, phase_2_hover_followed_by_throttle_blip_and_flips, phase_2_repeat_1, phase_2_repeat_2 |
| Firmware Version | 2026.6.0-alpha |
| Config Target | BETAFPVG473 |
| Total Runs | 5 |

### 1.2 Overall Outcome

| Category | Status |
|----------|--------|
| State Progression | ✅ Working (0→1→2→3→4→5 verified) |
| Criteria-Based Exits | ✅ Working (overshoot-based P adjustments verified) |
| Tuning Effectiveness | 🔄 In Progress (Roll axis tuning verified, Pitch pending) |
| Cross-Axis Decoupling | ⏳ Pending (need multi-axis flight) |
| Abort/Revert | ✅ Working (switch abort tested) |
| Persistence Behavior | ⏳ Pending |
| **Overall** | 🟢 **Core Functionality Verified** |

### 1.3 Top Failures (All Fixed)

| Priority | Issue | Root Cause | Status |
|----------|-------|------------|--------|
| P0 | THROTTLE_SWEEP never advances | Throttle normalization wrong | ✅ FIXED |
| P1 | Overshoot calculation wrong (122-194% reported) | Using final setpoint instead of peak setpoint | ✅ FIXED |
| P2 | Wiggle feedback too subtle | 30°/s amplitude hard to perceive | ✅ FIXED (60°/s) |
| P2 | Wiggle asymmetric (drone drifts right) | Half-sine bump caused net displacement | ✅ FIXED (symmetric) |

### 1.4 Latest Flight Test Results (phase_2_repeat_2)

| Metric | Value | Status |
|--------|-------|--------|
| Flight Duration | 63.4s | ✅ |
| State Progression | IDLE→HOVER_LOCK→THROTTLE_SWEEP→NOISE_CONFIRM→PD_RATIO_SEEK→PD_SCALE_UP | ✅ |
| Overshoot Range | 2.9% - 9.6% | ✅ In target band |
| Overshoot Mean | 6.8% | ✅ Centered in 5-10% target |
| P Gain Changes | 45→42→41→40 | ✅ Converging |
| Axis Completed | Roll only | 🔄 Need longer flight |

---

## 1.5 Bugs Found & Fixed (January 25, 2026)

### BUG-001: Throttle Normalization Error (CRITICAL)

**Symptom:** State machine stuck in THROTTLE_SWEEP (state 2) indefinitely despite throttle blips.

**Root Cause Analysis:**
```c
// WRONG - rcCommand[THROTTLE] is 1000-2000, not 0-1000
const float throttle = rcCommand[THROTTLE] / 1000.0f;  // Gives 1.0 to 2.0!

// Sweep detection thresholds
#define SWEEP_HIGH_THROTTLE  0.50f  // Always true when throttle >= 1.0
#define SWEEP_LOW_THROTTLE   0.30f  // Never true when throttle >= 1.0
```

**Impact:**
- `hasSeenHighThrottle` was always immediately true (any throttle value > 0.5)
- `hasSeenLowThrottle` was never true (stick bottom = 1000/1000 = 1.0 > 0.3)
- Sweep count never incremented → state never advanced

**Fix Applied:**
```c
// CORRECT - subtract 1000 first to get 0-1000 range, then divide
const float throttle = (rcCommand[THROTTLE] - 1000) / 1000.0f;  // Gives 0.0 to 1.0
```

**Files Modified:** `autotune_core.c` (8 locations)

### BUG-002: Wiggle Amplitude Too Low

**Symptom:** User reported wiggle feedback hard to see during flight.

**Fix:** Changed `WIGGLE_AMPLITUDE_DPS` from 30.0f to 60.0f in `autotune_feedback.c`

### BUG-003: Overshoot Calculation Error (CRITICAL)

**Symptom:** Overshoot values reported as 122-194% when actual overshoot was ~17-22%.

**Root Cause Analysis:**
```c
// WRONG - using final setpoint (near zero after stick release)
float finalSetpoint = 0.0f;
for (int i = count - avgCount; i < count; i++) {
    finalSetpoint += fabsf(setpoint[i]);
}
// If peak gyro = 850, final setpoint = 50:
// overshoot = (850 - 50) / 50 * 100 = 1600% ← WRONG!
```

**Correct Approach:**
```c
// Find peak setpoint (max commanded rate during maneuver)
float peakSetpoint = 0.0f;
for (uint16_t i = 0; i < count; i++) {
    if (fabsf(setpoint[i]) > peakSetpoint) {
        peakSetpoint = fabsf(setpoint[i]);
    }
}
// If peak gyro = 850, peak setpoint = 800:
// overshoot = (850 - 800) / 800 * 100 = 6.25% ← CORRECT!
```

**Files Modified:** `autotune_metrics.c` - `autotuneMetricsComputeOvershoot()`

**Verification:** After fix, overshoot readings are 2.9-9.6% (mean 6.8%), correctly within target band.

### BUG-004: Asymmetric Wiggle Feedback (Drift)

**Symptom:** Drone drifts right during hover when wiggle feedback plays.

**Root Cause:** The BUMP pattern element used a half-sine wave (0 → +peak → 0), which creates net displacement. Over multiple wiggles, this accumulates and causes drift.

**Fix:** Changed all feedback waveforms to symmetric full sine waves:
```c
// Before: Half-sine (asymmetric, causes drift)
feedbackState.currentOffset = sin_approx(phase * M_PIf) * WIGGLE_AMPLITUDE_DPS;

// After: Full sine (symmetric, zero net displacement)
feedbackState.currentOffset = sin_approx(phase * 2.0f * M_PIf) * WIGGLE_AMPLITUDE_DPS;
```

**Files Modified:** `autotune_feedback.c` - waveform generation and timing constants

---

## 2. Test Inventory & Organization

### 2.1 Proposed Folder Structure

```
betaflight/
├── tests/
│   └── autotune_v2/
│       └── <YYYY-MM-DD>/
│           └── <test_suite>/
│               └── <run_id>/
│                   ├── raw_logs/
│                   │   └── btfl_<timestamp>.bbl
│                   ├── processed/
│                   │   └── btfl_<timestamp>.bbl.csv
│                   ├── plots/
│                   │   ├── state_timeline.png
│                   │   ├── gains_progression.png
│                   │   ├── metrics_trace.png
│                   │   └── cross_axis_check.png
│                   ├── notes.md
│                   └── summary.json
```

### 2.2 Test Run Registry

| Run ID | File Name(s) | FW Build | Config | Environment | Pilot Notes | Save Commanded? | Status |
|--------|-------------|----------|--------|-------------|-------------|-----------------|--------|
| _[001]_ | _[filename]_ | _[commit]_ | _[snapshot]_ | _[conditions]_ | _[notes]_ | _[Y/N]_ | _[pending]_ |

### 2.3 Required Metadata per Run

For each flight test run, collect:

```markdown
## Run: [RUN_ID]

### Aircraft
- Quad model: 
- AUW (All-Up Weight): g
- Props: 
- Battery: mAh, S
- Motor KV: 

### Firmware
- Build date: 
- Commit: 
- Target: BETAFPVG473

### Initial Configuration
- Roll:  P=, I=, D=, F=
- Pitch: P=, I=, D=, F=
- Yaw:   P=, I=, D=, F=
- D-term LPF1: Hz
- D-term LPF2: Hz
- Gyro LPF1: Hz
- Gyro LPF2: Hz

### Environment
- Wind: mph
- Temperature: °F/°C
- Surface: 
- Obstacles: 

### Session
- Autotune switch: AUX
- Save-to-memory commanded: Y/N
- Abort triggered: Y/N (reason)
- Pilot subjective feel: 

### Files
- Raw BBL: 
- Exported CSV: 
- Export method: Blackbox Explorer / blackbox_decode
```

---

## 3. Claims Under Test

### 3.1 State Progression Claims

| Claim ID | Claim | Verification Method | Status |
|----------|-------|---------------------|--------|
| SP-01 | IDLE → HOVER_LOCK on activation + armed | Check debug[0] transition 0→1 | ⏳ |
| SP-02 | HOVER_LOCK achieved within 10s | Check duration in state 1 | ⏳ |
| SP-03 | THROTTLE_SWEEP entered after hover lock | Check debug[0] transition 1→2 | ⏳ |
| SP-04 | NOISE_CONFIRM entered or skipped appropriately | Check transition 2→3 or 2→4 | ⏳ |
| SP-05 | PD_RATIO_SEEK entered for each enabled axis | Check state=4, axis=0,1,2 | ⏳ |
| SP-06 | PD_SCALE_UP follows PD_RATIO_SEEK | Check transition 4→5 | ⏳ |
| SP-07 | F_TUNE entered if tuneFeedforward=1 | Check state=6 present | ⏳ |
| SP-08 | COMPLETE state reached | Check debug[0]=8 | ⏳ |
| SP-09 | No state skips occur | Verify monotonic progression | ⏳ |
| SP-10 | Abort returns to IDLE with reason code 30-35 | Check abort transitions | ⏳ |

### 3.2 Criteria & Metrics Claims

| Claim ID | Claim | Verification Method | Status |
|----------|-------|---------------------|--------|
| CM-01 | Event quality gates reject bad maneuvers | Check reason codes 11-17 | ⏳ |
| CM-02 | Overshoot metric computed and logged | Check debug[4] values | ⏳ |
| CM-03 | Overshoot converges to 5-10% band | Analyze debug[4] trend | ⏳ |
| CM-04 | Rollback triggered on oscillation/rebound | Check reason codes 20-21 | ⏳ |
| CM-05 | Gain limits enforced | Check reason codes 22-23 | ⏳ |
| CM-06 | Event count limits work | Check reason code 24 | ⏳ |

### 3.3 Tuning Effectiveness Claims

| Claim ID | Claim | Verification Method | Status |
|----------|-------|---------------------|--------|
| TE-01 | P gains change during PD_RATIO_SEEK | Compare debug[5] before/after | ⏳ |
| TE-02 | P and D scale together in PD_SCALE_UP | Check proportional changes | ⏳ |
| TE-03 | F gains change during F_TUNE | Compare debug[7] before/after | ⏳ |
| TE-04 | Final tune improves response | Pilot confirmation + metrics | ⏳ |
| TE-05 | Complete within 30 events / 1 battery | Count total events | ⏳ |

### 3.4 Cross-Axis Decoupling Claims

| Claim ID | Claim | Verification Method | Status |
|----------|-------|---------------------|--------|
| CD-01 | Roll tuning doesn't degrade Pitch | Compare Pitch metrics pre/post Roll | ⏳ |
| CD-02 | Pitch tuning doesn't degrade Roll | Compare Roll metrics pre/post Pitch | ⏳ |
| CD-03 | Cross-axis contamination rejects events | Check reason code 12 | ⏳ |

### 3.5 Persistence & Abort Claims

| Claim ID | Claim | Verification Method | Status |
|----------|-------|---------------------|--------|
| PA-01 | Abort restores pre-autotune gains | Compare gains after abort | ⏳ |
| PA-02 | Disarm triggers abort | Check reason code 31 | ⏳ |
| PA-03 | Switch-off triggers abort | Check reason code 30 | ⏳ |
| PA-04 | Timeout triggers abort after 3 minutes | Check reason code 33 | ⏳ |
| PA-05 | Save-to-memory only when commanded | Verify persistence behavior | ⏳ |

---

## 4. Analysis Procedure

### 4.1 Prerequisites

```bash
# Required Python packages
pip install pandas numpy matplotlib

# Required tools
# - Blackbox Explorer (for BBL → CSV export)
# - OR blackbox_decode CLI tool
```

### 4.2 Log Export Procedure

1. Open Blackbox Explorer
2. Load `.bbl` or `.bfl` file
3. File → Export to CSV
4. Save to `tests/autotune_v2/<date>/<run_id>/processed/`

**CRITICAL:** Blackbox CSV has **147-row header**. Always use `skiprows=147` in pandas.

### 4.3 Analysis Scripts

#### Script 1: Quick State Analysis (`analyze_quick.py`)

**Location:** `bb logs/analyze_quick.py` (copy to `tests/autotune_v2/scripts/`)

**Input:** Exported CSV file path (hardcoded, needs update)

**Usage:**
```bash
python analyze_quick.py
```

**Outputs:**
- Reason code distribution
- State distribution
- Transition timestamps
- Debug values at transitions

#### Script 2: Hover Effectiveness Plot (`plot_hover_effectiveness.py`)

**Location:** `bb logs/plot_hover_effectiveness.py`

**Input:** Exported CSV file path (hardcoded)

**Outputs:**
- `hover_effectiveness.png` - Motor RMS vs D-term filter plot
- D-term LPF1 transitions with timestamps
- Motor RMS statistics by filter setting

### 4.4 Recommended Analysis Sequence

For each test run:

1. **Export log** to CSV using Blackbox Explorer
2. **Verify header skip** is correct:
   ```python
   import pandas as pd
   df = pd.read_csv('log.csv', skiprows=147)
   assert df['time'].iloc[0] < 1e6, "Header skip incorrect!"
   ```
3. **Run state timeline extraction:**
   ```python
   # Extract state transitions
   df['state_prev'] = df['debug[0]'].shift(1)
   transitions = df[df['debug[0]'] != df['state_prev']]
   print(transitions[['time', 'debug[0]', 'debug[1]', 'debug[2]']].to_string())
   ```
4. **Generate state timeline plot** (see §5.A)
5. **Extract gain changes:**
   ```python
   # P gain changes
   df['p_prev'] = df['debug[5]'].shift(1)
   p_changes = df[df['debug[5]'] != df['p_prev']]
   print(p_changes[['time', 'debug[0]', 'debug[5]', 'debug[2]']].to_string())
   ```
6. **Compute metrics trends** for convergence analysis
7. **Check cross-axis effects** during single-axis tuning phases

### 4.5 Segmenting Maneuvers/Phases

**Timestamps:** Use `time` column (microseconds from log start)

**Convert to seconds:**
```python
df['time_s'] = df['time'] / 1e6
df['time_rel'] = df['time_s'] - df['time_s'].min()
```

**Phase segmentation by state:**
```python
# Filter by state
hover_lock = df[df['debug[0]'] == 1]
throttle_sweep = df[df['debug[0]'] == 2]
pd_ratio_seek = df[df['debug[0]'] == 4]
pd_scale_up = df[df['debug[0]'] == 5]
f_tune = df[df['debug[0]'] == 6]
```

**Maneuver windows:** Identified by reason code 1 (EVENT_DETECTED) followed by metric computation.

---

## 5. Evidence & Plots (State-Aware)

### 5.A State Timeline Overlay

**Purpose:** Visualize state progression with reason codes and phase/iteration index

**Required elements:**
- Y-axis: State enum (0-8)
- X-axis: Time (seconds)
- Overlay: Reason codes at transitions
- Markers: Maneuver windows (reason=1)
- Color coding by axis (debug[1])

**Script template:**
```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('log.csv', skiprows=147, low_memory=False)
df['time_s'] = df['time'] / 1e6
df['time_rel'] = df['time_s'] - df['time_s'].min()

fig, ax = plt.subplots(figsize=(14, 6))

# State timeline
ax.plot(df['time_rel'], df['debug[0]'], 'b-', linewidth=0.5, alpha=0.7, label='State')

# Mark transitions
df['state_prev'] = df['debug[0]'].shift(1)
transitions = df[df['debug[0]'] != df['state_prev']]
for idx in transitions.index:
    t = df.loc[idx, 'time_rel']
    state = df.loc[idx, 'debug[0]']
    reason = df.loc[idx, 'debug[2]']
    ax.axvline(x=t, color='gray', linestyle=':', alpha=0.5)
    ax.annotate(f'S{state} R{reason}', (t, state), fontsize=6)

ax.set_xlabel('Time (s)')
ax.set_ylabel('State')
ax.set_yticks(range(9))
ax.set_yticklabels(['IDLE', 'HOVER_LOCK', 'THROTTLE_SWEEP', 'NOISE_CONFIRM', 
                   'PD_RATIO', 'PD_SCALE', 'F_TUNE', 'PD_RETUNE', 'COMPLETE'])
ax.grid(True, alpha=0.3)
ax.legend()
plt.title('Autotune V2 State Timeline')
plt.tight_layout()
plt.savefig('state_timeline.png', dpi=150)
```

### 5.B Core Response Signals

**Required plots:**

1. **Commanded vs Measured Rates (per axis)**
   - Columns: `setpoint[0-2]` (gyro rate setpoints), `gyroADC[0-2]` (actual rates)
   - Segment by tuning axis

2. **Actuator Outputs**
   - Columns: `motor[0-3]`
   - Flag saturation (near 0 or 2000)

3. **Battery Voltage**
   - Column: `vbatLatest`
   - Identify sag events that could confound results

### 5.C Criteria/Metric Traces

**Required plots:**

1. **Overshoot progression:**
   ```python
   # debug[4] = overshoot × 10
   ax.plot(df['time_rel'], df['debug[4]'] / 10.0, label='Overshoot %')
   ax.axhline(y=5, color='g', linestyle='--', label='Target Low')
   ax.axhline(y=10, color='r', linestyle='--', label='Target High')
   ```

2. **P/D/F gain evolution:**
   ```python
   ax.plot(df['time_rel'], df['debug[5]'], label='P Gain')
   ax.plot(df['time_rel'], df['debug[6]'], label='D Gain')
   ax.plot(df['time_rel'], df['debug[7]'], label='F Gain')
   ```

3. **Reason code distribution:**
   ```python
   reason_counts = df['debug[2]'].value_counts().sort_index()
   print(reason_counts)
   ```

### 5.D Cross-Axis Regression Checks

**Procedure:**

1. During Roll tuning (axis=0), track Pitch and Yaw error metrics
2. Compare pre-tuning vs post-tuning error RMS on non-active axes
3. Flag if degradation exceeds 20% threshold

**Script template:**
```python
# Segment by active axis
roll_tune = df[(df['debug[0]'].isin([4,5,6])) & (df['debug[1]'] == 0)]
pitch_tune = df[(df['debug[0]'].isin([4,5,6])) & (df['debug[1]'] == 1)]

# Compute error metrics on other axes during roll tuning
if 'gyroADC[1]' in df.columns and 'setpoint[1]' in df.columns:
    roll_tune['pitch_error'] = roll_tune['gyroADC[1]'] - roll_tune['setpoint[1]']
    pitch_error_rms = (roll_tune['pitch_error'] ** 2).mean() ** 0.5
    print(f"Pitch error RMS during Roll tune: {pitch_error_rms:.2f}")
```

---

## 6. Findings

### 6.1 Per-Run Findings

_Template for each test run:_

```markdown
### Run: [RUN_ID]

#### Claim Verdicts
| Claim | Verdict | Evidence |
|-------|---------|----------|
| SP-01 | ✅/❌/⏳ | [timestamp, plot ref] |
| ... | ... | ... |

#### State-Flow Anomalies
- [ ] Skipped states: 
- [ ] Sticky states (>30s same state): 
- [ ] Repeated state entries: 
- [ ] Unexpected transitions: 

#### Criteria Anomalies
- [ ] Premature triggers: 
- [ ] Late triggers: 
- [ ] Noisy triggers: 
- [ ] Inconsistent gating: 

#### Tuning Effectiveness
- Initial: P=, D=, F=
- Final: P=, D=, F=
- Overshoot: % → %
- Direction: Improved/Degraded/Neutral
- Pilot confirmation: 

#### Coupling Issues
- Roll→Pitch effect: 
- Pitch→Roll effect: 
- Yaw coupling: 
```

### 6.2 Aggregate Findings

_To be filled after multiple test runs:_

- **Patterns across runs:**
- **Conditions correlating with failures:**
- **Consistent success scenarios:**

---

## 7. Failure Mode Identification

### 7.1 Failure Classification Categories

| Category | Code | Description |
|----------|------|-------------|
| Workflow/State Progression Bug | WF | State machine logic error |
| Criteria/Metric Evaluation Flaw | CR | Threshold, gate, or metric bug |
| Logging/Observability Gap | LO | Cannot prove what happened |
| Maneuver Inadequacy | MA | Pilot input insufficient |
| Numerical Stability | NS | dt/saturation/precision issue |
| Cross-Axis Coupling | CA | Decoupling violation |
| Persistence/Save Behavior | PS | Memory save incorrect |
| Pilot Interaction / UX | UX | Unclear feedback or behavior |

### 7.2 Failure Registry

| Failure ID | Category | Severity | Run ID | Timestamp | Description | Evidence |
|------------|----------|----------|--------|-----------|-------------|----------|
| _[F-001]_ | _[WF]_ | _[Blocker]_ | _[run]_ | _[time]_ | _[desc]_ | _[plot/log ref]_ |

---

## 8. Countermeasures & Recommendations

### 8.1 Countermeasure Template

For each identified failure:

```markdown
### Failure: [F-ID] - [Title]

**Category:** [WF/CR/LO/MA/NS/CA/PS/UX]

**Immediate Countermeasure:**
- [Lowest-effort mitigation]

**Root-Cause Fix Direction:**
- [Code area likely needing change]
- [Specific file/function if known]

**Verification Step:**
- [Next test to confirm fix]
- [What must change in logs]
```

### 8.2 Countermeasure Categories

| Type | Examples |
|------|----------|
| Gating/Hysteresis | Adjust thresholds, add hysteresis bands |
| Logging Enhancement | Add debug fields, refine reason codes |
| Transition Conditions | Tighten entry/exit criteria |
| Safety Clamps | Add rate limits, gain bounds |
| Maneuver Scripts | Change pilot instructions for better SNR |
| Analysis Script Updates | Fix interpretation of log fields |

---

## 9. Next-Test Proposal (Adaptive Test Loop)

### 9.1 Current Knowledge Gaps

| Gap ID | Description | Hypotheses | Discriminating Test |
|--------|-------------|------------|---------------------|
| _[G-01]_ | _[what we don't know]_ | _[A/B]_ | _[minimal test]_ |

### 9.2 Proposed Next Tests

| Priority | Test ID | Purpose | Duration | Required Conditions |
|----------|---------|---------|----------|---------------------|
| 1 | _[T-xx]_ | _[goal]_ | _[est. time]_ | _[setup requirements]_ |

### 9.3 Test Progression Strategy

1. **First flight:** Basic state progression verification
2. **Second flight:** Complete tuning cycle on single axis
3. **Third flight:** Full multi-axis tuning
4. **Fourth flight:** Abort/revert validation
5. **Fifth flight:** Edge case stress testing

---

## 10. Issue List (Handoff-Ready)

### 10.1 Open Issues

| ID | Severity | Symptom | Evidence | Suspected Cause | Countermeasure | Validation Criteria |
|----|----------|---------|----------|-----------------|----------------|---------------------|
| _[I-001]_ | _[Blocker/Major/Minor]_ | _[symptom]_ | _[run, time, plot]_ | _[cause]_ | _[fix]_ | _[expected change]_ |

### 10.2 Closed Issues

| ID | Resolution | Verified In |
|----|------------|-------------|
| _[I-xxx]_ | _[fix applied]_ | _[run ID]_ |

---

## 11. Observability Assessment

### 11.1 Current Observability Status

| Aspect | Status | Notes |
|--------|--------|-------|
| State reconstruction | ✅ OK | debug[0] provides clear state enum |
| Transition reasons | ✅ OK | debug[2] provides reason codes |
| Gain values | ✅ OK | debug[5,6,7] provide P/D/F |
| Overshoot metric | ✅ OK | debug[4] × 10 |
| Lag metric | 🟡 Partial | Logged in F_TUNE, not elsewhere |
| Rebound detection | 🔴 Gap | Not explicitly logged |
| Event buffer contents | 🔴 Gap | Cannot verify event data |
| Old→New gain values | 🟡 Partial | Can infer from transitions |

### 11.2 Non-Verifiable Claims (With Current Logs)

If any claims cannot be verified, list here with required instrumentation changes:

| Claim | Gap | Required Change |
|-------|-----|-----------------|
| _[claim]_ | _[what's missing]_ | _[add debug field X]_ |

---

## 12. Blockers Requiring Immediate Attention

Per the non-negotiable standards, flag as **Blocker** if:

- [ ] Cannot reconstruct state progression from logs
- [ ] Transitions occur without auditable reasons/metrics
- [ ] Abort/revert cannot be proven correct
- [ ] Tuning changes applied without being logged
- [ ] Cross-axis regressions are severe or uncontrolled
- [ ] Persistence occurs without explicit command (or fails when commanded)

### Current Blockers

_None identified - awaiting first flight test data._

---

## Appendix A: Debug Channel Quick Reference

| Channel | Macro | Content | Scale | Analysis Use |
|---------|-------|---------|-------|--------------|
| debug[0] | AUTOTUNE_DEBUG_STATE | State enum | 0-8 | State timeline |
| debug[1] | AUTOTUNE_DEBUG_AXIS | Axis | 0/1/2 | Per-axis analysis |
| debug[2] | AUTOTUNE_DEBUG_REASON | Reason code | 0-35 | Transition cause |
| debug[3] | AUTOTUNE_DEBUG_DECISION | Decision/delta | varies | Gain change direction |
| debug[4] | AUTOTUNE_DEBUG_OVERSHOOT | Overshoot % | ×10 | Convergence tracking |
| debug[5] | AUTOTUNE_DEBUG_GAIN_P | P gain | direct | Gain evolution |
| debug[6] | AUTOTUNE_DEBUG_GAIN_D | D gain | direct | Gain evolution |
| debug[7] | AUTOTUNE_DEBUG_GAIN_F | F gain | direct | Gain evolution |

## Appendix B: State Enum Reference

| Value | State | Description |
|-------|-------|-------------|
| 0 | IDLE | Inactive, waiting for activation |
| 1 | HOVER_LOCK | Confirming stable hover |
| 2 | THROTTLE_SWEEP | Filter characterization |
| 3 | NOISE_CONFIRM | Validating filter settings |
| 4 | PD_RATIO_SEEK | Finding P/D ratio for damping |
| 5 | PD_SCALE_UP | Scaling P+D for speed |
| 6 | F_TUNE | Feedforward tuning |
| 7 | PD_RETUNE_AFTER_F | Re-validating after F change |
| 8 | COMPLETE | Tuning finished |

## Appendix C: Reason Code Reference

| Range | Category | Codes |
|-------|----------|-------|
| 0-10 | Normal progression | NONE, EVENT_DETECTED, QUALITY_OK, ... |
| 11-19 | Quality gate failures | INSUFFICIENT_DEFLECTION, CROSS_AXIS, ... |
| 20-29 | Safety events | ROLLBACK_OSCILLATION, ROLLBACK_OVERSHOOT, ... |
| 30-35 | Abort conditions | ABORT_SWITCH, ABORT_DISARM, ABORT_FAILSAFE, ... |

See [LOG_ANALYSIS.md](LOG_ANALYSIS.md) §3.4 for complete list.

---

*Document template ready. Update with actual test data after each flight test session.*
