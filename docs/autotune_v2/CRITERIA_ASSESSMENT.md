# Autotune V2 Criteria & Metrics Assessment

**Version:** 1.0  
**Date:** January 23, 2026  
**Role:** Criteria Evaluation Lead Engineer  
**Status:** Initial Assessment

---

## 1. Executive Summary

### Overall Criteria Health: ⚠️ CAUTION

The autotune V2 implementation has a well-structured state machine with clear criteria definitions in code. However, several areas require attention before production use.

### Top 5 Risks

| # | Risk | Severity | Impact |
|---|------|----------|--------|
| 1 | **Hardcoded sample period assumption** in metrics (4000µs) ignores actual loop rate | Major | Incorrect lag/settling calculations on non-250Hz systems |
| 2 | **No minimum sample count gating** before overshoot calculation returns valid result | Major | Metrics can be computed from noisy/incomplete event data |
| 3 | **Rebound detection threshold uses first peak as reference** without noise floor consideration | Major | Noise can trigger false rebound detection |
| 4 | **noiseConfirmIterations static variable** is vulnerable to state machine re-entry | Minor | Potential for stuck state on edge cases |
| 5 | **Filter application is TODO placeholder** - recommendations computed but not applied | Blocker | Filter phase cannot actually tune filters |

### "Most Likely to Cause Field Failures" Shortlist

1. **Filter recommendations not applied** - The throttle sweep will complete but filters won't change
2. **Sample period mismatch** - Lag target comparisons will fail on 8K systems (125µs period vs assumed 4000µs)
3. **Cross-axis coupling** - Single-axis tuning with no explicit axis isolation during maneuvers

---

## 2. Criteria Inventory (by State)

### 2.1 AUTOTUNE_STATE_IDLE

| Criterion Type | Name | Location | Signals | Transitions |
|---------------|------|----------|---------|-------------|
| **Entry** | Switch deactivated OR disarm | `autotuneUpdateActivation()` | `enabled` param | From any state |
| **Exit** | Switch activated + armed | `autotuneUpdateActivation()` | `enabled`, arm state | → HOVER_LOCK |

**Notes:** Clean entry/exit. No criteria issues.

---

### 2.2 AUTOTUNE_STATE_HOVER_LOCK

| Criterion Type | Name | Location | Signals | Transitions |
|---------------|------|----------|---------|-------------|
| **Entry** | Autotune activated | `transitionToState()` | Master state | From IDLE |
| **Exit/Success** | Stable hover for 1 second | `stateHoverLockUpdate()` | `hoverLockStartUs`, sticks, throttle | → THROTTLE_SWEEP |
| **Exit/Failure** | Timeout 10 seconds | `stateHoverLockUpdate()` | `stateEntryTimeUs` | → IDLE |
| **Stability Gate** | Sticks centered | `stateHoverLockUpdate()` | `rcCommand[FD_*]` < 5% | Resets lock timer |
| **Stability Gate** | Throttle stable | `stateHoverLockUpdate()` | Throttle delta < 5% | Resets lock timer |

**Metric Specifications:**

| Metric | Definition | Units | Window | Filtering | Debounce |
|--------|------------|-------|--------|-----------|----------|
| `sticksCentered` | All RC sticks < 25 (5% of ±500) | boolean | Instantaneous | None | 1 second persistence |
| `throttleStable` | \|throttle - lastThrottle\| < 0.05 | boolean | Per-loop | None | 1 second persistence |

**⚠️ Issue CRIT-001:** Throttle normalization uses `rcCommand[THROTTLE] / 1000.0f`, but rcCommand[THROTTLE] range may vary by configuration. Should use `rcData[THROTTLE]` or proper range handling.

---

### 2.3 AUTOTUNE_STATE_THROTTLE_SWEEP

| Criterion Type | Name | Location | Signals | Transitions |
|---------------|------|----------|---------|-------------|
| **Entry** | Hover locked | `transitionToState()` | Master state | From HOVER_LOCK |
| **Exit/Success** | Sweep complete | `autotuneFilterSweepComplete()` | sweepCount, hoverStableStartUs | → NOISE_CONFIRM |
| **Sweep Completion** | 2 sweeps + 2s stable hover | `autotuneFilterSweepComplete()` | `sweepCount >= 2`, hover duration | N/A |

**Metric Specifications:**

| Metric | Definition | Units | Window | Filtering | Debounce |
|--------|------------|-------|--------|-----------|----------|
| `sweepCount` | High→Low→Hover transitions | count | Session | None | Requires full cycle |
| `hasSeenHighThrottle` | throttle ≥ 50% | boolean | Session | None | None |
| `hasSeenLowThrottle` | throttle ≤ 30% | boolean | Session | None | None |
| `currentNoise` | RMS of gyro magnitude variance | deg/s | 32 samples | Circular buffer variance | None |
| `hoverNoiseLevel` | EMA of noise in hover band | deg/s | Continuous | α=0.05 EMA | None |

**⚠️ Issue CRIT-002:** The `measureCurrentNoise()` function computes RMS of deviation from mean over a 32-sample window, but does not account for gyro DC offset changes during maneuvers. This may report high "noise" during intentional stick movements.

**⚠️ Issue CRIT-003:** Noise measurement samples gyro magnitude (`sqrt(x²+y²+z²)`) rather than individual axis noise. This conflates all axes and may not accurately characterize axis-specific filtering needs.

---

### 2.4 AUTOTUNE_STATE_NOISE_CONFIRM

| Criterion Type | Name | Location | Signals | Transitions |
|---------------|------|----------|---------|-------------|
| **Entry** | Filters recommended | `transitionToState()` | Master state | From THROTTLE_SWEEP |
| **Exit/Success** | Noise acceptable | `autotuneFilterNoiseAcceptable()` | `hoverNoiseLevel < 15.0` | → PD_RATIO_SEEK |
| **Exit/Fallback** | Max iterations (5) | `stateNoiseConfirmUpdate()` | `noiseConfirmIterations` | → PD_RATIO_SEEK |
| **Retry** | Noise too high | `stateNoiseConfirmUpdate()` | Noise level | Tighten LPF by 15% |
| **Settling Gate** | 500ms settling time | `stateNoiseConfirmUpdate()` | `stateEntryTimeUs` | Wait before measure |

**Metric Specifications:**

| Metric | Definition | Units | Window | Filtering | Debounce |
|--------|------------|-------|--------|-----------|----------|
| `noiseAcceptable` | hoverNoiseLevel < AUTOTUNE_NOISE_ACCEPTABLE (15.0) | boolean | Instantaneous | EMA from filter module | None |

**❌ Issue CRIT-004 (Blocker):** `autotuneFilterApplyRecommendations()` is a TODO stub. The noise confirm phase measures noise but cannot actually change filters, making this phase ineffective.

**⚠️ Issue CRIT-005:** The static variable `noiseConfirmIterations` persists across function calls but is only reset when the state exits to PD_RATIO_SEEK. If the pilot aborts and restarts, this counter may retain stale values.

---

### 2.5 AUTOTUNE_STATE_PD_RATIO_SEEK

| Criterion Type | Name | Location | Signals | Transitions |
|---------------|------|----------|---------|-------------|
| **Entry** | Noise confirmed | `transitionToState()` | Master state | From NOISE_CONFIRM |
| **Exit/Success** | Target overshoot reached | `pdRatioSeekDecision()` | `overshootPct ≈ targetOvershootPct` | → PD_SCALE_UP |
| **Exit/Limit** | Max events reached | `statePdRatioSeekUpdate()` | `eventCount >= 10` | → PD_SCALE_UP |
| **Exit/Timeout** | 60s per-axis timeout | `checkTimeouts()` | `stateElapsed` | → next axis |
| **Event Quality** | All gates passed | `autotuneEventCheckQuality()` | Event data | Reject/retry |
| **Rebound Safety** | hasRebound detected | `pdRatioSeekDecision()` | `metrics.hasRebound` | Rollback P |
| **Convergence** | \|overshoot - target\| < 2% | `pdRatioSeekDecision()` | Overshoot percentage | ADVANCE |
| **Bracket Narrow** | P_high - P_low < 3 | `pdRatioSeekDecision()` | Bracket state | ADVANCE |

**Metric Specifications:**

| Metric | Definition | Units | Window | Filtering | Debounce |
|--------|------------|-------|--------|-----------|----------|
| `overshootPct` | (peakGyro - finalSetpoint) / finalSetpoint × 100 | % | Event buffer | None | None |
| `hasRebound` | Any peak after first > firstPeak × 5% | boolean | Event buffer | None | None |
| `targetOvershootPct` | overshootTargetLow + aggNorm × (high - low) | % | Config | None | N/A |

**⚠️ Issue CRIT-006:** The overshoot calculation uses `finalSetpoint` from the last 5 samples of the event buffer, but during a step response the gyro may not have settled. This could under-report overshoot if the event ends during transient.

**⚠️ Issue CRIT-007:** Peak detection uses absolute value comparison (`fabsf(gyro[i])`) but overshoot calculation uses `fabsf(setpoint[i])` for final value. Sign handling may be inconsistent for negative-direction maneuvers.

---

### 2.6 AUTOTUNE_STATE_PD_SCALE_UP

| Criterion Type | Name | Location | Signals | Transitions |
|---------------|------|----------|---------|-------------|
| **Entry** | PD ratio found | `transitionToState()` | Master state | From PD_RATIO_SEEK |
| **Exit/Success** | Lag ≤ target | `pdScaleUpDecision()` | `metrics.lagMs` | → F_TUNE or next axis |
| **Exit/Limit** | P at max gain (200) | `pdScaleUpDecision()` | `pState.currentValue` | → F_TUNE or next axis |
| **Exit/Limit** | Max events (10) | `statePdScaleUpUpdate()` | `eventCount` | → F_TUNE or next axis |
| **Safety/Rebound** | hasRebound detected | `pdScaleUpDecision()` | `metrics.hasRebound` | ROLLBACK |
| **Safety/Overshoot** | overshoot > target × 1.5 | `pdScaleUpDecision()` | `overshootPct` | ROLLBACK |

**Metric Specifications:**

| Metric | Definition | Units | Window | Filtering | Debounce |
|--------|------------|-------|--------|-----------|----------|
| `lagMs` | Time to reach 50% of final setpoint | ms | Event buffer | None | None |
| `currentScale` | P_current / P_ratioSeek | ratio | Tracked | None | N/A |
| `lagTargetMs` | 100 - (aggressiveness × 75) | ms | Config | None | N/A |

**⚠️ Issue CRIT-008 (Major):** The lag calculation assumes a fixed sample period of 4000µs (250Hz). Systems running at 4K (250µs) or 8K (125µs) will have ~16-32x error in lag measurements. The `loopTimeUs` should come from the actual PID loop rate.

**⚠️ Issue CRIT-009:** The lag target formula `100 - (aggressiveness × 75)` means:
- aggressiveness=0 → 100ms lag target
- aggressiveness=1 → 25ms lag target

The 100ms default may be too sluggish for most quads. The PRD mentions `10ms` default, suggesting a discrepancy.

---

### 2.7 AUTOTUNE_STATE_F_TUNE

| Criterion Type | Name | Location | Signals | Transitions |
|---------------|------|----------|---------|-------------|
| **Entry** | PD scaled, lag still > target | `transitionToState()` | Master state | From PD_SCALE_UP |
| **Exit/Success** | Lag ≤ target | `fTuneDecision()` | `metrics.lagMs` | → PD_RETUNE or next axis |
| **Exit/Limit** | F at max (200) | `fTuneDecision()` | `fState.currentValue` | → PD_RETUNE or next axis |
| **Exit/Limit** | Max events (10) | `stateFTuneUpdate()` | `eventCount` | → PD_RETUNE or next axis |
| **Safety/Overshoot** | overshoot > target × 1.3 | `fTuneDecision()` | `overshootPct` | ROLLBACK + retune flag |

**Metric Specifications:**

| Metric | Definition | Units | Window | Filtering | Debounce |
|--------|------------|-------|--------|-----------|----------|
| `preFTuneOvershoot` | Overshoot before F tuning started | % | Snapshot | None | N/A |

**✅ Good:** F_TUNE correctly sets `pdRetuneNeeded = true` when overshoot increases, triggering PD re-validation.

---

### 2.8 AUTOTUNE_STATE_PD_RETUNE_AFTER_F

| Criterion Type | Name | Location | Signals | Transitions |
|---------------|------|----------|---------|-------------|
| **Entry** | F changed | `transitionToState()` | `pdRetuneNeeded` | From F_TUNE |
| **Exit/Success** | Overshoot acceptable | `statePdRetuneUpdate()` | `overshootPct ≤ target × 1.2` | → next axis |
| **Exit/Limit** | Max 3 events | `statePdRetuneUpdate()` | `eventCount` | → next axis |
| **Correction** | Rebound detected | `statePdRetuneUpdate()` | `hasRebound` | -5% P and D |
| **Threshold** | Overshoot delta > 3% | `statePdRetuneEnter()` | Pre/post comparison | Sets retune flag |

**✅ Good:** Quick validation with bounded iteration (max 3 events).

---

### 2.9 AUTOTUNE_STATE_COMPLETE

| Criterion Type | Name | Location | Signals | Transitions |
|---------------|------|----------|---------|-------------|
| **Entry** | All axes done | `advanceToNextAxisOrComplete()` | Axis loop | From last axis |
| **Exit** | Switch off | `autotuneUpdateActivation()` | `enabled = false` | → IDLE |

**✅ No criteria issues.**

---

## 3. Metric Specification Details

### 3.1 Overshoot Percentage

**Definition:**
```c
overshoot_pct = (peakGyro - finalSetpoint) / finalSetpoint × 100
```

**Implementation:** [autotune_metrics.c#L81-L109](../../../src/main/flight/autotune_v2/autotune_metrics.c#L81-L109)

| Property | Value | Issue |
|----------|-------|-------|
| **Units** | Percentage | ✅ Correct |
| **Sampling Window** | EVENT_BUFFER_SIZE (128) samples | ⚠️ May be too short for slow responses |
| **Filtering** | None (raw peak detection) | ⚠️ Noise-sensitive |
| **Debounce** | None | ⚠️ Single-sample peak can be noise spike |
| **Validity Gating** | `buffer->count < 10` returns 0 | ⚠️ 10 samples at 4ms = 40ms, may be too short |
| **Numerical Stability** | `finalSetpoint < 0.001f` returns 0 | ✅ Division by zero protected |

**Edge Cases:**
- **Noise spike as peak:** A single-sample noise spike will be detected as the peak, over-estimating overshoot.
- **Negative direction:** Uses `fabsf()` so works for both directions.
- **No overshoot:** If peak < final, returns 0 (not negative).

---

### 3.2 Rebound Detection

**Definition:**
```c
hasRebound = any peak after first peak exceeds firstPeak × REBOUND_THRESH (5%)
```

**Implementation:** [autotune_metrics.c#L115-L148](../../../src/main/flight/autotune_v2/autotune_metrics.c#L115-L148)

| Property | Value | Issue |
|----------|-------|-------|
| **Units** | Boolean | ✅ Correct |
| **Threshold** | 5% of first peak (AUTOTUNE_REBOUND_THRESHOLD) | ⚠️ Should be relative to setpoint, not first peak |
| **Peak Count** | Max 10 peaks detected | ✅ Reasonable limit |

**⚠️ Issue CRIT-010:** The rebound threshold is relative to the first peak magnitude, not the setpoint. If the first peak has 50% overshoot, the rebound threshold becomes 52.5% of setpoint (50% × 1.05). For a well-damped response with 5% overshoot, the threshold is only 5.25% of setpoint. This makes rebound detection inconsistent across overshoot levels.

---

### 3.3 Lag (Time to 50%)

**Definition:**
```c
lagMs = (index where gyro first crosses 50% of finalSetpoint) × samplePeriodUs / 1000
```

**Implementation:** [autotune_metrics.c#L217-L263](../../../src/main/flight/autotune_v2/autotune_metrics.c#L217-L263)

| Property | Value | Issue |
|----------|-------|-------|
| **Units** | Milliseconds | ✅ Correct |
| **Sample Period** | Hardcoded 4000µs (250Hz) | ❌ **Major: Should use actual loop rate** |
| **Validity Gating** | Minimum 10 samples | ✅ OK |
| **Numerical Stability** | Returns 0 if never reaches 50% | ✅ OK |

**❌ Issue CRIT-008 (Duplicate):** Critical bug - the 4000µs assumption breaks on all non-250Hz systems.

---

### 3.4 Settling Time

**Definition:**
```c
settlingTimeMs = time from last point outside 5% band to end of buffer
```

**Implementation:** [autotune_metrics.c#L180-L215](../../../src/main/flight/autotune_v2/autotune_metrics.c#L180-L215)

| Property | Value | Issue |
|----------|-------|-------|
| **Units** | Milliseconds | ✅ Correct |
| **Settling Band** | 5% of final setpoint (AUTOTUNE_SETTLING_BAND) | ✅ Matches PRD |
| **Sample Period** | Hardcoded 4000µs | ❌ **Same as lag issue** |

---

### 3.5 Trust Score

**Definition:**
```c
trustScore ∈ [0.1, 1.0], affects step size
trustScore += 0.1 on good event
trustScore -= 0.2 on bad event
```

**Implementation:** [autotune_rollback.c#L65-L96](../../../src/main/flight/autotune_v2/autotune_rollback.c#L65-L96)

| Property | Value | Issue |
|----------|-------|-------|
| **Initial Value** | 0.5 | ✅ Conservative |
| **Minimum** | 0.1 | ✅ Never zero |
| **Asymmetry** | -0.2 bad vs +0.1 good | ✅ Correct bias toward caution |

**✅ No issues with trust system implementation.**

---

### 3.6 Event Quality Gates

| Gate | Threshold | Implementation | Issue |
|------|-----------|----------------|-------|
| **Min Deflection** | 15° (AUTOTUNE_STICK_DEFLECTION_MIN_DEG) | `autotuneEventCheckDeflection()` | ⚠️ Units may be wrong - deflection stored as normalized float (0-1) not degrees |
| **Max Cross-Axis** | 10° (AUTOTUNE_CROSS_AXIS_MAX_DEG) | `autotuneEventCheckCrossAxis()` | ⚠️ Same units issue |
| **Throttle Band** | 25-75% | `autotuneEventCheckThrottle()` | ✅ Correct |
| **Min Duration** | 50ms | `autotuneEventCheckQuality()` | ✅ Correct |
| **Max Duration** | 500ms | `autotuneEventCheckQuality()` | ✅ Correct |

**⚠️ Issue CRIT-011:** The event detector stores `stickDeflection` as a normalized float (-1 to 1), not degrees. The quality gate compares this to `AUTOTUNE_STICK_DEFLECTION_MIN_DEG = 15.0`, which will always pass since |stickDeflection| ≤ 1.0 < 15.0.

**Code evidence:**
```c
// In autotune_event.c line ~79:
currentEvent.stickDeflection = detector.peakDeflection;  // This is 0-1 normalized

// In autotune_event.c line ~181:
bool autotuneEventCheckDeflection(float deflection)
{
    return deflection >= AUTOTUNE_STICK_DEFLECTION_MIN_DEG;  // 15.0 degrees!
}
```

This gate **will never pass** since normalized deflection (0-1) is always < 15.

---

## 4. Robustness & Correctness Checks

### 4.A Impossible or Unlikely-to-Meet Criteria

| ID | Criterion | Issue | Severity |
|----|-----------|-------|----------|
| **IMP-001** | Event deflection ≥ 15° | Gate compares normalized (0-1) to degrees (15) | ❌ Blocker |
| **IMP-002** | Cross-axis ≤ 10° | Same units mismatch | ❌ Blocker |

### 4.B Premature / Fragile Criteria

| ID | Criterion | Issue | Severity |
|----|-----------|-------|----------|
| **FRAG-001** | First peak detection | Uses single local max, noise can trigger early | Major |
| **FRAG-002** | Lag crossing detection | No interpolation, single-sample resolution | Minor |
| **FRAG-003** | Hover throttle stability | 5% band with no hysteresis, can oscillate | Minor |

### 4.C Units / Scaling / Frame Mistakes

| ID | Location | Issue | Severity |
|----|----------|-------|----------|
| **UNIT-001** | Event quality gates | Degrees vs normalized float confusion | ❌ Blocker |
| **UNIT-002** | Lag calculation | Hardcoded 4000µs sample period | Major |
| **UNIT-003** | Throttle normalization | `/1000.0f` assumes specific RC range | Minor |

### 4.D Coupling & Confounding

| ID | Issue | Evidence | Severity |
|----|-------|----------|----------|
| **COUP-001** | Cross-axis contamination | Quality gate exists but units broken | Major |
| **COUP-002** | Noise vs maneuver confusion | Noise measured during potential maneuvers | Minor |
| **COUP-003** | Battery voltage compensation | None implemented | Minor |

### 4.E Criteria Drift vs PRD

| PRD Specification | Code Implementation | Assessment |
|-------------------|---------------------|------------|
| "Lag target 10ms default" | `lagTargetMs = 100 - agg×75` (100ms default) | ❌ Likely bug |
| "Apply filter changes" | TODO stub | ❌ Missing feature |
| "Settling band 5%" | Implemented as 5% | ✅ Matches |
| "REBOUND_THRESH 0.05" | Uses 5% of first peak | ⚠️ Semantics differ |

---

## 5. Per-State Criteria Contracts

### 5.1 HOVER_LOCK Contract

| Property | Value |
|----------|-------|
| **Mission** | Establish stable hover baseline |
| **"Done" Criteria** | Sticks centered + throttle stable for 1 second |
| **"Failed" Criteria** | 10 second timeout |
| **"Unsafe" Criteria** | Disarm, failsafe |
| **Expected Trend** | Throttle variance → 0 |
| **Logged at Decision** | State, reasonCode |

### 5.2 THROTTLE_SWEEP Contract

| Property | Value |
|----------|-------|
| **Mission** | Characterize noise across throttle range |
| **"Done" Criteria** | 2 sweeps + 2s stable hover |
| **"Failed" Criteria** | Timeout (uses total timeout) |
| **"Unsafe" Criteria** | N/A |
| **Expected Trend** | noiseLevel[] populated, sweepCount → 2+ |
| **Logged at Decision** | sweepProgress, throttleRange |

### 5.3 PD_RATIO_SEEK Contract

| Property | Value |
|----------|-------|
| **Mission** | Find P value for critical damping (D fixed) |
| **"Done" Criteria** | overshoot ∈ [target - 2%, target + 2%] OR bracket < 3 |
| **"Failed" Criteria** | 10 events without convergence |
| **"Unsafe" Criteria** | Rebound detected → rollback |
| **Expected Trend** | overshoot → target, bracket width → 0 |
| **Logged at Decision** | overshoot×10, P gain, reasonCode, decision |

### 5.4 PD_SCALE_UP Contract

| Property | Value |
|----------|-------|
| **Mission** | Increase response speed while maintaining damping |
| **"Done" Criteria** | lag ≤ lagTarget OR P at max (200) |
| **"Failed" Criteria** | 10 events without reaching lag target |
| **"Unsafe" Criteria** | Rebound OR overshoot > 1.5× target → rollback |
| **Expected Trend** | lag → lagTarget, scale → increasing |
| **Logged at Decision** | overshoot×10, P, D, reasonCode |

### 5.5 F_TUNE Contract

| Property | Value |
|----------|-------|
| **Mission** | Reduce lag via feedforward |
| **"Done" Criteria** | lag ≤ lagTarget OR F at max (200) |
| **"Failed" Criteria** | 10 events |
| **"Unsafe" Criteria** | overshoot > 1.3× target → rollback + retune |
| **Expected Trend** | lag → lagTarget, F → increasing |
| **Logged at Decision** | overshoot×10, reasonCode |

---

## 6. Evidence & Logging Adequacy

### 6.1 Current Logging Status

| Data Point | Logged? | Channel | Notes |
|------------|---------|---------|-------|
| State | ✅ Yes | DEBUG_STATE (0) | Enum value |
| Axis | ✅ Yes | DEBUG_AXIS (1) | 0/1/2 |
| Reason Code | ✅ Yes | DEBUG_REASON (2) | Enum value |
| Decision | ✅ Yes | DEBUG_DECISION (3) | Enum value |
| Overshoot | ✅ Yes | DEBUG_OVERSHOOT (4) | ×10 scaling |
| P Gain | ✅ Yes | DEBUG_GAIN_P (5) | Raw value |
| D Gain | ✅ Yes | DEBUG_GAIN_D (6) | Raw value |
| F Gain | ⚠️ Partial | DEBUG_GAIN_F (7) | Only on some paths |

### 6.2 Missing Critical Log Data

| Missing Data | Why Needed | Proposed Fix |
|--------------|------------|--------------|
| Lag measurement | Key metric for PD_SCALE_UP/F_TUNE decisions | Add DEBUG_LAG channel |
| Rebound flag | Critical safety trigger | Encode in reason code |
| Trust score | Explains step size decisions | Add DEBUG_TRUST channel |
| Bracket state | Explains Newton refinement | Log P_low, P_high |
| Event validity | Why events rejected | Already in reason code ✅ |
| Filter recommendations | Verify filter phase | Log recommended LPF |

### 6.3 Proposed Minimal Additions

```c
// In autotune_debug.h, add:
#define AUTOTUNE_DEBUG_LAG       4  // Repurpose or add
#define AUTOTUNE_DEBUG_TRUST     5  // Repurpose or add

// In decision functions, add:
AUTOTUNE_DEBUG_SET(AUTOTUNE_DEBUG_LAG, (int16_t)(metrics.lagMs * 10));
```

---

## 7. Issue List (Handoff-Ready)

### CRIT-001: Throttle Normalization May Vary

| Property | Value |
|----------|-------|
| **ID** | CRIT-001 |
| **Severity** | Minor |
| **Criterion Affected** | HOVER_LOCK throttle stability |
| **Evidence** | `rcCommand[THROTTLE] / 1000.0f` assumes 0-1000 range |
| **Failure Mode** | Incorrect throttle percentage on non-standard configs |
| **Proposed Fix** | Use `(rcCommand[THROTTLE] - PWM_RANGE_MIN) / (float)(PWM_RANGE_MAX - PWM_RANGE_MIN)` |
| **Validation** | Test with various RC configurations |

---

### CRIT-004: Filter Application Not Implemented

| Property | Value |
|----------|-------|
| **ID** | CRIT-004 |
| **Severity** | ❌ Blocker |
| **Criterion Affected** | NOISE_CONFIRM filter tightening |
| **Evidence** | `autotuneFilterApplyRecommendations()` is `// TODO: Phase 1` |
| **Failure Mode** | Filter phase runs but filters never change, noise never reduces |
| **Proposed Fix** | Implement filter application to gyro/D-term LPF |
| **Validation** | Blackbox log shows filter cutoff changes during throttle sweep |

---

### CRIT-008: Hardcoded Sample Period

| Property | Value |
|----------|-------|
| **ID** | CRIT-008 |
| **Severity** | Major |
| **Criterion Affected** | Lag, settling time calculations |
| **Evidence** | `const float samplePeriodUs = 4000.0f;` in `autotuneMetricsAnalyze()` |
| **Failure Mode** | 8K system: lag reported as 32× actual, will never meet target |
| **Proposed Fix** | Pass actual PID loop period from `gyro.targetLooptime` |
| **Validation** | Test on 250Hz, 1K, 4K, 8K systems, verify lag values match expected |

---

### CRIT-009: Lag Target Formula Mismatch

| Property | Value |
|----------|-------|
| **ID** | CRIT-009 |
| **Severity** | Minor |
| **Criterion Affected** | PD_SCALE_UP exit criterion |
| **Evidence** | `lagTargetMs = 100.0f - (runtime.aggressiveness * 75.0f)` gives 100ms default |
| **Failure Mode** | Default lag target too sluggish, quads won't scale up enough |
| **Proposed Fix** | Review with PRD author; likely should be `25 - agg×15` for 10-25ms range |
| **Validation** | Confirm reasonable lag values in flight test |

---

### CRIT-010: Rebound Threshold Relative to Wrong Reference

| Property | Value |
|----------|-------|
| **ID** | CRIT-010 |
| **Severity** | Major |
| **Criterion Affected** | Rebound detection (safety trigger) |
| **Evidence** | `peakMag > firstPeak * threshold` in `autotuneMetricsDetectRebound()` |
| **Failure Mode** | High-overshoot responses have high threshold, may miss oscillation |
| **Proposed Fix** | Use setpoint as reference: `peakMag > finalSetpoint * threshold` |
| **Validation** | Test rebound detection with various overshoot levels |

---

### CRIT-011: Event Quality Gate Units Mismatch

| Property | Value |
|----------|-------|
| **ID** | CRIT-011 |
| **Severity** | ❌ Blocker |
| **Criterion Affected** | Event deflection quality gate |
| **Evidence** | `stickDeflection` is 0-1 normalized, compared to 15.0 degrees |
| **Failure Mode** | **All events fail quality gate** - normalized value never ≥ 15 |
| **Proposed Fix** | Either: (a) convert stick position to degrees using rates, or (b) change threshold to normalized value (e.g., 0.15 for 15% deflection) |
| **Validation** | Verify events pass quality gates during normal stick movements |

---

### CRIT-005: Static Variable State Leak

| Property | Value |
|----------|-------|
| **ID** | CRIT-005 |
| **Severity** | Minor |
| **Criterion Affected** | NOISE_CONFIRM iteration count |
| **Evidence** | `static uint8_t noiseConfirmIterations = 0;` outside function scope |
| **Failure Mode** | Counter may have stale value on re-entry after abort |
| **Proposed Fix** | Move to runtime state or reset in stateNoiseConfirmEnter() |
| **Validation** | Test abort-and-restart scenario |

---

## 8. Recommended Priority Order

### P0 - Must Fix Before Any Testing

1. **CRIT-011** - Event quality gate units (Blocker: no events will pass)
2. **CRIT-004** - Filter application stub (Blocker: filter phase non-functional)

### P1 - Must Fix Before Field Testing

3. **CRIT-008** - Sample period assumption (Major: breaks on non-250Hz)
4. **CRIT-010** - Rebound threshold reference (Major: safety gate unreliable)

### P2 - Should Fix Before Release

5. **CRIT-009** - Lag target formula (tune quality)
6. **CRIT-005** - Static variable leak (edge case stability)
7. **CRIT-001** - Throttle normalization (edge case correctness)

---

## 9. Appendix: Code References

### Key Decision Functions

| Function | File | Line | Purpose |
|----------|------|------|---------|
| `pdRatioSeekDecision()` | autotune_core.c | ~390 | P adjustment decision |
| `pdScaleUpDecision()` | autotune_core.c | ~680 | Scale adjustment decision |
| `fTuneDecision()` | autotune_core.c | ~1045 | F adjustment decision |
| `autotuneEventCheckQuality()` | autotune_event.c | ~192 | Event validation |
| `autotuneMetricsAnalyze()` | autotune_metrics.c | ~49 | Metrics extraction |
| `autotuneFilterSweepComplete()` | autotune_filter.c | ~163 | Sweep completion check |

### Threshold Constants

| Constant | Value | File | Usage |
|----------|-------|------|-------|
| `AUTOTUNE_STICK_DEFLECTION_MIN_DEG` | 15.0 | autotune_debug.h | Event quality |
| `AUTOTUNE_CROSS_AXIS_MAX_DEG` | 10.0 | autotune_debug.h | Event quality |
| `AUTOTUNE_OVERSHOOT_TARGET_DEFAULT` | 7.5% | autotune_debug.h | Decision target |
| `AUTOTUNE_REBOUND_THRESHOLD` | 0.05 | autotune_debug.h | Oscillation detection |
| `AUTOTUNE_NOISE_ACCEPTABLE` | 15.0 | autotune_debug.h | Noise gate |
| `AUTOTUNE_HOVER_LOCK_DURATION_US` | 1,000,000 | autotune_debug.h | 1s hover |
| `AUTOTUNE_HOVER_LOCK_TIMEOUT_US` | 10,000,000 | autotune_debug.h | 10s timeout |

---

*Document generated by Criteria Evaluation Lead Engineer*  
*Last updated: January 23, 2026*
