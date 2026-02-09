# Phase 2: P/D Ratio Tuning for Post-Maneuver Ringing Suppression

## Overview

Phase 2 extends the FF Autotune system with automatic P/D ratio optimization. Once the feedforward gain has converged on an axis (Phase 1 complete), Phase 2 activates to detect and suppress **ringing** — the oscillation of the gyro around the setpoint that occurs after the initial overshoot during the plateau region of a maneuver.

### The Problem

After Phase 1 converges, the F-term ensures good tracking during the *rising* portion of a maneuver. However, once the setpoint reaches its peak and levels off (pilot holding stick at deflection), the gyro can **ring** around the setpoint:

```
         ok
          ↓          ↓ ← ringing
setpoint ─────────────────────────────
         ╱  ╲  ╱╲ ╱╲
gyro ───╱    ╲╱  ╵  ╲─────────────────

        RISING │  PLATEAU (ringing region)
               │
         first overshoot is acceptable
```

- The **first overshoot** after the rise is acceptable — it's a natural consequence of the system's response
- The **subsequent oscillations** (ringing) indicate the P/D ratio is not optimal
- Too much P relative to D: underdamped response, ringing persists
- Too much D relative to P: overdamped, sluggish settling (less common in practice)

### Relationship to Phase 1

| Aspect | Phase 1 (F-Term) | Phase 2 (P/D Ratio) |
|--------|-------------------|---------------------|
| **What it tunes** | Feedforward gain | P-term and/or D-term |
| **Where it measures** | During RISING (setpoint increasing) | After RISING (plateau/settling region) |
| **What it optimizes** | Tracking error during ramp | Ringing/oscillation at steady-state |
| **Prerequisite** | None (starts immediately) | Phase 1 must be CONVERGED on the axis |
| **Error metric** | Mean signed tracking error | Oscillation amplitude & zero-crossing count |

---

## Analysis Window: Where Ringing Happens

The existing 4-state machine provides the framework:

```
IDLE ──→ RISING ──→ ADJUSTING ──→ WAITING ──→ IDLE
                 ↑
                 │ Ringing analysis window starts here
                 │ (transition from RISING to post-rise)
```

### Defining the Ringing Analysis Window

The ringing occurs in a specific temporal region after the fast setpoint ramp:

1. **Window Start**: When the RISING state ends (setpoint acceleration drops below threshold — the stick has reached its target deflection)
2. **Window Duration**: A configurable analysis period (default ~150ms) after the rise ends
3. **Window End**: After the analysis duration, or if the setpoint drops significantly (pilot releasing stick)

During this window:
- The **setpoint is approximately constant** (high magnitude, low acceleration)
- The **gyro should settle** to match the setpoint
- Any oscillation of the gyro around the setpoint is the ringing we want to suppress

### Why This Window Is Consistent

This region is highly repeatable across maneuvers because:
- The pilot's stick has reached its target position and is being held
- The feedforward contribution drops to near-zero (setpoint delta ≈ 0)
- The response is dominated by P, I, and D terms
- The system is in a classic step-response settling regime

---

## Ringing Detection

### Primary Metric: Oscillation Energy

Rather than a single number, we compute a **ringing score** that captures both the amplitude and frequency of oscillation in the analysis window.

#### 1. Zero-Crossing Count (Schmitt Trigger)

Count the number of times the tracking error `(gyroRate - setpoint)` crosses zero during the analysis window, **excluding the first crossing** (the initial overshoot settling).

Zero-crossing detection uses a **Schmitt trigger** with `ring_deadband` as the hysteresis threshold. An `int8_t ringLastSide` field remembers which side of the deadband the signal was last on:

- When `error > +deadband`: latch `ringLastSide = +1`
- When `error < -deadband`: latch `ringLastSide = -1`  
- When `-deadband <= error <= +deadband`: `ringLastSide` retains its previous value (no crossing counted)

A crossing is registered only when `ringLastSide` changes from its previous value (i.e., signal went from above `+deadband` to below `-deadband` or vice versa).

```
error signal in plateau window:
     +  ╲    ╱╲    ╱╲
  +db ----╲--╱--╲--╱--╲------  (latch side=+1)
  0 ──────╳────╳────╳────╳─  (crossings after 1st: 4)
  -db ------╲╱----╲╱----╲╱--  (latch side=-1)
     -

     → 4 zero-crossings = significant ringing
```

#### 2. Peak-to-Peak Amplitude

Measure the maximum excursion of the error signal after the first overshoot:

```
ringing_amplitude = max(error_in_window) - min(error_in_window)
```

This is measured **after skipping the first peak** (the acceptable initial overshoot).

#### 3. Combined Ringing Score

```
ringing_score = zero_crossing_count * avg_peak_amplitude
```

- This weights both how *often* and how *severely* the system oscillates
- A single large overshoot that settles scores low (1 crossing, even if amplitude is high)
- Many small oscillations score high (many crossings, even if amplitude is moderate)

### Ringing Assessment

```
if ringing_score > ringing_threshold_high:
    assessment = RINGING         → needs P/D adjustment
elif ringing_score > ringing_threshold_low:
    assessment = MILD_RINGING    → borderline, may need adjustment
else:
    assessment = WELL_DAMPED     → no adjustment needed
```

---

## P/D Adjustment Strategy

### Guiding Principle

To suppress ringing, we need to increase the damping ratio of the closed-loop system. Two levers:

1. **Decrease P** — reduces the proportional gain driving the oscillation
2. **Increase D** — adds damping to resist rate changes

**Default preference: decrease P first**, because:
- Increasing D amplifies high-frequency noise (motor heat, possible resonance)
- Decreasing P is always safe in terms of noise
- In practice, most ringing on pitch is caused by P being slightly too aggressive for the given D

### Adjustment Algorithm

Phase 2 uses a similar bracketing approach as Phase 1, but applied to P-term:

```
Phase 2a: P-ADJUSTMENT (preferred)
  1. Measure ringing_score at current P
  2. Decrease P by p_step
  3. Measure ringing_score at new P
  4. If ringing improved: continue decreasing P (or bracket)
  5. If ringing got worse or response became too sluggish: stop, try Phase 2b
  6. Converge via bracketing when improvement plateaus

Phase 2b: D-ADJUSTMENT (fallback, if P reduction alone insufficient)
  - Only entered if P was reduced to minimum useful level but ringing persists
  - Increase D by d_step
  - Re-measure ringing
  - Same bracketing convergence
  - Capped by d_max to avoid excessive noise amplification
```

### What "Too Sluggish" Means

When we decrease P, we risk making the system too slow to respond. We detect this by monitoring the **first overshoot amplitude** during the RISING phase:

- If the first overshoot drops below a minimum threshold: P is too low, the system is overdamped
- This sets the lower bound for P reduction

### Adjustment Boundaries

| Parameter | Description | Constraint |
|-----------|-------------|------------|
| P minimum | Lowest acceptable P | Must maintain adequate first-overshoot (system still responsive) |
| P maximum | Highest acceptable P | Current value (we only decrease P) |
| D minimum | Lowest acceptable D | Current value (we only increase D) |
| D maximum | Highest acceptable D | `d_max` setting, or a noise-safety ceiling |
| Step sizes | How much to adjust per maneuver | Small: 1-2 units for P, 1 unit for D |

---

## State Machine Extension

### New Autotune Phase Enum

```c
typedef enum {
    FF_AUTOTUNE_PHASE1_FF = 0,    // Tuning feedforward gain
    FF_AUTOTUNE_PHASE2_PD,        // Phase 2a: P/D ratio for ringing suppression
    FF_AUTOTUNE_PHASE2B_SCALE,    // Phase 2b: Uniform P+D scale-down for noise
    FF_AUTOTUNE_PHASE3_RECHECK,   // F-term spot check after P/D changes
    FF_AUTOTUNE_COMPLETE           // All phases converged
} ffAutotunePhase_e;
```

### Per-Axis Phase 2 State

```c
typedef struct {
    // Phase tracking
    ffAutotunePhase_e phase;

    // Ringing measurement
    float ringErrorAccumulator;     // Error values during analysis window
    float ringPeakPos;              // Max positive error in window (after 1st overshoot)
    float ringPeakNeg;              // Max negative error in window
    uint16_t ringZeroCrossings;     // Zero-crossing count in window
    uint16_t ringSampleCount;       // Samples in current analysis window
    int8_t ringLastSide;            // Schmitt trigger: +1 = above +deadband, -1 = below -deadband, 0 = unknown
    bool ringFirstPeakPassed;       // Have we passed the first overshoot?
    bool ringWindowActive;          // Currently in analysis window
    timeUs_t ringWindowStartTime;   // When analysis window opened

    // Noise measurement (Phase 2b, accumulated during RISING)
    float noiseAccumulator;         // Sum of |D-term| during RISING
    uint16_t noiseSampleCount;      // Sample count during RISING
    float noiseBaseline;            // Baseline noise score (first measurement)
    int16_t scaleAdjustment;        // Cumulative uniform P+D scale-down

    // P/D adjustment state
    int16_t pAdjustment;            // Cumulative P adjustment (negative = decreased)
    int16_t dAdjustment;            // Cumulative D adjustment (positive = increased)
    bool adjustingD;                // Currently in D adjustment fallback

    // Ringing history for bracketing
    ffRingHistoryEntry_t ringHistory[FF_AUTOTUNE_HISTORY_SIZE];
    uint8_t ringHistoryCount;

    // Ringing bracket state
    int16_t ringLowerP;             // P value with ringing (too high)
    int16_t ringUpperP;             // P value well-damped (might be too low)
    ffBracketState_e ringBracketState;
} ffPhase2State_t;
```

### Analysis Window Within the State Machine

The existing 4-state machine does not need to change. Phase 2 analysis hooks into the **transition from RISING → ADJUSTING** and the **ADJUSTING state** itself:

```
State: RISING
  └── Phase 1 measures tracking error (as before)
  └── Phase 2: when RISING ends, open the ringing analysis window

State: ADJUSTING (100ms delay)
  └── Phase 1 waits then processes F-term (as before)
  └── Phase 2: continue collecting ringing data during this delay
  └── Phase 2: after 100ms, process BOTH F-term (if Phase 1) AND ringing data

State: WAITING
  └── Phase 2 analysis window closes (if not already closed by duration limit)
```

The ringing analysis window duration may extend slightly beyond the ADJUSTING 100ms delay into the WAITING state. A configurable `ringing_window_ms` (default 150ms) controls this.

---

## Integration with PID Controller

### P-Term Override

When Phase 2 is active and has made adjustments:

```c
// In pid.c, where P term is calculated:
float pGain = pidRuntime.pidCoefficient[axis].Kp;

#ifdef USE_FF_AUTOTUNE
if (ffAutotuneIsPhase2Active() && axis <= FD_PITCH) {
    pGain += PTERM_SCALE * (ffAutotuneGetPAdjustment(axis) * 0.01f);
}
#endif

pidData[axis].P = pGain * errorRate;
```

### D-Term Override (Phase 2b only)

```c
// In pid.c, where D term gain is applied:
float dGain = pidRuntime.pidCoefficient[axis].Kd;

#ifdef USE_FF_AUTOTUNE
if (ffAutotuneIsPhase2Active() && axis <= FD_PITCH) {
    dGain += DTERM_SCALE * (ffAutotuneGetDAdjustment(axis) * 0.01f);
}
#endif
```

### Data Available in the Analysis Window

During the ringing analysis window, we have access to:
- `gyroRate` — current filtered gyro reading
- `currentPidSetpoint` — the target rate
- `pidData[axis].P` — the P-term contribution
- `pidData[axis].D` — the D-term contribution
- `tracking_error = gyroRate - setpoint` — instantaneous error

---

## Configuration Parameters (New)

### Phase 2a: P/D Balance (Ringing)

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `ff_autotune_pd_enabled` | bool | ON | OFF/ON | Enable Phase 2 P/D tuning (requires Phase 1 also enabled) |
| `ff_autotune_ring_window_ms` | uint8 | 150 | 50-250 | Duration of ringing analysis window after rise ends (ms) |
| `ff_autotune_ring_threshold` | uint8 | 20 | 5-100 | Ringing score above which adjustment is triggered |
| `ff_autotune_ring_deadband` | uint8 | 5 | 2-20 | Schmitt trigger deadband for zero-crossing detection (deg/s) |
| `ff_autotune_p_step` | uint8 | 2 | 1-5 | P-term adjustment step size per maneuver |
| `ff_autotune_d_step` | uint8 | 1 | 1-3 | D-term adjustment step size (fallback in Phase 2a) |
| `ff_autotune_p_adjust_max` | uint8 | 10 | 2-20 | Maximum cumulative P reduction |
| `ff_autotune_d_adjust_max` | uint8 | 5 | 1-10 | Maximum cumulative D increase |
| `ff_autotune_p_adj_roll` | int8 | 0 | -20..0 | Learned Roll P adjustment (persisted) |
| `ff_autotune_p_adj_pitch` | int8 | 0 | -20..0 | Learned Pitch P adjustment (persisted) |
| `ff_autotune_d_adj_roll` | int8 | 0 | 0..10 | Learned Roll D adjustment (persisted) |
| `ff_autotune_d_adj_pitch` | int8 | 0 | 0..10 | Learned Pitch D adjustment (persisted) |

### Phase 2b: P/D Scale-Down (Noise)

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `ff_autotune_noise_threshold` | uint8 | 10 | 5-30 | Noise improvement % required to continue reducing |
| `ff_autotune_scale_step` | uint8 | 1 | 1-3 | Uniform P+D scale-down step per iteration |
| `ff_autotune_scale_max` | uint8 | 8 | 2-15 | Maximum cumulative scale-down |
| `ff_autotune_scale_adj_roll` | int8 | 0 | -15..0 | Learned Roll scale-down (persisted) |
| `ff_autotune_scale_adj_pitch` | int8 | 0 | -15..0 | Learned Pitch scale-down (persisted) |

> **Note:** `noise_window_ms` was removed. Noise is measured during the RISING phase (`|D-term|` accumulation) rather than a separate post-maneuver window.

---

## Debug Output

All phases use the consolidated `DEBUG_FF_AUTOTUNE` mode. Channels 0-3 are universal; channels 4-7 are phase-multiplexed based on `debug[3]` (phase).

See `DEBUG_PHASE2.md` for the full channel layout and interpretation guide.

---

## Algorithm Walkthrough: A Typical Flight

```
1. Pilot activates FF Autotune mode

2. Phase 1: F-Term Tuning (per axis)
   ├── Maneuver 1-6: Bracket F-term gain
   ├── Maneuver 7-10: Binary search within bracket
   └── Converge at F = 27 (example)

4. Phase 2a activates on this axis (F-term converged)

5. Phase 2a: P Reduction
   ├── Maneuver 11: Measure ringing at P=33 → score=45 (RINGING)
   ├── Decrease P to 31
   ├── Maneuver 12: Measure ringing at P=31 → score=28 (MILD)
   ├── Decrease P to 29
   ├── Maneuver 13: Measure ringing at P=29 → score=12 (WELL_DAMPED)
   ├── [bracket established: ringing between P=31 and well-damped at P=29]
   ├── Maneuver 14: Test P=30 → score=18 (MILD, below threshold)
   └── Converge at P=30 (adjustment = -3)

6. Phase 2b: P/D Scale-Down (Noise)
   ├── Maneuver 15: Measure baseline |D-term| during RISING = 80
   ├── Apply scaleAdjustment = -1 (reduce both P and D by 1)
   ├── Maneuver 16: Measure |D-term| = 65 (improved)
   ├── Apply scaleAdjustment = -2
   ├── Maneuver 17: Measure |D-term| = 58 (improvement < threshold)
   └── Converge at scaleAdjustment = -2

7. If Phase 2a was insufficient (ringing still above threshold at minimum P):
   D-term increase fallback within Phase 2a
   ├── Increase D by d_step
   ├── Re-measure ringing
   └── Converge similarly

8. F-Term Spot Check (after Phase 2b converged)
   ├── Re-run Phase 1 tracking error measurement for 2-3 maneuvers
   ├── Compare avg tracking error against the Phase 1 converged value
   ├── If error shifted beyond deadband:
   │     └── Re-open Phase 1 bracket, re-converge F (typically 1-3 maneuvers)
   └── If still within deadband: F validated, no change needed

7. Pilot deactivates mode → save F gain + P/D adjustments to EEPROM
```

---

## Phase 3: F-Term Spot Check

After Phase 2 converges on an axis, the P/D ratio has changed. Since F-term optimal gain depends partly on the closed-loop dynamics (which P and D affect), we do a quick revalidation.

### How It Works

1. **Enter Phase 3** when Phase 2 bracket state reaches CONVERGED
2. **Collect 2-3 RISING maneuvers** using the same tracking error metric as Phase 1 (mean signed error during setpoint ramp)
3. **Compare** the average tracking error against the Phase 1 converged value and deadband:
   - If within deadband: F is still good, transition to `FF_AUTOTUNE_COMPLETE`
   - If outside deadband: re-open the Phase 1 bracket around the current F gain and re-converge
4. **Re-convergence is fast** because we start near the optimal value — typically 1-3 maneuvers to re-establish the bracket

### Why This Is Lightweight

- Uses the exact same measurement infrastructure as Phase 1 (no new code for error accumulation)
- Only needs 2-3 validation maneuvers, not a full search
- If F doesn't need adjustment (the common case), it adds ~10 seconds of flight time
- If F does need a tweak, the bracketing starts narrow so convergence is fast

---

## EEPROM Save Policy Extension

The existing save-on-deactivation policy extends naturally:

- When mode switches OFF, save **all** modified parameters:
  - F-term gains (Phase 1) — `ff_autotune_gain_roll`, `ff_autotune_gain_pitch`
  - P adjustments (Phase 2a) — `ff_autotune_p_adj_roll`, `ff_autotune_p_adj_pitch`
  - D adjustments (Phase 2a) — `ff_autotune_d_adj_roll`, `ff_autotune_d_adj_pitch`
  - Scale adjustments (Phase 2b) — `ff_autotune_scale_adj_roll`, `ff_autotune_scale_adj_pitch`
- Adjustments are stored as **deltas from the base PID**, not absolute values
- This means the pilot can still independently tune P and D in the configurator, and the autotune adjustments are applied on top
- The effective PID modification in `pid.c` is:
  ```
  P_effective = base_P + pAdjustment + scaleAdjustment
  D_effective = base_D + dAdjustment + scaleAdjustment
  ```

---

## Safety Considerations

1. **P cannot go below a safety floor**: Even with maximum `p_adjust_max` reduction, the effective P must remain above a minimum functional level (e.g., base_P * 0.5)
2. **D cannot exceed d_max**: The D adjustment is capped so that effective D never exceeds the configured `d_max` value
3. **Phase 2 requires Phase 1 convergence**: Prevents P/D tuning from fighting an unconverged F-term
4. **First overshoot monitoring**: If P reduction causes the initial overshoot to disappear entirely (overdamped response), P reduction stops — the system needs *some* overshoot to confirm it's responsive
5. **Noise guard for D increase**: If motor noise increases beyond a threshold after D increase, the D adjustment is rolled back

---

## Files to Modify

### New/Modified Files

| File | Changes |
|------|---------|
| `src/main/flight/ff_autotune.h` | Add Phase 2 enums (incl. Phase2b_Scale, noise assessment), state structs, new API functions |
| `src/main/flight/ff_autotune.c` | Add ringing analysis (Schmitt trigger), P/D adjustment, Phase 2b noise scale-down, Phase 3 recheck |
| `src/main/pg/ff_autotune.h` | Add Phase 2a + 2b config fields |
| `src/main/pg/ff_autotune.c` | Register new parameters with defaults, PG version 3 |
| `src/main/cli/settings.c` | Add Phase 2a + 2b CLI parameters |
| `src/main/flight/pid.c` | Add P-term and D-term override hooks, scale adjustment (additive on both P and D) |

---

## Success Criteria

1. **Ringing reduction**: After Phase 2, zero-crossing count in analysis window should drop by at least 50%
2. **Response preservation**: First overshoot amplitude should not decrease by more than 30% from baseline
3. **Convergence speed**: Phase 2 should converge within 5-8 maneuvers per axis
4. **Noise neutrality**: Motor noise and D-term noise floor should not increase measurably when only P is adjusted
5. **Repeatability**: P/D adjustments should be consistent (within ±1 unit) across flights on the same craft

---

## Empirical Data: v3 Flight 1 Blackbox Analysis

Analysis of `mbff_v3_test_flight_1.csv` (62,823 samples, 62.4s flight) with pitch PID config `P=33, I=60, D=23, F=33`:

### Maneuver Summary (15 pitch maneuvers detected)

Filtering to large maneuvers (|setpoint| > 500 deg/s) which exhibit the real ringing problem:

| Man# | MeanSP | Duration | ZeroCrossings | RMS Err | Peak Err | D/P ratio | D-errDot corr |
|------|--------|----------|---------------|---------|----------|-----------|---------------|
| 0 | -662 | 334ms | 8 | 56.8 | 195 | 0.83 | -0.887 |
| 2 | -514 | 438ms | 17 | 49.8 | 154 | 0.92 | -0.826 |
| 4 | +644 | 322ms | 19 | 38.2 | 139 | 0.94 | -0.845 |
| 5 | -611 | 343ms | 9 | 41.2 | 122 | 0.78 | -0.851 |
| 6 | +654 | 335ms | 4 | 38.2 | 121 | 0.80 | -0.814 |
| 7 | +655 | 325ms | 6 | 50.4 | 157 | 0.81 | -0.869 |
| 8 | +664 | 321ms | 7 | 54.6 | 201 | 1.11 | -0.929 |
| 9 | +655 | 320ms | 14 | 43.9 | 117 | 0.72 | -0.832 |
| 10 | -643 | 309ms | 4 | 46.8 | 129 | 0.71 | -0.874 |
| 12 | +650 | 337ms | 6 | 46.3 | 143 | 0.65 | -0.781 |
| 13 | -579 | 313ms | 7 | 36.2 | 125 | 0.99 | -0.839 |

### Key Observations

1. **D-term is consistently fighting the oscillation**: All large maneuvers show strong negative D-errDot correlation (-0.78 to -0.93), confirming D opposes the error rate. The damping is present but not sufficient to fully suppress ringing.

2. **Ringing severity varies even among similar maneuvers**: Zero-crossings range from 4 (well-damped) to 19 (significant ringing) for maneuvers at similar setpoints. This suggests ringing is sensitive to exact conditions (throttle position, turbulence, etc.) — reinforcing the need for statistical averaging across multiple maneuvers before adjusting.

3. **D/P ratio hints at the problem**: Man#8 has the highest D/P ratio (1.11) and strongest damping correlation (-0.929) but still shows 7 ZC. The typical D/P ratio is 0.72–0.94, which may be too low for critical damping.

4. **Small maneuvers are noise, not ringing**: Maneuvers #3 and #11 (|setpoint| ~108-124 deg/s) show high ZC counts (23, 28) but trivial error amplitude (peak 6-7 deg/s). Phase 2 should only analyze maneuvers within the same setpoint window as Phase 1 (default 100-600 deg/s minimum, but ringing analysis benefits from requiring higher setpoint, e.g. >300 deg/s).

5. **Peak error is large**: Even well-damped maneuvers show 120+ deg/s peak error (the first overshoot). Phase 2 must skip this initial peak and focus on the subsequent oscillations. The "skip first peak, then count crossings" approach in the design is validated by this data.

### Implication for Default Thresholds

Based on this data:
- **ring_threshold**: A zero-crossing count of 8+ in the plateau window (after first peak) indicates actionable ringing. Suggest default threshold of 8 ZC for the "RINGING" assessment.
- **ring_deadband**: Error oscillations are typically 20-50 deg/s amplitude. A 5 deg/s deadband for zero-crossing detection filters noise without missing real oscillations.
- **Minimum maneuver setpoint for Phase 2**: Should be higher than Phase 1's lower bound. Suggest 300 deg/s to avoid false positives from small maneuver noise.
- **Analysis window duration**: Maneuver plateau durations are 300-440ms. A 150ms analysis window starting after the first peak captures the critical ringing region without extending into the stick-release phase.

---

## Design Decisions

1. **F-term spot check after P/D changes**: ACCEPTED. After Phase 2 converges P/D, a brief Phase 3 re-measures tracking error during RISING using the same Phase 1 metric. If the F-term is still within deadband, no change needed. If it drifted, Phase 1 re-opens its bracket and re-converges (typically 1-3 maneuvers since it starts from a near-optimal value). This adds minimal flight time but catches coupling between F and P/D.

2. **Independent P and D adjustment** (not ratio-based): We adjust P and D independently rather than a single ratio parameter. This gives more flexibility and maps directly to the CLI values pilots understand.

3. **No I-term or other adjustments for now**: Phase 2 focuses solely on P and D. I-term wind-up and other interactions are deferred to future work if needed after flight testing validates the core approach.

---

## Revision History

| Date | Version | Author | Changes |
|------|---------|--------|---------|
| 2026-02-07 | 0.1 | PMantix + Claude | Initial design document |
| 2026-02-08 | 0.2 | PMantix + Claude | Updated: Schmitt trigger zero-crossing detection (`ringLastSide` replacing `ringPrevError`), Phase 2b separated as P/D scale-down (noise measurement during RISING), consolidated `FF_AUTOTUNE` debug mode, `noise_window_ms` removed, PG version 3, scale adjustment EEPROM fields |
