# Phase 2 Debug Reference: P/D Ratio Tuning

## Debug Mode Setup

In Betaflight CLI:
```
set debug_mode = FF_AUTOTUNE_PD
save
```

To also log Phase 1 data simultaneously, use two flights with different debug modes:
- Flight 1: `debug_mode = FF_AUTOTUNE` (Phase 1 channels)
- Flight 2: `debug_mode = FF_AUTOTUNE_PD` (Phase 2 channels)

---

## Debug Channel Mapping: FF_AUTOTUNE_PD

| Channel | Name | Range | Description |
|---------|------|-------|-------------|
| `debug[0]` | Ringing Score | 0-65535 | Combined score: `zero_crossings * avg_peak_amplitude`. Updates after each ringing window closes. Higher = more ringing. |
| `debug[1]` | Zero Crossings | 0-100+ | Number of times error crossed zero in analysis window (after skipping first overshoot). |
| `debug[2]` | Ring Amplitude | 0-500+ | Peak-to-peak error amplitude in deg/s (after first overshoot). Measures how far error swings. |
| `debug[3]` | P Adjustment | -20..0 | Current cumulative P-term delta. Negative = P reduced. Steps by `p_step` per maneuver. |
| `debug[4]` | D Adjustment | 0..10 | Current cumulative D-term delta. Positive = D increased. Only non-zero in Phase 2b. |
| `debug[5]` | Window Active | 0 or 1 | 1 when ringing analysis window is open (measuring oscillation). |
| `debug[6]` | Autotune Phase | 0-3 | Current phase for the debug axis (see Phase State table below). |
| `debug[7]` | Ring Assessment | 0-2 | Assessment from last ringing measurement (see Assessment table below). |

---

## Debug Channel Mapping: FF_AUTOTUNE (Phase 1, for reference)

| Channel | Name | Range | Description |
|---------|------|-------|-------------|
| `debug[0]` | Gain | 0-200 | Current FF gain value |
| `debug[1]` | Tracking Error | +/-500 | `fabsf(gyroRate) - fabsf(setpoint)` in deg/s |
| `debug[2]` | Window State | 0-3 | State machine state (IDLE/RISING/ADJUSTING/WAITING) |
| `debug[3]` | Last Assessment | -1, 0, +1 | Result of last maneuver (LAG/OPTIMAL/LEAD) |
| `debug[4]` | Avg Error | +/-500 | Average tracking error x10 |
| `debug[5]` | Sample Count | 0-255 | Samples in current maneuver |
| `debug[6]` | History Count | 0-8 | Entries in history buffer |
| `debug[7]` | Bracket State | 0-2 | Gain search state (SEARCHING/BRACKETED/CONVERGED) |

---

## Phase State (debug[6])

| Value | Phase | Description | What's happening |
|-------|-------|-------------|-----------------|
| **0** | `PHASE1_FF` | F-term tuning | Adjusting feedforward gain via tracking error |
| **1** | `PHASE2_PD` | P/D ratio tuning | Measuring ringing, adjusting P (and optionally D) |
| **2** | `PHASE3_RECHECK` | F-term spot check | Verifying F-term still valid after P/D changes |
| **3** | `COMPLETE` | All phases converged | No more adjustments will be made |

### Expected Phase Flow
```
0 (Phase 1) --> 1 (Phase 2) --> 2 (Phase 3) --> 3 (Complete)
                                    |
                                    v
                              0 (Phase 1)   <-- only if F-term drifted
```

### Phase Transition Conditions

| From | To | Condition |
|------|----|-----------|
| Phase 1 | Phase 2 | F-term bracket CONVERGED and `pd_enabled = ON` |
| Phase 1 | Complete | F-term bracket CONVERGED and `pd_enabled = OFF` |
| Phase 2 | Phase 3 | P (or D) bracket CONVERGED |
| Phase 3 | Complete | F-term recheck within deadband (3 maneuvers) |
| Phase 3 | Phase 1 | F-term drifted beyond deadband (re-opens narrow bracket) |

---

## Ringing Assessment (debug[7])

| Value | Assessment | Meaning | Action Taken |
|-------|-----------|---------|--------------|
| **0** | `WELL_DAMPED` | Score below threshold/2 | No P/D adjustment (or sets upper P bracket bound) |
| **1** | `MILD` | Score between threshold/2 and threshold | Cautious: may continue decreasing P or binary search |
| **2** | `RINGING` | Score above threshold | Decrease P by `p_step` (sets lower P bracket bound) |

---

## Ringing Analysis Window Lifecycle

```
                    RISING ends
                        |
                        v
Window Opens -----> Accumulating -----> Window Closes
                   (collecting         (score computed,
                    zero-crossings      assessment made,
                    and peak-to-peak    P/D adjusted)
                    amplitude)
                        |
                  ring_window_ms
                  (default 150ms)

  OR: window closes early if stick returns to neutral (WAITING -> IDLE)
```

### What Happens Inside the Window

1. **Error signal**: `gyroRate - setpoint` (signed tracking error)
2. **First peak skip**: The first zero-crossing is marked as the end of the initial overshoot (acceptable). It is NOT counted.
3. **Zero-crossing counting**: Each subsequent crossing of the zero line (with `ring_deadband` hysteresis) increments the counter.
4. **Amplitude tracking**: After the first peak, the maximum positive and negative error excursions are tracked.
5. **Score**: `zero_crossings * ((peakPos - peakNeg) / 2)`

### Visual Example

```
Error signal in plateau window:

     +50 |      /\        /\
         |     /  \      /  \
  +db  --|----/----\----/----\------  (deadband = +5)
     0   |--x------x--x------x-----  (zero-crossings after 1st: 4)
  -db  --|--------/----\----/-------  (deadband = -5)
         |       /      \/
    -30  |      /
         |    /  <-- first peak (skipped)
         +-----------------------------> time
              ^                    ^
         window open          window close (150ms)
```

In this example:
- First zero-crossing: marks end of initial overshoot (not counted)
- Subsequent crossings: 4 counted
- Peak positive (after first peak): +50
- Peak negative (after first peak): -30
- Amplitude: 50 - (-30) = 80
- Score: 4 * (80/2) = 160

---

## How to Analyze Blackbox Logs

### 1. Open in Blackbox Explorer

Load your `.bbl` or `.bfl` file. Add these traces:
- `debug[0]` - Ringing Score (scale: auto)
- `debug[3]` - P Adjustment (scale: fixed -15 to 0)
- `debug[5]` - Window Active (scale: fixed 0 to 1)
- `debug[6]` - Phase (scale: fixed 0 to 3)

Also overlay:
- `gyro[1]` (pitch gyro) and `setpoint[1]` (pitch setpoint)

### 2. Verify Window Timing

1. Find a maneuver: large setpoint ramp followed by plateau
2. Check that `debug[5]` goes HIGH right when the setpoint stops ramping
3. Check that `debug[5]` stays HIGH for approximately `ring_window_ms`
4. During the HIGH period, look at gyro[1] vs setpoint[1]:
   - Oscillation visible? Score should be non-zero
   - Smooth settling? Score should be low

### 3. Check Ringing Score Progression

Plot `debug[0]` across multiple maneuvers:
- Should decrease as P adjustment becomes more negative
- If it doesn't decrease: P reduction not helping, may need D increase

### 4. Check P Adjustment Progression

Plot `debug[3]` across the flight:
- Should step down by `p_step` after each "RINGING" assessment
- Should stabilize when well-damped region found
- If it reaches `p_adjust_max` (e.g., -10) and stops: check `debug[4]` for D increase

### 5. Verify Phase Transitions

Plot `debug[6]` across the flight:
- 0 (Phase 1) should last 6-10 maneuvers
- 1 (Phase 2) should last 5-8 maneuvers
- 2 (Phase 3) should last exactly 3 maneuvers
- 3 (Complete) should hold steady until deactivation

---

## CLI Parameters Reference

### Phase 2 Tuning Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ff_autotune_pd_enabled` | ON | OFF/ON | Master enable for Phase 2 |
| `ff_autotune_ring_window_ms` | 150 | 50-250 | Analysis window length after rise ends (ms) |
| `ff_autotune_ring_threshold` | 20 | 5-100 | Score above this = RINGING assessment |
| `ff_autotune_ring_deadband` | 5 | 2-20 | Error deadband for zero-crossing detection (deg/s) |
| `ff_autotune_p_step` | 2 | 1-5 | P adjustment step per maneuver |
| `ff_autotune_d_step` | 1 | 1-3 | D adjustment step per maneuver (Phase 2b) |
| `ff_autotune_p_adjust_max` | 10 | 2-20 | Maximum cumulative P reduction |
| `ff_autotune_d_adjust_max` | 5 | 1-10 | Maximum cumulative D increase |

### Learned Values (saved to EEPROM)

| Parameter | Range | Description |
|-----------|-------|-------------|
| `ff_autotune_p_adj_roll` | -20..0 | Cumulative P reduction on roll |
| `ff_autotune_p_adj_pitch` | -20..0 | Cumulative P reduction on pitch |
| `ff_autotune_d_adj_roll` | 0..10 | Cumulative D increase on roll |
| `ff_autotune_d_adj_pitch` | 0..10 | Cumulative D increase on pitch |

### How Adjustments Apply to PID

The adjustments are deltas added to the base PID coefficients:

```
Effective P = base_Kp + PTERM_SCALE * (p_adj * 0.01)
Effective D = base_Kd + DTERM_SCALE * (d_adj * 0.01)
```

- `PTERM_SCALE = 0.032029`
- `DTERM_SCALE = 0.000529`
- Adjustments are **additive** - the pilot's base P/D in the configurator is preserved
- A `p_adj` of -5 reduces the effective P by `0.032029 * 0.05 = 0.0016` per unit of errorRate

---

## Quick Troubleshooting

| Symptom | What to Check | Likely Fix |
|---------|---------------|------------|
| `debug[6]` stuck at 0 | Phase 1 not converging | Switch to `FF_AUTOTUNE` debug mode, check bracket state |
| `debug[5]` never goes to 1 | Window not opening | Ensure Phase 2 is active (debug[6]=1); verify maneuvers are large enough |
| `debug[0]` always 0 | No ringing detected or window too short | Increase `ring_window_ms`; decrease `ring_deadband` |
| `debug[0]` always very high | Threshold too low for this craft | Increase `ring_threshold` (try 40-60) |
| `debug[3]` drops to max immediately | Step size too large or threshold too sensitive | Decrease `p_step` to 1; increase `ring_threshold` |
| `debug[3]` never changes | Always assessed as WELL_DAMPED | P/D ratio is already good, or threshold too high |
| `debug[4]` increasing | Phase 2b active (D increase) | Check if P hit limit; verify `p_adjust_max` setting |
| `debug[6]` oscillates 0 <-> 2 | F-term keeps needing re-adjustment | Increase `error_deadband` to reduce sensitivity |
| `debug[7]` always 2 (ringing) | Ringing not fixable by P/D alone | Mechanical issue, filter tuning needed, or adjust PID base values |
| Sluggish quad after tuning | P reduced too much | Decrease `p_adjust_max`; or manually bump `p_adj` toward 0 |
| Motor noise after tuning | D increased too much | Decrease `d_adjust_max`; set `d_adj_*` back to 0 |

---

## Example Blackbox Analysis Script (Python)

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load blackbox CSV export (debug_mode = FF_AUTOTUNE_PD)
df = pd.read_csv('flight_log.csv')

fig, axes = plt.subplots(6, 1, figsize=(14, 16), sharex=True)

# Panel 1: Gyro vs Setpoint (pitch axis)
axes[0].plot(df['time'], df['setpoint[1]'], label='Setpoint', alpha=0.8)
axes[0].plot(df['time'], df['gyroADC[1]'], label='Gyro', alpha=0.8)
axes[0].set_ylabel('deg/s')
axes[0].legend()
axes[0].set_title('Pitch: Setpoint vs Gyro')

# Panel 2: Ringing Score
axes[1].plot(df['time'], df['debug[0]'], label='Ringing Score',
             color='red', drawstyle='steps-post')
axes[1].axhline(20, color='orange', linestyle='--', alpha=0.5, label='Threshold')
axes[1].axhline(10, color='green', linestyle='--', alpha=0.5, label='Threshold/2')
axes[1].set_ylabel('Score')
axes[1].legend()

# Panel 3: Zero Crossings and Amplitude
ax3b = axes[2].twinx()
axes[2].plot(df['time'], df['debug[1]'], label='Zero Crossings',
             color='blue', drawstyle='steps-post')
ax3b.plot(df['time'], df['debug[2]'], label='Amplitude',
          color='purple', drawstyle='steps-post', alpha=0.7)
axes[2].set_ylabel('ZC Count')
ax3b.set_ylabel('Amplitude (deg/s)')
axes[2].legend(loc='upper left')
ax3b.legend(loc='upper right')

# Panel 4: P and D Adjustments
axes[3].plot(df['time'], df['debug[3]'], label='P Adjustment',
             color='green', drawstyle='steps-post')
axes[3].plot(df['time'], df['debug[4]'], label='D Adjustment',
             color='orange', drawstyle='steps-post')
axes[3].set_ylabel('Adjustment')
axes[3].legend()

# Panel 5: Window Active
axes[4].fill_between(df['time'], 0, df['debug[5]'],
                     alpha=0.3, color='cyan', label='Window Active')
axes[4].set_ylabel('Window')
axes[4].set_yticks([0, 1])
axes[4].set_yticklabels(['Closed', 'Open'])
axes[4].legend()

# Panel 6: Phase and Assessment
axes[5].plot(df['time'], df['debug[6]'], label='Phase',
             color='black', drawstyle='steps-post')
axes[5].plot(df['time'], df['debug[7]'], label='Assessment',
             color='red', drawstyle='steps-post', alpha=0.7)
axes[5].set_ylabel('State')
axes[5].set_yticks([0, 1, 2, 3])
axes[5].set_yticklabels(['Phase1/WellDamp', 'Phase2/Mild', 'Phase3/Ringing', 'Complete'])
axes[5].set_xlabel('Time (s)')
axes[5].legend()

plt.tight_layout()
plt.savefig('phase2_analysis.png', dpi=150)
plt.show()
```

---

## Interpreting a Successful Phase 2 Run

A healthy Phase 2 log looks like this:

```
Time (maneuvers):
  #1-#8:   debug[6]=0                Phase 1: F-term converging
  #9:      debug[6] -> 1             Phase 2 starts
  #10:     debug[0]=45, debug[7]=2   Ringing detected, P decreased to -2
  #11:     debug[0]=30, debug[7]=1   Mild ringing, P decreased to -4
  #12:     debug[0]=12, debug[7]=0   Well damped! Bracket formed.
  #13:     debug[0]=22, debug[7]=1   Binary search: P=-3
  #14:     debug[0]=15, debug[7]=0   Converged at P=-3
  #14:     debug[6] -> 2             Phase 3 spot check
  #15-#17: debug[6]=2                F-term validation (3 maneuvers)
  #17:     debug[6] -> 3             COMPLETE
```

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2026-02-07 | 1.0 | Initial Phase 2 debug reference |
