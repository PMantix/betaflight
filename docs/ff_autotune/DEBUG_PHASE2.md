# FF Autotune Debug Reference

## Debug Mode Setup

In Betaflight CLI:
```
set debug_mode = FF_AUTOTUNE
save
```

All autotune phases are visible through this single debug mode. Channel 3 (`phase`) tells you which phase is active and how to interpret channels 4-7.

> **Note**: The old `FF_AUTOTUNE_PD` debug mode has been removed. All information is now consolidated into `FF_AUTOTUNE`.

---

## Channel Layout Overview

There are 8 debug channels. Channels 0-3 are **universal** (always the same meaning). Channels 4-7 are **phase-multiplexed** (their meaning depends on the current phase in channel 3).

### Universal Channels (always valid)

| Channel | Name | Range | Description |
|---------|------|-------|-------------|
| `debug[0]` | Gain | 0-200 | Current FF gain value for this axis |
| `debug[1]` | Tracking Error | +/-500 | Instantaneous `fabsf(gyroRate) - fabsf(setpoint)` in deg/s |
| `debug[2]` | Window State | 0-3 | Maneuver state machine (see Window State table) |
| `debug[3]` | Phase | 0-4 | Current autotune phase (see Phase table). **Use this to interpret channels 4-7.** |

### Window State (debug[2])

| Value | State | Description |
|-------|-------|-------------|
| **0** | `IDLE` | Waiting for maneuver: stick near center |
| **1** | `RISING` | Stick moving away from center, measuring tracking error |
| **2** | `ADJUSTING` | Rise ended, waiting 100ms then processing |
| **3** | `WAITING` | Waiting for stick to return to center |

### Phase (debug[3])

| Value | Phase | Description |
|-------|-------|-------------|
| **0** | `PHASE1_FF` | F-term gain tuning via tracking error bracketing |
| **1** | `PHASE2_PD` | Phase 2a: P/D balance (ringing suppression) |
| **2** | `PHASE2B_SCALE` | Phase 2b: P/D scale-down (D-term noise reduction) |
| **3** | `PHASE3_RECHECK` | F-term spot check after P/D changes |
| **4** | `COMPLETE` | All phases converged, no more adjustments |

---

## Phase-Specific Channels (debug[4]-debug[7])

### When Phase = 0 (Phase 1: F-term)

| Channel | Name | Range | Description |
|---------|------|-------|-------------|
| `debug[4]` | Last Avg Error | +/-500 | Average tracking error x10 from last completed maneuver |
| `debug[5]` | Bracket State | 0-2 | `SEARCHING`(0), `BRACKETED`(1), `CONVERGED`(2) |
| `debug[6]` | Last Assessment | -1, 0, +1 | `LAG`(-1), `OPTIMAL`(0), `LEAD`(+1) |
| `debug[7]` | History Count | 0-8 | Number of entries in the history ring buffer |

**What to look for**: `debug[5]` should progress from 0 -> 1 -> 2 as the F-term converges. `debug[6]` should alternate between -1 and +1 during bracketing, then settle to 0. When `debug[5]` = 2 (converged), the phase will transition to Phase 2a (if `pd_enabled`) or Complete.

### When Phase = 1 (Phase 2a: P/D Balance)

| Channel | Name | Range | Description |
|---------|------|-------|-------------|
| `debug[4]` | Ringing Score | 0-65535 | `zero_crossings * avg_peak_amplitude`. Higher = more ringing |
| `debug[5]` | P Adjustment | -20..0 | Cumulative P-term delta (negative = P reduced) |
| `debug[6]` | D Adjustment | 0..10 | Cumulative D-term delta (positive = D increased) |
| `debug[7]` | Ring Assessment | 0-2 | `WELL_DAMPED`(0), `MILD`(1), `RINGING`(2) |

**What to look for**: `debug[4]` (ringing score) should decrease as `debug[5]` (P adjustment) becomes more negative. `debug[7]` should go from 2 (ringing) toward 0 (well damped). When P bracket converges, transitions to Phase 2b.

### When Phase = 2 (Phase 2b: P/D Scale-Down)

| Channel | Name | Range | Description |
|---------|------|-------|-------------|
| `debug[4]` | Noise Score | 0-65535 | Average `|D-term|` x10 during maneuver rise. Lower = less motor noise |
| `debug[5]` | Scale Adjustment | -15..0 | Cumulative uniform P+D scale-down (negative = reduced) |
| `debug[6]` | Noise Baseline | 0-65535 | Baseline noise score measured before any scale-down |
| `debug[7]` | Noise Assessment | 0-2 | `HIGH`(0), `ACCEPTABLE`(1), `MINIMAL`(2) |

**What to look for**: `debug[4]` (noise score) should decrease relative to `debug[6]` (baseline) as `debug[5]` (scale adjustment) steps down. When noise improvement drops below threshold or `scale_max` is reached, transitions to Phase 3.

### When Phase = 3 (Phase 3: F-term Recheck)

| Channel | Name | Range | Description |
|---------|------|-------|-------------|
| `debug[4]` | Recheck Status | 0-1 | 0 = waiting for maneuver, 1 = collecting data |
| `debug[5]` | Error Drift | +/-500 | Difference between current avg error and Phase 1 converged error |
| `debug[6]` | (unused) | 0 | Reserved |
| `debug[7]` | (unused) | 0 | Reserved |

**What to look for**: After 3 maneuvers, if `debug[5]` (error drift) is within the deadband, the phase transitions to Complete. If it drifted, it goes back to Phase 1 with a narrow bracket for fast re-convergence.

### When Phase = 4 (Complete)

All channels 4-7 are zero. Tuning is done.

---

## Phase Flow Diagram

```
Phase 1 (FF)  -->  Phase 2a (P/D balance)  -->  Phase 2b (P/D scale)  -->  Phase 3 (recheck)  -->  Complete
                                                                                |
                                                                                v
                                                                          Phase 1 (FF)
                                                                       (only if F drifted)
```

### Phase Transition Conditions

| From | To | Condition |
|------|----|-----------|
| Phase 1 | Phase 2a | F-term bracket converged AND `pd_enabled = ON` |
| Phase 1 | Complete | F-term bracket converged AND `pd_enabled = OFF` |
| Phase 2a | Phase 2b | P (or D) bracket converged |
| Phase 2b | Phase 3 | Noise acceptable, or scale hit max, or noise increased |
| Phase 3 | Complete | F-term recheck within deadband (3 maneuvers) |
| Phase 3 | Phase 1 | F-term drifted beyond deadband (re-opens narrow bracket) |

---

## Ringing Analysis Window (Phase 2a)

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
2. **First peak skip**: The first zero-crossing marks the end of the initial overshoot (acceptable, NOT counted)
3. **Zero-crossing counting** (Schmitt trigger): Uses `ringLastSide` to latch which side of the deadband the signal was last on. A crossing is only registered when the signal reaches the *opposite* threshold (see Schmitt Trigger section below)
4. **Amplitude tracking**: After the first peak, max positive and negative error excursions are tracked
5. **Score**: `zero_crossings * ((peakPos - peakNeg) / 2)`

### Schmitt Trigger Zero-Crossing Detection

The zero-crossing counter uses a **latching Schmitt trigger** (hysteresis) with the `ring_deadband` as the threshold:

```
  ringLastSide:   +1                     -1                +1
                   |                      |                 |
     +db  --------|--\-------------------/---\-------------|--  (+deadband)
                      \               /       \          /
       0  ------------\-----------/--x---------\------/-----  (zero line)
                       \       /                 \  /
     -db  --------------\---/---------------------\/--------  (-deadband)
                         ↑                         ↑
                   crossing #1               crossing #2

Key insight: The signal must reach +deadband to latch side=+1, and then
must reach -deadband to register a crossing (and latch side=-1). While
the signal is between -deadband and +deadband, ringLastSide retains its
value — no crossings are counted in the dead zone.
```

**Why Schmitt trigger and not consecutive-sample comparison:**
A continuous error signal transitions through the deadband gradually (e.g., +6 → +3 → 0 → -3 → -6). Comparing consecutive samples `prev > +deadband && curr < -deadband` fails because the signal can't jump from above +5 to below -5 in a single PID loop sample at ~1kHz. The Schmitt trigger solves this by *remembering* which side was last reached, regardless of how many samples ago that was.

### Visual Example

```
Error signal in plateau window:

     +50 |      /\        /\
         |     /  \      /  \
  +db  --|----/----\----/----\------  (deadband = +5, latches side=+1)
     0   |--x------x--x------x-----  (zero-crossings after 1st: 4)
  -db  --|--------/----\----/-------  (deadband = -5, latches side=-1)
         |       /      \/
    -30  |      /
         |    /  <-- first peak (skipped)
         +-----------------------------> time
              ^                    ^
         window open          window close (150ms)
```

Score: 4 * ((50 - (-30)) / 2) = 4 * 40 = 160

---

## Noise Measurement (Phase 2b)

Unlike the ringing window (post-rise plateau), Phase 2b measures D-term noise **during the RISING phase** of the maneuver.

```
IDLE --> RISING (accumulate |pidData[axis].D|) --> ADJUSTING (compute score) --> WAITING --> IDLE
```

1. At `IDLE -> RISING`: noise accumulator resets, starts collecting `|D-term|` each PID loop iteration
2. During `RISING`: accumulates `fabsf(pidData[axis].D)` and increments sample count
3. At `RISING -> ADJUSTING`: noise score computed as `(accumulator / sampleCount) * 10`

### Phase 2b Algorithm

1. First maneuver: measure **baseline** noise score (no adjustment yet)
2. Apply `scaleAdjustment -= scale_step` (reduce both P and D uniformly)
3. Next maneuver: measure noise at new setting
4. If noise improved >= `noise_threshold`% vs baseline: continue reducing
5. If improvement < threshold OR hit `scale_max`: converge, move to Phase 3
6. If noise increased: revert last step, converge

The scale adjustment is **additive** on top of Phase 2a's P and D adjustments:
```
Effective P adjustment = pAdjustment + scaleAdjustment
Effective D adjustment = dAdjustment + scaleAdjustment
```
This preserves the P/D ratio found in Phase 2a while uniformly reducing both.

---

## CLI Parameters Reference

### Phase 1: F-term Tuning

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ff_autotune_enabled` | OFF | OFF/ON | Master enable |
| `ff_autotune_setpoint_low` | 100 | 50-300 | Min setpoint for tracking window (deg/s) |
| `ff_autotune_setpoint_high` | 600 | 300-1000 | Max setpoint for tracking window (deg/s) |
| `ff_autotune_min_accel` | 100 | 50-500 | Min acceleration (x100 = deg/s^2) |
| `ff_autotune_gain_step` | 5 | 1-20 | Initial gain step size |
| `ff_autotune_gain_max` | 200 | 50-255 | Maximum FF gain |
| `ff_autotune_gain_min` | 0 | 0-50 | Minimum FF gain |
| `ff_autotune_error_deadband` | 10 | 5-50 | Error deadband for "optimal" (deg/s) |
| `ff_autotune_converge_threshold` | 3 | 1-10 | Bracket width to declare converged |

### Phase 2a: P/D Balance (Ringing)

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ff_autotune_pd_enabled` | ON | OFF/ON | Enable Phase 2 P/D tuning |
| `ff_autotune_ring_window_ms` | 150 | 50-250 | Analysis window length after rise ends (ms) |
| `ff_autotune_ring_threshold` | 20 | 5-100 | Score above this = RINGING |
| `ff_autotune_ring_deadband` | 5 | 2-20 | Zero-crossing detection deadband (deg/s) |
| `ff_autotune_p_step` | 2 | 1-5 | P adjustment step per maneuver |
| `ff_autotune_d_step` | 1 | 1-3 | D adjustment step (fallback) |
| `ff_autotune_p_adjust_max` | 10 | 2-20 | Maximum cumulative P reduction |
| `ff_autotune_d_adjust_max` | 5 | 1-10 | Maximum cumulative D increase |

### Phase 2b: P/D Scale-Down (Noise)

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `ff_autotune_noise_threshold` | 10 | 5-30 | Noise improvement % required to continue reducing |
| `ff_autotune_scale_step` | 1 | 1-3 | Uniform P+D scale-down step per iteration |
| `ff_autotune_scale_max` | 8 | 2-15 | Maximum cumulative scale-down |

### Learned Values (saved to EEPROM)

| Parameter | Range | Description |
|-----------|-------|-------------|
| `ff_autotune_gain_roll` | 0-255 | Learned FF gain on roll |
| `ff_autotune_gain_pitch` | 0-255 | Learned FF gain on pitch |
| `ff_autotune_p_adj_roll` | -20..0 | Cumulative P reduction on roll |
| `ff_autotune_p_adj_pitch` | -20..0 | Cumulative P reduction on pitch |
| `ff_autotune_d_adj_roll` | 0..10 | Cumulative D increase on roll |
| `ff_autotune_d_adj_pitch` | 0..10 | Cumulative D increase on pitch |
| `ff_autotune_scale_adj_roll` | -15..0 | Cumulative P+D scale-down on roll |
| `ff_autotune_scale_adj_pitch` | -15..0 | Cumulative P+D scale-down on pitch |

---

## How to Analyze Blackbox Logs

### 1. Open in Blackbox Explorer

Load your `.bbl` or `.bfl` file. Add these traces:
- `debug[0]` - Gain (scale: 0-200)
- `debug[1]` - Tracking Error (scale: auto)
- `debug[2]` - Window State (scale: fixed 0-3)
- `debug[3]` - Phase (scale: fixed 0-4)
- `debug[4]` through `debug[7]` - Phase-specific (scale: auto)

Also overlay:
- `gyro[1]` (pitch gyro) and `setpoint[1]` (pitch setpoint)

### 2. Read Channel 3 First

Always look at `debug[3]` (phase) to know how to interpret channels 4-7:
- If `debug[3]` = 0: channels 4-7 show F-term tuning state
- If `debug[3]` = 1: channels 4-7 show ringing/P/D balance state
- If `debug[3]` = 2: channels 4-7 show noise/scale state
- If `debug[3]` = 3: channels 4-7 show recheck state

### 3. Verify Phase Progression

Plot `debug[3]` across the flight:
- 0 (Phase 1) should last 6-10 maneuvers
- 1 (Phase 2a) should last 5-8 maneuvers
- 2 (Phase 2b) should last 2-9 maneuvers (depends on noise)
- 3 (Phase 3) should last exactly 3 maneuvers
- 4 (Complete) should hold steady

---

## Interpreting a Successful Run

```
Time (maneuvers):
  #1-#8:   debug[3]=0                Phase 1: F-term converging
  #9:      debug[3] -> 1             Phase 2a starts
  #10:     debug[4]=45, debug[7]=2   Ringing detected, P decreased to -2
  #11:     debug[4]=30, debug[7]=1   Mild ringing, P decreased to -4
  #12:     debug[4]=12, debug[7]=0   Well damped! Bracket formed.
  #13:     debug[4]=22, debug[7]=1   Binary search: P=-3
  #14:     debug[4]=15, debug[7]=0   Converged at P=-3
  #14:     debug[3] -> 2             Phase 2b starts
  #15:     debug[4]=80, debug[6]=80  Baseline noise measured (score=80)
  #16:     debug[4]=65, debug[5]=-1  First scale-down, noise dropped to 65
  #17:     debug[4]=58, debug[5]=-2  Second scale-down, improvement < threshold
  #17:     debug[3] -> 3             Phase 3 starts
  #18-#20: debug[3]=3                F-term validation (3 maneuvers)
  #20:     debug[3] -> 4             COMPLETE
```

---

## Quick Troubleshooting

| Symptom | What to Check | Likely Fix |
|---------|---------------|------------|
| `debug[3]` stuck at 0 | Phase 1 not converging | Check `debug[5]` (bracket state), ensure maneuvers are big enough |
| `debug[3]` stuck at 1 | Phase 2a not converging | Check `debug[4]` (ringing score), try larger `p_step` or lower `ring_threshold` |
| `debug[3]` jumps 1 -> 2 -> 3 very fast | Phase 2b found no noise | Normal if D is already low, or quad is clean |
| `debug[3]` oscillates 0 <-> 3 | F-term keeps drifting | Increase `error_deadband` to reduce sensitivity |
| Phase 2a: `debug[4]` always 0 | No ringing detected | P/D ratio is already good, or `ring_window_ms` too short |
| Phase 2a: `debug[4]` always high | Can't suppress ringing | Increase `p_adjust_max`, or check mechanical issues |
| Phase 2b: `debug[4]` not decreasing | Noise not from D-term | Scale-down won't help; check motor/prop balance |
| Phase 2b: `debug[5]` hits -8 immediately | `scale_max` too low | Increase `scale_max` (try 12-15) |
| Sluggish quad after tuning | Too much P reduction + scaling | Decrease `p_adjust_max` and `scale_max`; reset learned values to 0 |
| Motor noise after tuning | D increased in 2a but not scaled enough in 2b | Increase `scale_max` or decrease `noise_threshold` |

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2026-02-07 | 1.0 | Initial Phase 2 debug reference |
| 2026-02-08 | 2.0 | Consolidated to single `FF_AUTOTUNE` debug mode (removed `FF_AUTOTUNE_PD`). Added Phase 2b (noise scale-down) documentation. Phase-multiplexed channel layout. |
| 2026-02-08 | 2.1 | Added Schmitt trigger zero-crossing documentation. Documented `ringLastSide` hysteresis latch replacing broken consecutive-sample comparison. |
