# FF Autotune Debug Channels Reference

## Debug Mode Setup

In Betaflight Configurator or CLI:
```
set debug_mode = FF_AUTOTUNE
save
```

## Debug Channel Mapping

| Channel | Name | Range | Description |
|---------|------|-------|-------------|
| `debug[0]` | Gain | 0-200 | Current FF gain value (100 = 1.0x feedforward) |
| `debug[1]` | Tracking Error | ±500+ | `fabsf(gyroRate) - fabsf(setpoint)` in deg/s × 10 |
| `debug[2]` | Window State | 0-3 | State machine state (see below) |
| `debug[3]` | Last Assessment | -1, 0, +1 | Result of last maneuver analysis |
| `debug[4]` | Avg Error | ±500 | Average tracking error from last maneuver × 10 |
| `debug[5]` | Sample Count | 0-1000+ | Number of samples in current maneuver |
| `debug[6]` | History Count | 0-8 | Entries in convergence history buffer |
| `debug[7]` | Bracket State | 0-2 | Gain search state (see below) |

---

## State Machine (debug[2])

| Value | State | Description |
|-------|-------|-------------|
| **0** | `IDLE` | Waiting for maneuver to start |
| **1** | `RISING` | Stick moving away from center, accumulating error |
| **2** | `ADJUSTING` | Maneuver ended, waiting 100ms before applying gain change |
| **3** | `WAITING` | Gain applied, waiting for stick to return to neutral |

### Expected State Flow
```
IDLE (0) → RISING (1) → ADJUSTING (2) → WAITING (3) → IDLE (0)
```

### State Transition Conditions

| From | To | Condition |
|------|-----|-----------|
| IDLE | RISING | `inSetpointWindow && magnitudeIncreasing && hasSignificantAccel` |
| RISING | ADJUSTING | `!magnitudeIncreasing OR !hasSignificantAccel` |
| ADJUSTING | WAITING | 100ms elapsed (then applies gain adjustment) |
| WAITING | IDLE | `isNeutral && !hasSignificantAccel` |

---

## Assessment Result (debug[3])

| Value | Meaning | Action Taken |
|-------|---------|--------------|
| **-1** | LAG | Gyro trailing setpoint → **increase** FF gain |
| **0** | OPTIMAL | Tracking is good → no change |
| **+1** | LEAD | Gyro leading setpoint → **decrease** FF gain |

---

## Bracket State (debug[7])

| Value | State | Description |
|-------|-------|-------------|
| **0** | `SEARCHING` | Looking for initial gain boundaries |
| **1** | `BRACKETED` | Found upper and lower bounds, binary searching |
| **2** | `CONVERGED` | Gain has stabilized within tolerance |

---

## How to Analyze Blackbox Logs

### 1. Open in Blackbox Explorer

Load your `.bbl` or `.bfl` file and add these traces:
- `debug[0]` - Gain
- `debug[2]` - State
- `debug[3]` - Assessment

### 2. Verify State Transitions

**Good behavior:**
- State cycles: 0 → 1 → 2 → 3 → 0
- State 1 (RISING) duration matches your stick input duration
- State 2 (ADJUSTING) lasts ~100ms
- State 3 (WAITING) lasts until stick returns to center

**Problems to look for:**
- Stuck in state 0: Thresholds too high, maneuvers not detected
- Stuck in state 1: Maneuver end not detected
- Rapid state cycling: Thresholds too sensitive

### 3. Check Gain Convergence

Watch `debug[0]` over multiple maneuvers:
- Should gradually stabilize
- Large oscillations indicate measurement noise
- Check `debug[7]` reaches 2 (CONVERGED)

### 4. Verify Assessment Logic

For each maneuver:
1. Find where `debug[2]` transitions from 1 → 2
2. Check `debug[3]` value at that point
3. Compare `debug[1]` (tracking error) during state 1:
   - Positive error (gyro > setpoint) → should assess as LEAD (+1)
   - Negative error (gyro < setpoint) → should assess as LAG (-1)

### 5. Sample Count Validation

Check `debug[5]` when state transitions from 1 → 2:
- Should be 10+ samples for valid measurement
- Very low counts may indicate noise triggering false maneuvers

---

## CLI Parameters Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ff_autotune_enabled` | OFF | Enable/disable autotune |
| `ff_autotune_threshold` | 200 | Minimum setpoint (deg/s) to trigger |
| `ff_autotune_rate` | 50 | Gain adjustment step size |
| `ff_autotune_min_samples` | 10 | Minimum samples for valid maneuver |

---

## Quick Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Never leaves IDLE | Threshold too high | Lower `ff_autotune_threshold` |
| Constant state cycling | Threshold too low | Raise `ff_autotune_threshold` |
| Gain oscillates wildly | Noisy measurements | Increase `ff_autotune_min_samples` |
| Gain never changes | Not detecting maneuvers | Check state machine in log |
| Assessment always same | Tracking error sign issue | Check `debug[1]` polarity |

---

## Example Analysis Script (Python)

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load blackbox CSV export
df = pd.read_csv('flight_log.csv')

# Plot key debug channels
fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

axes[0].plot(df['time'], df['debug[0]'], label='Gain')
axes[0].set_ylabel('FF Gain')
axes[0].legend()

axes[1].plot(df['time'], df['debug[1]'], label='Tracking Error', alpha=0.7)
axes[1].axhline(0, color='k', linestyle='--', alpha=0.3)
axes[1].set_ylabel('Error (deg/s × 10)')
axes[1].legend()

axes[2].plot(df['time'], df['debug[2]'], label='State', drawstyle='steps-post')
axes[2].set_ylabel('Window State')
axes[2].set_yticks([0, 1, 2, 3])
axes[2].set_yticklabels(['IDLE', 'RISING', 'ADJUSTING', 'WAITING'])
axes[2].legend()

axes[3].plot(df['time'], df['debug[3]'], label='Assessment', drawstyle='steps-post')
axes[3].set_ylabel('Assessment')
axes[3].set_yticks([-1, 0, 1])
axes[3].set_yticklabels(['LAG', 'OPTIMAL', 'LEAD'])
axes[3].set_xlabel('Time (s)')
axes[3].legend()

plt.tight_layout()
plt.savefig('ff_autotune_analysis.png', dpi=150)
plt.show()
```
