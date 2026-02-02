# Autotune V2 Debug Cheat Sheet

Quick reference for interpreting debug values during autotune operation.

## Enable Debug Mode

```
set debug_mode = AUTOTUNE
save
```

## Debug Channels Overview

| Channel | Name | General Meaning |
|---------|------|-----------------|
| `debug[0]` | State | Current autotune state (0-5) |
| `debug[1]` | Axis | Current axis being tuned (0=Roll, 1=Pitch, 2=Yaw) |
| `debug[2]` | Reason | Reason code for last action |
| `debug[3]` | Progress | State-specific progress indicator |
| `debug[4]` | Metric A | State-specific metric (see below) |
| `debug[5]` | Metric B | State-specific metric (see below) |
| `debug[6]` | Metric C | State-specific: Filter change OR D gain |
| `debug[7]` | Metric D | State-specific: Filter value OR F gain |

**Note:** debug[6] and debug[7] meanings change based on state:
- **THROTTLE_SWEEP**: Filter change reason / value
- **PD_RATIO_SEEK, PD_SCALE_UP**: D gain / F gain

---

## State Values (debug[0])

| Value | State | Description |
|-------|-------|-------------|
| 0 | IDLE | Waiting for arm + switch |
| 1 | HOVER_STABILIZE | Establishing stable hover baseline |
| 2 | THROTTLE_SWEEP | Characterizing motor noise profile |
| 3 | WAIT_FOR_IDLE | Waiting for sticks centered |
| 4 | PD_RATIO_SEEK | Finding optimal P/D ratio |
| 5 | PD_SCALE_UP | Scaling gains until oscillation limit |

---

## State-Specific Debug Meanings

### State 0: IDLE
| Channel | Meaning |
|---------|---------|
| `debug[3-5]` | Not used (0) |

### State 1: HOVER_STABILIZE
| Channel | Meaning |
|---------|---------|
| `debug[3]` | Hover progress (0-100) |
| `debug[4]` | Noise level estimate |
| `debug[5]` | Not used |

### State 2: THROTTLE_SWEEP ⭐
| Channel | Meaning |
|---------|---------|
| `debug[3]` | Sweep progress: `sweepCount×100 + hoverProgress` |
| `debug[4]` | **Peak frequency (Hz)** from FFT analysis |
| `debug[5]` | **Peak magnitude** (noise strength at peak freq, sqrt-scaled ×10) |
| `debug[6]` | **Filter change reason code** (see table below) |
| `debug[7]` | **Filter change value** (the Hz value applied) |

**Example:** `debug[3]=347` means sweep #3, 47% hover progress

**Filter Change Reason Codes (debug[6]):**
| Code | Meaning |
|------|---------|
| 0 | No change |
| 1 | LPF1 cutoff adjusted |
| 2 | LPF2 cutoff adjusted |
| 3 | DynNotch min Hz adjusted |
| 4 | DynNotch max Hz adjusted |
| 5 | Static notch 1 added/adjusted |
| 6 | Static notch 2 added/adjusted |

### State 3: WAIT_FOR_IDLE
| Channel | Meaning |
|---------|---------|
| `debug[3-5]` | Not used (0) |

### State 4: PD_RATIO_SEEK ⭐
| Channel | Meaning |
|---------|---------|
| `debug[3]` | Number of valid events captured |
| `debug[4]` | **Last overshoot % (×10)** |
| `debug[5]` | P/D ratio (×10) |
| `debug[6]` | **Current D gain** |
| `debug[7]` | **Current F gain** |

**Example:** `debug[4]=153` means 15.3% overshoot

### State 5: PD_SCALE_UP ⭐
| Channel | Meaning |
|---------|---------|
| `debug[3]` | Scale factor (×100) |
| `debug[4]` | Oscillation detected (0/1) |
| `debug[5]` | Max safe gain found (×10) |
| `debug[6]` | **Current D gain** |
| `debug[7]` | **Current F gain** |

---

## Quick Validation Checks

### Filter Characterization Working?
```
During State 2 (THROTTLE_SWEEP):
✓ debug[4] shows values 80-300 Hz (typical motor noise)
✓ debug[5] shows peak magnitude (noise strength)
✓ debug[6]/[7] show filter changes when applied
✗ debug[4] = 0 throughout → FFT not reading peaks
```

### Overshoot Detection Working?
```
During State 4 (PD_RATIO_SEEK):
✓ debug[4] shows varying values (20-300 range typical)
✓ Values change after each stick flick
✗ debug[4] = 0 always → Events not capturing
```

### Gains Adjusting?
```
During State 5 (PD_SCALE_UP):
✓ debug[2] increases over time
✓ debug[3] shows scale factor > 100
✗ debug[2] stays constant → Gains not scaling
```

---

## Typical Value Ranges

| Metric | Healthy Range | Warning |
|--------|---------------|---------|
| Peak Frequency | 80-300 Hz | <60 or >400 Hz unusual |
| Overshoot % | 5-25% | >40% may indicate instability |
| D Gain (×10) | 20-60 | <10 very low, >80 very high |
| P/D Ratio (×10) | 80-150 | <50 or >200 unusual |

---

## Blackbox Field Names

When analyzing in Blackbox Explorer:

| Debug Channel | Blackbox Field |
|---------------|----------------|
| `debug[0]` | `debug[0]` or `debug_0` |
| `debug[1]` | `debug[1]` or `debug_1` |
| `debug[2]` | `debug[2]` or `debug_2` |
| `debug[3]` | `debug[3]` or `debug_3` |
| `debug[4]` | `debug[4]` or `debug_4` |
| `debug[5]` | `debug[5]` or `debug_5` |

---

## Python Analysis Snippet

```python
import pandas as pd

# Load log (adjust skiprows as needed for header)
df = pd.read_csv('flight_log.csv', skiprows=147)

# Filter by state
state_2 = df[df['debug[0]'] == 2]  # THROTTLE_SWEEP
state_4 = df[df['debug[0]'] == 4]  # PD_RATIO_SEEK

# Check filter characterization
print(f"Peak freq range: {state_2['debug[4]'].min()}-{state_2['debug[4]'].max()} Hz")
print(f"Throttle coverage: {state_2['debug[5]'].max()}%")

# Check overshoot values
overshoots = state_4['debug[4]'].unique()
print(f"Overshoot values: {[v/10 for v in overshoots if v > 0]}%")
```

---

## Common Issues

| Symptom | Likely Cause |
|---------|--------------|
| State stuck at 0 | Switch not enabled or not armed |
| State stuck at 1 | Not hovering (throttle too low/high) |
| Peak freq = 0 | Dynamic notch disabled or FFT not running |
| Overshoot = 0 | Stick flicks too small or not detected |
| D gain not changing | Overshoot target already met |

---

*Last updated: 2026-01-25*
