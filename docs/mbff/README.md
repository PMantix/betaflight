# Model-Based Feedforward (MBFF)

## Overview

MBFF is an experimental replacement for the classic Betaflight feedforward (F) term. Instead of computing feedforward from the rate setpoint derivative, MBFF uses:

1. **Reference Trajectory Model** - Smooth internal rate reference with configurable time constant
2. **Desired Acceleration** - Combines trajectory-following and preview/tracking correction
3. **Torque Effectiveness Model** - RPM-based scaling to adapt to throttle/battery state

## Quick Start

1. Enable MBFF via CLI:
   ```
   set mbff_enable = ON
   ```

2. Start with conservative defaults:
   ```
   set mbff_ts = 15        # Trajectory time constant (ms)
   set mbff_tp = 50        # Preview horizon (ms) - higher = less noise
   set mbff_ka = 100       # Trajectory acceleration gain (x100)
   set mbff_kr = 25        # Preview correction gain (x100) - lower = less noise
   set mbff_b0 = 100       # Base effectiveness (x100)
   set mbff_b1 = 50        # RPM-based effectiveness (x100)
   set mbff_ff_limit = 50  # FF limit as % of max axis authority
   set mbff_gain = 500     # Master output gain (x10, so 500 = 50.0x)
   ```

3. Reduce P and D gains (MBFF provides more sustained authority):
   ```
   # Try reducing P/D by 20-30% initially
   ```

## Configuration Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `mbff_enable` | OFF | ON/OFF | Enable model-based feedforward |
| `mbff_ts` | 15 | 5-50 | Trajectory time constant in ms |
| `mbff_tp` | 50 | 10-100 | Preview horizon in ms (higher = less noise) |
| `mbff_ka` | 100 | 0-200 | Trajectory acceleration gain (x100) |
| `mbff_kr` | 25 | 0-200 | Preview/tracking correction gain (x100, lower = less noise) |
| `mbff_b0` | 100 | 10-500 | Base torque effectiveness (x100) |
| `mbff_b1` | 50 | 0-200 | RPM-based effectiveness scaling (x100) |
| `mbff_ff_limit` | 50 | 10-100 | FF output limit as % of max axis authority |
| `mbff_gain` | 500 | 10-2000 | Master output gain (x10, so 500 = 50.0x) |
| `mbff_preview_threshold` | 50 | 0-200 | Setpoint rate-of-change threshold for preview (deg/s) |

## Debug Mode

Use `set debug_mode = MBFF` to log:
- debug[0]: Reference rate (ω_ref) [deg/s]
- debug[1]: Desired acceleration (α_des) [deg/s²]
- debug[2]: Torque effectiveness (g) [raw value]
- debug[3]: MBFF output (u_FF) [raw value]
- debug[4]: Classic FF output for comparison [raw value]
- debug[5]: Average RPM² / 1,000,000
- debug[6]: Trajectory error (ω_sp - ω_ref) [deg/s]
- debug[7]: Tracking error (ω_ref - ω_meas) [deg/s]

## Preview Term Gating

The preview term (`k_r * (ω_ref - gyro) / T_p`) helps MBFF respond to tracking errors during maneuvers. However, it can amplify gyro noise during steady-state (hover or sustained rolls).

**Solution:** The preview term is gated by the rate of change of the setpoint:
- When stick is **stationary** (hover or held in a roll): preview OFF → no noise amplification
- When stick is **moving** (transitions): preview ON → helps tracking

```
preview_scale = |d(setpoint)/dt| / (threshold × 100)
```

Adjust `mbff_preview_threshold` to tune sensitivity:
- **Higher values** (100-200): Preview activates only during aggressive stick movements
- **Lower values** (25-50): Preview activates during gentler transitions
- **0**: Disable gating (preview always on - may cause oscillation)

## Tuning Guide

### For Whoops (65-75mm)
- Start with `mbff_ts = 18-22` (slightly slower trajectory)
- `mbff_tp = 30-40` (longer preview for slower motors)
- Lower `mbff_b0` if too aggressive

### For Freestyle (5")
- `mbff_ts = 10-15` (faster trajectory)
- `mbff_tp = 20-30`
- Higher `mbff_b1` for more RPM adaptation

### Signs of Good Tuning
- Smooth tracking at high rates
- Less noise in motor outputs
- Consistent feel across throttle range
- Reduced need for high P/D

### Signs of Poor Tuning
- Oscillation (reduce `mbff_ka` or increase `mbff_ts`)
- Sluggish response (increase `mbff_ka`, decrease `mbff_ts`)
- Over-reaction to throttle (reduce `mbff_b1`)

## Theory

See [PRD.md](PRD.md) for detailed design rationale.

### Key Equations

**Trajectory Update:**
```
ω_ref[k+1] = ω_ref[k] + (dt / T_s) · (ω_sp − ω_ref[k])
```

**Desired Acceleration:**
```
α_traj = (ω_sp − ω_ref) / T_s
α_prev = (ω_ref − ω_meas) / T_p
α_des = k_a · α_traj + k_r · α_prev
```

**Torque Effectiveness:**
```
g(RPM²) = b0 + b1 · (RPM² / 1e6)
u_FF = gain · α_des / g(RPM²)
```

## Comparison with Classic FF

| Aspect | Classic FF | MBFF |
|--------|-----------|------|
| Input | d(setpoint)/dt | Setpoint + Gyro + RPM |
| Max stick behavior | Drops to zero | Maintains authority |
| Throttle adaptation | None (or separate TPA) | Built-in via RPM |
| Noise sensitivity | Higher (derivative) | Lower (integration) |
| Complexity | Simple | Moderate |

## Requirements

- Bidirectional DShot (for RPM telemetry)
- STM32G4 or better recommended
- Works without RPM but with reduced adaptation

## Known Limitations (Prototype)

- No yaw-specific tuning yet
- No online learning/adaptation
- Angle mode not yet integrated
- Single set of parameters for all axes

## Files

- `src/main/flight/mbff.h` - Header with types and API
- `src/main/flight/mbff.c` - Core implementation
- `docs/mbff/PRD.md` - Product requirements document
- `docs/mbff/README.md` - This file
