# Model-Based Feedforward (MBFF) v2

## Overview

MBFF is an advanced feedforward system that replaces classic Betaflight feedforward with a physics-based approach validated by blackbox log analysis.

**Key discovery:** Angular acceleration correlates with **squared eRPM differential**, not motor command:

```
delta(gyro) ∝ (eRPM²_right - eRPM²_left)   R² = 0.89-0.99
```

This makes physical sense: thrust ∝ RPM² (propeller aerodynamics).

**Why does this matter?** By using actual motor speed (eRPM) instead of commanded speed, MBFF bypasses ESC response time and thrust curve nonlinearities—resulting in highly accurate feedforward.

## Quick Start

1. **Enable MBFF:**
   ```
   set mbff_enable = ON
   save
   ```

2. **Enable online learning (recommended):**
   ```
   set mbff_learn_enable = ON
   save
   ```
   
   The system will automatically learn your quad's effectiveness coefficient during flight.

3. **Fly aggressively.** MBFF learns during rapid maneuvers (>400 deg/s setpoint).

**Requirement:** Bidirectional DSHOT for eRPM telemetry.

## How It Works

### The Delay Problem

Your gyro signal passes through multiple filters before the PID controller sees it:

```
Gyro → LPF1 → LPF2 → Notch → Dynamic Notch → ... → PID
```

This filtering introduces **4-8ms of delay**. When you move the stick, the PID doesn't "see" the quad's response until milliseconds later—causing overshoot and sluggish correction.

**Feedforward bypasses this delay** by computing an open-loop command directly.

### The Validated Physics Model

Blackbox analysis revealed a two-stage causal chain:

```
Relationship 1: delta(gyro) ~ eRPM²_diff        (R² = 0.89-0.99)
Relationship 2: eRPM_diff ~ delta(motor_diff)  (R² = 0.62-0.90)
```

**Physics explanation:**
1. **Thrust ∝ RPM²** — From propeller aerodynamics
2. **Motor has inertia** — Command controls rate of change of RPM, not RPM itself

### eRPM² Differential Formulas

Motor mapping (Betaflight Quad X):
```
M0 = rear-right (CW)    M1 = front-right (CCW)
M2 = rear-left (CCW)    M3 = front-left (CW)
```

```
Roll:  (eRPM[0]² + eRPM[1]²) - (eRPM[2]² + eRPM[3]²)   [right - left]
Pitch: (eRPM[0]² + eRPM[2]²) - (eRPM[1]² + eRPM[3]²)   [rear - front]
Yaw:   (eRPM[0]² + eRPM[3]²) - (eRPM[1]² + eRPM[2]²)   [CW - CCW]
```

### FF Computation

The physics model relates angular acceleration to eRPM² differential:

```
alpha = E_rpm2 × eRPM_sq_diff      [deg/s²]
u_ff = alpha × ff_scale
```

Where:
- `alpha` = angular acceleration (deg/s²)
- `E_rpm2` = effectiveness coefficient (≈0.006 for roll, ≈0.003 for pitch)
- `eRPM_sq_diff` = squared eRPM differential for the axis
- `ff_scale` = output scaling to mixer units

**Note on units:** E_rpm2 has units of (deg/s²) per (eRPM²_diff). The online learner time-scales the per-PID-loop gyro delta by multiplying by pidFrequency to get angular acceleration (deg/s²), ensuring learned E values match the Python-validated physics model.

## Configuration Parameters

### Core Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mbff_enable` | OFF | Enable model-based feedforward |
| `mbff_e_rpm2_roll` | 0.006 | eRPM² effectiveness for roll (deg/s² per eRPM²_diff) |
| `mbff_e_rpm2_pitch` | 0.003 | eRPM² effectiveness for pitch (deg/s² per eRPM²_diff) |
| `mbff_e_rpm2_yaw` | 0.003 | eRPM² effectiveness for yaw (deg/s² per eRPM²_diff) |
| `mbff_ff_scale` | 0.1 | Output scaling factor |
| `mbff_ff_limit` | 0.5 | FF output limit (0-1) |

### Learning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mbff_learn_enable` | ON | Enable online learning |
| `mbff_lambda` | 0.995 | Forgetting factor for RLS |
| `mbff_setpoint_thresh` | 400 | Min setpoint for learning (deg/s) |
| `mbff_gyro_delta_thresh` | 4 | Min gyro delta per PID loop (deg/s) |

**Note:** The E_rpm² values have units of (deg/s²) per (eRPM²_diff). They are small because eRPM² values are large (millions). The online learner time-scales gyro delta to angular acceleration, so learned E values match the Python-validated physics model.

## Online Learning

MBFF can automatically learn your quad's effectiveness coefficient (E_rpm²) during flight—no manual tuning required.

### How It Works

1. **Gate**: Check if conditions are valid (aggressive maneuvering, not crashed)
2. **Measure**: Observe eRPM²_diff and corresponding angular acceleration (d(gyro)/dt)
3. **Fit**: Use zero-intercept linear regression: `d(gyro)/dt = E × eRPM²_diff`
4. **Update**: Smoothly blend new estimate using RLS with forgetting factor

**Important:** The firmware measures gyro delta per PID loop (deg/s), then multiplies by `pidFrequency` to convert to angular acceleration (deg/s²). This ensures the learned E values match the Python-validated physics model (E ≈ 0.006 for roll, 0.003 for pitch).

The learned coefficient adapts to your specific quad, props, battery voltage, and flight conditions.

### Gating Conditions (Critical!)

Learning is **only valid** under specific conditions:

| Condition | Threshold | Why |
|-----------|-----------|-----|
| Active maneuvering | `\|setpoint\| > 400 deg/s` | Excludes hover noise |
| Not crashed | `\|gyro_product\| < 1e6` | Excludes tumble data |
| Accelerating | `\|delta(gyro)\| > 15` | Ensures measurable response |

**Critical:** The gyro delta threshold is per **PID loop iteration**, NOT per blackbox sample! Blackbox logs every `frameIntervalPDenom` PID loops (typically 4), so blackbox gyro deltas appear ~4x larger.

> **Implementation Note:** The `gyro_delta_thresh` default (4 deg/s) assumes a 4kHz PID loop. If using a different PID rate, this threshold may need adjustment. A future improvement would be to express this as angular acceleration (deg/s²) and scale automatically with PID frequency.

### Enabling Learning

```
set mbff_learn_enable = ON
set mbff_lambda = 995          # Forgetting factor (0.995)
set mbff_setpoint_thresh = 400 # Min setpoint (deg/s)
set mbff_gyro_delta_thresh = 4  # Min gyro delta per PID loop
save
```

Then fly with aggressive rolls and flips. Learning only occurs during rapid maneuvers.

### Expected Results

From blackbox analysis:
- **Valid samples**: ~0.4% of flight time passes gating
- **Typical yield**: 50-130 samples per 30-second aggressive flight
- **R² achieved**: 0.89-0.99 with proper gating

### Monitoring with Debug Mode

Use `set debug_mode = MBFF_LEARN` to monitor learning:

| Debug Slot | Value |
|------------|-------|
| debug[0] | Learned E_rpm² (roll) |
| debug[1] | Learned E_rpm² (pitch) |
| debug[2] | eRPM²_diff (current axis) |
| debug[3] | delta(gyro) measured |
| debug[4] | delta(gyro) predicted |
| debug[5] | Gate status (0 = learning active) |
| debug[6] | Sample count |
| debug[7] | R² estimate |

## Debug Modes

### DEBUG_MBFF

For monitoring FF output and behavior:

| Debug Slot | Value |
|------------|-------|
| debug[0] | eRPM²_diff roll |
| debug[1] | eRPM²_diff pitch |
| debug[2] | MBFF output roll |
| debug[3] | MBFF output pitch |
| debug[4] | Gyro delta (sample-to-sample) |
| debug[5] | Gate status |
| debug[6] | E_rpm² roll (×10000) |
| debug[7] | E_rpm² pitch (×10000) |

### DEBUG_MBFF_LEARN

See "Monitoring with Debug Mode" above.

## Tuning Guide

### If Using Online Learning (Recommended)

**You probably don't need to tune anything.** Just enable learning and fly aggressively. The system will figure out E_rpm² automatically.

If the quad feels sluggish initially:
- Increase `mbff_e_rpm2_roll` / `mbff_e_rpm2_pitch`
- Increase `mbff_ff_scale` for more overall FF authority

### Manual Tuning (Without Learning)

If you prefer manual tuning:

1. **Start with validated defaults:**
   ```
   set mbff_e_rpm2_roll = 0.006
   set mbff_e_rpm2_pitch = 0.003
   set mbff_ff_scale = 0.1
   ```

2. **Increase E_rpm²** if tracking feels sluggish

3. **Decrease E_rpm²** if over-rotating or oscillating

4. **Increase ff_scale** for more overall FF effect

### Signs of Good Tuning

- ✓ Stick response feels immediate
- ✓ Tracking is accurate at all rotation rates  
- ✓ Consistent feel across throttle range
- ✓ Less oscillation than with high P/D

### Signs of Poor Tuning

- ✗ Oscillation → reduce `mbff_ff_scale` or `mbff_e_rpm2_*`
- ✗ Sluggish response → increase `mbff_e_rpm2_*` or enable learning
- ✗ Inconsistent across throttle → verify eRPM telemetry is working
- ✗ Over-rotation at high rates → increase `mbff_b_init`

## Comparison with Classic FF

| Aspect | Classic FF | MBFF v2 |
|--------|-----------|------|
| **Input** | d(setpoint)/dt | eRPM² differential |
| **Model** | None (pure derivative) | Physics: thrust ∝ RPM² |
| **R² correlation** | ~0.15 (motor cmd) | 0.89-0.99 (eRPM²) |
| **Throttle adaptation** | Fixed or TPA | Automatic via eRPM |
| **Tuning** | ff_weight, ff_boost | E_rpm² (auto-learned) |
| **Requirement** | None | Bidirectional DSHOT |

## Requirements

- **Bidirectional DShot** - Required for RPM telemetry (used for effectiveness scaling)
- **STM32G4 or better** - Recommended for computational headroom
- Works without RPM but with reduced adaptation capability

## Files

| File | Description |
|------|-------------|
| `src/main/flight/mbff.h` | Header with types and API |
| `src/main/flight/mbff.c` | Core implementation |
| `docs/mbff/PRD.md` | Product requirements document |
| `docs/mbff/README.md` | This file |
