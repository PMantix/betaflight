# Model-Based Feedforward (MBFF) Prototype

## 1. Purpose

This document defines a **prototype Model-Based Feedforward (MBFF)** control term intended to **replace the existing Betaflight Feedforward (F) term** in rate mode.

The goal is to improve stick responsiveness, high-rate authority, and noise robustness by generating feedforward commands based on a **simple physical model and a reference trajectory**, rather than relying solely on the derivative of the rate setpoint.

This PRD is intended to guide an **experimental fork of Betaflight** and enable iterative flight testing on small multirotors (initially whoops).

---

## 2. Motivation

### 2.1 Limitations of Current Betaflight Feedforward

The current Betaflight F-term is primarily proportional to the **rate setpoint derivative**:

* Strong response during rapid stick movement
* Near-zero contribution when the stick is held at a constant position

In realistic stick shaping (cosine / eased ramps with a flat maximum):

* Feedforward is largest early in the maneuver
* Feedforward collapses near max stick / max rate
* Sustained torque demand must be provided by P/I/D

This results in:

* Degraded tracking at high rates
* Increased reliance on P and D gains
* Sensitivity to filtering and noise

### 2.2 Desired Behavior

A better feedforward term should:

* Respond immediately to **pilot intent** (stick movement)
* Continue providing **sustained authority** when the craft cannot reach the commanded rate
* Reduce required P/D gains
* Be robust to filtering and sensor noise
* Adapt naturally to throttle, battery sag, and motor effectiveness

---

## 3. High-Level Concept

The proposed MBFF replaces classic F with a **trajectory- and model-based feedforward** term:

1. Build an **internal reference rate trajectory** from the stick-derived rate setpoint
2. Compute the **angular acceleration required now** to follow that trajectory within a short preview horizon
3. Convert desired acceleration into an axis command using a **simple torque-effectiveness model**

Feedback PID remains in place for stability and disturbance rejection.

---

## 4. Control Architecture Overview

Per axis (roll, pitch, yaw):

```
RC Stick → Rate Setpoint (ω_sp)
              ↓
      Reference Trajectory (ω_ref)
              ↓
   Desired Acceleration (α_des)
              ↓
  Model-Based FF Command (u_FF)
              ↓
        + PID Output
              ↓
           Mixer → Motors
```

---

## 5. Reference Trajectory Model

### 5.1 Purpose

The reference trajectory represents **pilot intent**, not instantaneous command. It avoids explicit differentiation of the setpoint and provides a clean acceleration signal.

### 5.2 Definition

A first-order trajectory is used:

```
ω_ref[k+1] = ω_ref[k] + (dt / T_s) · (ω_sp − ω_ref[k])
```

Where:

* `ω_sp` is the standard Betaflight rate setpoint
* `ω_ref` is the internal reference rate
* `T_s` is the trajectory time constant

Typical values:

* Whoop racing: 15–20 ms
* Freestyle: 10–15 ms

---

## 6. Desired Acceleration Computation

The desired angular acceleration combines:

1. **Trajectory-following acceleration**
2. **Preview / tracking correction**

```
α_traj = (ω_sp − ω_ref) / T_s
α_prev = (ω_ref − ω_meas) / T_p

α_des = k_a · α_traj + k_r · α_prev
```

Where:

* `ω_meas` is gyro rate
* `T_p` is the preview horizon
* `k_a`, `k_r` are weighting gains

Key properties:

* No explicit d(ω_sp)/dt
* Feedforward remains active during held stick
* Anticipates required torque to meet near-future state

Typical values:

* `T_p`: 20–40 ms
* `k_a`: 1.0
* `k_r`: 0.5–1.2

---

## 7. Torque Effectiveness Model

### 7.1 Motivation

Motor/prop torque authority varies strongly with RPM, throttle, battery voltage, and loading. The feedforward command should be normalized by current effectiveness.

### 7.2 Model Definition

A minimal RPM-based gain model is used:

```
g(RPM²) = b0 + b1 · (RPM² / 1e6)
```

The feedforward axis command is:

```
u_FF = α_des / g(RPM²)
```

Where:

* `RPM²` is the average squared motor RPM from bidirectional DShot telemetry
* `b0`, `b1` are tunable constants

This model:

* Scales FF naturally with throttle and battery sag
* Avoids hard-coded FF tuning per setup
* Can later be adapted via online identification

---

## 8. Feedback PID Integration

* PID remains active and unchanged in structure
* PID tracks `ω_ref` (not directly `ω_sp`)
* D-term is computed on **gyro rate (measurement)** with existing filtering

```
error = ω_ref − ω_meas
u_PID = P(error) + I(error) − D(dω_meas/dt)

u_total = u_PID + u_FF
```

PID gains are expected to be **lower** than with classic FF.

---

## 9. Saturation and Safety Handling

### 9.1 Command Clamping

* `u_FF` must be clamped to a configurable fraction of max axis authority (e.g. 30–60%)
* Prevents model FF from demanding impossible torque

### 9.2 Learning / Adaptation (Future)

Although not required for the prototype, the design supports future online calibration of `b0`, `b1` using:

* Measured angular acceleration
* Axis command
* RPM telemetry

Learning must be gated when:

* Motors are saturated
* Throttle is too low
* Crash recovery is active

---

## 10. Integration into Betaflight

### 10.1 Replacement Strategy

* Disable or bypass existing Betaflight Feedforward term
* Insert MBFF computation in the same control stage where FF is currently applied

### 10.2 Required Signals

* Rate setpoint (`ω_sp`)
* Gyro rate (`ω_meas`)
* Loop dt
* Motor RPM telemetry (average RPM²)

### 10.3 Configuration Parameters (Initial)

Expose as CLI or feature flags:

* `mbff_enable`
* `mbff_ts`
* `mbff_tp`
* `mbff_ka`
* `mbff_kr`
* `mbff_b0`
* `mbff_b1`
* `mbff_ff_limit`

Defaults should be conservative.

---

## 11. Expected Benefits

* Improved high-rate tracking (especially at max stick)
* Reduced dependence on P and D gains
* Greater tolerance to filtering
* More consistent feel across throttle and battery states

---

## 12. Non-Goals (Prototype Phase)

* No full MPC
* No full aerodynamic modeling
* No mandatory online identification
* No change to angle / horizon outer loops

---

## 13. Validation Plan

Initial validation:

* Blackbox comparison vs classic FF
* Focus on:

  * High-rate holds
  * Noise sensitivity
  * Required P/D gains

Platforms:

* 65–75 mm whoops
* Bidirectional DShot, 48–96 kHz PWM

---

## 14. Status

**Prototype / Experimental**

Intended for a research fork of Betaflight, not immediate upstream inclusion.
