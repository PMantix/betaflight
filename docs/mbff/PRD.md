# Model-Based Feedforward (MBFF) v2

## 1. Purpose

MBFF is a **physics-based feedforward** system that uses the quadcopter's actual motor dynamics to compute optimal open-loop motor commands. Unlike classic Betaflight feedforward, MBFF:

1. **Uses real physics** — Validated model: angular acceleration ∝ eRPM² differential (R² = 0.89-0.99)
2. **Learns in-flight** — Adapts effectiveness coefficient to each airframe
3. **Compensates for gyro delay** — Bypasses the 4-8ms feedback path delay

**Key discovery (from blackbox analysis):** The relationship between motor command and angular acceleration follows a two-stage chain:

```
delta(motor_diff) → eRPM_diff → eRPM²_diff → delta(gyro)
```

This is a **dynamic** relationship — the *change* in motor command correlates with motor speed (not motor speed change), because motor command controls motor acceleration.

---

## 2. Motivation

### 2.1 The Gyro Delay Problem

The gyro signal passes through multiple filter stages before reaching the PID controller:

| Filter Stage | Type | Cutoff | Approx. Delay |
|-------------|------|--------|---------------|
| Hardware LPF | O1 | 250 Hz | ~0.6 ms |
| gyro_lpf1 | O1 | 150 Hz | ~1.1 ms |
| gyro_lpf2 | O2 biquad | 500 Hz | ~0.3 ms |
| Dynamic notches | Varies | Varies | ~2-3 ms |

D-term adds further delay:

| Filter Stage | Type | Cutoff | Approx. Delay |
|-------------|------|--------|---------------|
| dterm_lpf1 | O1 | 75 Hz | ~2.1 ms |
| dterm_lpf2 | O2 biquad | 250 Hz | ~0.6 ms |

**Total feedback path delay: 4-8 ms**

This delay means the PID controller is always reacting to where the quad *was*, not where it *is*. The result is phase lag, reduced tracking accuracy, and the need for conservative gains.

### 2.2 Why Feedforward Helps

Feedforward bypasses the feedback delay entirely by computing motor commands directly from the setpoint:

```
Setpoint → [Derivative] → [Model Inversion] → Motor Command
                ↓
         No filter delay!
```

When the pilot moves the stick, FF immediately commands the motors based on *what acceleration is needed*, without waiting for the gyro to measure the response.

### 2.3 Limitations of Classic Betaflight FF

Classic Betaflight feedforward is:

```
u_ff = ff_gain * d(setpoint)/dt
```

Problems:
- **No physics model**: The gain is a magic number that doesn't relate to actual motor effectiveness
- **Manual tuning required**: Different quads need different FF values
- **No damping compensation**: Ignores aerodynamic drag that opposes rotation
- **Battery/throttle variation**: Effectiveness changes but FF gain is constant

---

## 3. Physics Model

### 3.1 Original Single-Axis Model (Simplified)

The classic plant model assumes:

```
α = E·u - b·ω
```

Where:
- `α` = angular acceleration (deg/s²)
- `E` = motor effectiveness (deg/s² per unit command)
- `u` = axis command (mixer output, normalized -1 to +1)
- `b` = aerodynamic damping coefficient (1/s)
- `ω` = angular rate (deg/s)

However, **blackbox log analysis revealed a more accurate two-stage model** (see Section 12).

### 3.2 Validated Model: eRPM²-Based Dynamics

Analysis of real flight data shows angular acceleration correlates with **squared eRPM differential**, not motor command:

```
α = E_rpm² · (ΣeRPM²_right - ΣeRPM²_left)  [for roll axis]
```

**Physical basis:**
1. **Thrust ∝ RPM²** — From propeller aerodynamics, thrust scales with the square of rotational speed
2. **Torque = Thrust × arm** — Motor thrust creates torque about the center of mass
3. **α = Torque / Inertia** — Angular acceleration follows from Newton's second law

### 3.3 Two-Stage Causal Chain

The complete dynamics involve two relationships:

```
Relationship 1: delta(gyro) ~ eRPM²_diff        (R² = 0.89-0.99)
Relationship 2: eRPM_diff ~ delta(motor_diff)  (R² = 0.62-0.90)
```

**Key insight:** The *change* in motor command correlates with motor speed level (due to motor inertia dynamics). This is because motor command controls motor *acceleration*, so:

```
delta(motor_diff) → eRPM_diff → eRPM²_diff → delta(gyro)
```

In other words: changing the motor command changes the motor speed, and the squared motor speed differential creates angular acceleration.

### 3.4 Model Parameters

The physics model is:

```
d(gyro)/dt = E_rpm² × eRPM²_diff      [deg/s²]
```

| Parameter | Typical Value | Physical Meaning |
|-----------|---------------|------------------|
| E_rpm² (roll) | ~0.006 | Angular acceleration per unit eRPM²_diff (deg/s²) |
| E_rpm² (pitch) | ~0.003 | Angular acceleration per unit eRPM²_diff (deg/s²) |
| b | ~0 (TBD) | Aerodynamic damping (not yet characterized) |

**Note on units:** E has units of (deg/s²) / (eRPM²_diff). The values are small (~0.006) because eRPM² values are very large (millions).

**Sign convention:** Roll and pitch may have opposite signs due to motor geometry and gyro axis orientation.

### 3.5 eRPM Differential Definitions

Motor mapping (Betaflight Quad X):
```
M0 = rear-right (CW)    M1 = front-right (CCW)
M2 = rear-left (CCW)    M3 = front-left (CW)
```

eRPM² differential formulas:
```
Roll:  (eRPM[0]² + eRPM[1]²) - (eRPM[2]² + eRPM[3]²)   [right - left]
Pitch: (eRPM[0]² + eRPM[2]²) - (eRPM[1]² + eRPM[3]²)   [rear - front]
Yaw:   (eRPM[0]² + eRPM[3]²) - (eRPM[1]² + eRPM[2]²)   [CW - CCW]
```

---

## 4. Feedforward Computation

### 4.1 eRPM²-Based Feedforward (Recommended)

With bidirectional DSHOT, we can use the validated eRPM² relationship directly:

```
α_predicted = E_rpm² · eRPM²_diff
u_ff = α_predicted × ff_scale
```

Where:
- `eRPM²_diff` = real-time differential from DSHOT telemetry
- `E_rpm²` = learned effectiveness coefficient
- `ff_scale` = output scaling to mixer units

**Advantages:**
- Uses actual motor speed, not commanded speed
- Bypasses ESC response time and thrust curve nonlinearities
- Highest correlation with actual acceleration (R² = 0.89-0.99)

### 4.2 Motor-Command Fallback

If eRPM telemetry is unavailable, use the motor command derivative:

```
α_cmd = E_combined × d(motor_diff)/dt
u_ff = α_cmd × ff_scale
```

Where `E_combined ≈ 0.015` (from Slope1 × Slope2 chain).

**Note:** The relationship is **dynamic** — acceleration correlates with the *derivative* of motor command, not motor command itself.

### 4.3 Classic Setpoint-Based FF (Hybrid Approach)

The classic approach uses setpoint derivative:

```
α_cmd = d(ω_setpoint)/dt
u_ff = α_cmd / E
```

This can be combined with eRPM-based correction for best results.

### 4.4 Why Setpoint-Based FF Still Matters

1. **Zero latency** — Setpoint is known before motor response occurs
2. **No telemetry required** — Works without bidirectional DSHOT
3. **Predictive** — Anticipates required acceleration before motors spin up

The eRPM² relationship validates the *physics*, while setpoint-based FF provides the *prediction*.

---

## 5. Online Parameter Learning

### 5.1 What to Learn

With the eRPM² model, we learn a single coefficient per axis:

```
d(gyro)/dt = E_rpm² × eRPM²_diff      [deg/s²]
```

Where:
- `d(gyro)/dt` = angular acceleration (deg/s²) = gyroDelta × pidFrequency
- `E_rpm²` = effectiveness coefficient (deg/s² per eRPM²_diff)
- Typical values: E ≈ 0.006 for roll, 0.003 for pitch

**Critical implementation note:** The firmware measures gyro delta per PID loop iteration (deg/s). To get angular acceleration matching the Python-validated physics model, this delta must be multiplied by `pidFrequency` before feeding into RLS. Without this scaling, learned E values would be ~4000x smaller than expected.

### 5.2 Recursive Least Squares (Zero-Intercept)

For a zero-intercept model `y = slope × x`:

```
slope = Σ(x·y) / Σ(x²)
```

With recursive update and forgetting factor:

```
sum_xy = λ × sum_xy + x_new × y_new
sum_xx = λ × sum_xx + x_new × x_new
E_rpm2 = sum_xy / sum_xx
```

Where `λ ≈ 0.995` is the forgetting factor.

### 5.3 Gating Conditions (Critical!)

Learning is **only valid** under specific conditions. Improper gating destroys correlation:

| Condition | Threshold | Reason |
|-----------|-----------|--------|
| Active maneuvering | `\|setpoint\| > 400 deg/s` | Excludes hover noise |
| Not crashed | `\|gyro_product\| < 1e6` | Excludes tumble data |
| Accelerating | `\|gyroDelta\| > 4` per PID loop | Ensures measurable response |

**Critical:** The gyro delta threshold is per **PID loop iteration**, NOT per blackbox sample!

Blackbox logs every `frameIntervalPDenom` PID loops (typically 4), so:
- Blackbox gyro deltas appear ~4x larger than per-PID-loop deltas
- Threshold of 4 deg/s per PID loop ≈ 16 deg/s per blackbox sample

```c
// Gating uses per-PID-loop delta
float gyroDelta = gyro.gyroADCf[axis] - gyroPrev[axis];
if (fabsf(gyroDelta) < 4.0f) return;  // Skip learning

// But RLS uses TIME-SCALED delta for proper E values
float y = gyroDelta * pidFrequency;   // Convert to deg/s²
```

### 5.4 Convergence Properties

From blackbox analysis:
- **Valid samples**: ~0.4% of flight time passes gating
- **Typical yield**: 50-130 samples per 30-second flight
- **Required samples**: ~20 minimum for stable estimate
- **R² achieved**: 0.89-0.99 with proper gating

### 5.5 Multi-Parameter Learning (Future)

Once E_rpm² is validated, damping `b` can be added:

```
delta(gyro) = E_rpm² × eRPM²_diff - b × gyro
```

This becomes a 2-parameter regression. Start with `b = 0` until the basic model is working.

---

## 6. Integration with PID

MBFF adds to the existing PID output (and in doing so, replaces the original F term):

```
u_total = u_pid + u_ff

Where:
  u_pid = P·error + I·∫error - D·(dω_meas/dt)
  u_ff  = (α_cmd + b·ω_sp) / E
  error = ω_sp - ω_meas
```

**Key points:**
- PID handles disturbances, steady-state error, and model mismatch
- FF handles the predictable response to setpoint changes
- With accurate FF, PID gains can be reduced (less work for feedback)

---

## 7. Configuration Parameters

| Parameter | CLI Name | Default | Range | Description |
|-----------|----------|---------|-------|-------------|
| Enable | `mbff_enable` | OFF | ON/OFF | Master enable |
| E_rpm² (roll) | `mbff_e_rpm2_roll` | 0.006 | 0.001-0.02 | eRPM² effectiveness roll |
| E_rpm² (pitch) | `mbff_e_rpm2_pitch` | 0.003 | 0.001-0.02 | eRPM² effectiveness pitch |
| E_rpm² (yaw) | `mbff_e_rpm2_yaw` | 0.003 | 0.001-0.02 | eRPM² effectiveness yaw |
| FF Scale | `mbff_ff_scale` | 0.1 | 0.01-1.0 | Output scaling factor |
| FF Limit | `mbff_limit` | 0.5 | 0.1-1.0 | Max FF contribution |
| Learning | `mbff_learn` | ON | ON/OFF | Enable online learning |
| Learn rate | `mbff_lambda` | 0.995 | 0.99-0.999 | Forgetting factor |
| Setpoint gate | `mbff_setpoint_thresh` | 400 | 100-800 | Min setpoint for learning |
| Gyro delta gate | `mbff_gyro_delta_thresh` | 4 | 1-50 | Min gyro delta per PID loop |

**Note:** The E_rpm² values are small (0.003-0.006) because eRPM² values are large (millions). The sign of E_rpm² is handled internally based on axis.

---

## 8. Expected Benefits

| Metric | Improvement | Explanation |
|--------|-------------|-------------|
| Tracking error | 45-64% reduction | FF compensates for filter delay |
| Phase lag | Significant reduction | Open-loop path has no delay |
| Tune sensitivity | Lower | Learned model adapts to aircraft |
| PID gain requirements | Lower | FF does heavy lifting |

**Key insight from simulation**: Poorly-tuned PIDs benefit *more* from MBFF (64% improvement) than well-tuned PIDs (45% improvement). MBFF provides a safety net for suboptimal tuning.

---

## 9. Validation (Simulation Results)

Simulation parameters:
- Loop rate: 8 kHz
- Plant: E=20, b=2
- Gyro filters: O1@250Hz + O1@150Hz + O2@500Hz + notch delay
- D-term filters: O1@75Hz + O2@250Hz
- Setpoint: Step + sinusoidal tracking tests

### Results Summary

| Configuration | RMS Tracking Error | Improvement |
|--------------|-------------------|-------------|
| Good PID, no FF | 27.0 deg/s | baseline |
| Good PID + learned FF | 14.7 deg/s | **45%** |
| Bad PID, no FF | 71.3 deg/s | baseline |
| Bad PID + learned FF | 25.5 deg/s | **64%** |

### Learning Accuracy

| Parameter | True Value | Learned Value | Error |
|-----------|------------|---------------|-------|
| E | 20.0 | 19.96 | 0.2% |
| b | 2.0 | 1.98 | 1.0% |

---

## 10. Status

**Phase: Plant Model Validated, Ready for Implementation**

- [x] Physics model defined
- [x] FF computation implemented (simulation)
- [x] Online learning algorithm (simulation)
- [x] Simulation validation complete
- [x] Documentation compared against BetaFlight core implementation
- [x] Blackbox log analysis — plant model validated with R² = 0.89-0.99
- [x] Key discovery: eRPM² relationship, proper gating conditions
- [ ] **Next: Betaflight integration using eRPM²-based feedforward**
- [ ] Flight testing with learned parameters
- [ ] Tuning guide

### 10.1 Implementation Roadmap

| Phase | Task | Priority | Notes |
|-------|------|----------|-------|
| 1 | Implement eRPM²_diff calculation | HIGH | Roll/pitch axis differentials |
| 2 | Implement basic FF: `u_ff = E * eRPM²_diff` | HIGH | Use starting slopes from analysis |
| 3 | Add gating logic | HIGH | setpoint > 400, no crash, delta > 15 |
| 4 | Implement online learning (RLS) | MEDIUM | Refine E during flight |
| 5 | Add motor-command fallback | LOW | For non-DSHOT setups |
| 6 | Characterize damping (b) | LOW | May not be needed initially |

### 10.2 Open Questions

1. **eRPM telemetry latency** — What is the delay between motor speed and eRPM reading? This affects feedforward timing.

2. **Dynamic vs static model** — Analysis suggests `α ∝ d(motor_cmd)/dt`, not `α ∝ motor_cmd`. Need to verify this interpretation and determine if FF should use motor derivative.

3. **Yaw axis** — Insufficient data in current logs. Need flights with deliberate yaw spins.

4. **Throttle dependency** — Does E vary with throttle? Current analysis doesn't stratify by throttle level.

---

## 11. Betaflight Compatibility Analysis

### 11.1 Integration Point

MBFF integrates into the existing PID loop via the feedforward term:

```c
// In pid.c - classic feedforward:
pidData[axis].F = feedforwardGain * pidSetpointDelta;

// MBFF replacement:
pidData[axis].F = mbffUpdate(axis, setpoint, gyroRate, dT);
```

The feedforward is added to the PID sum:
```c
pidSum = P + I + D + F + S;  // F = feedforward
```

### 11.2 Available Signals

All required signals are available at the PID update point:
- `pidRuntime.setpointRate[axis]` - current setpoint (deg/s)
- `gyro.gyroADCf[axis]` - filtered gyro (deg/s)  
- `pidRuntime.dT` - loop period (seconds)
- `getMotorMixRange()` - for saturation awareness
- `getCoreTimeUs()` - microsecond timestamp for learning

### 11.3 Current MBFF v1 Code Location

The existing (deprecated) MBFF v1 implementation is in:
- [mbff.c](../../src/main/flight/mbff.c) - main algorithm
- [mbff.h](../../src/main/flight/mbff.h) - API and types

**Recommendation**: Update these files in-place for v2 using the same `pidData[axis].F` integration pattern.

---

## 12. Blackbox Log Analysis

### 12.1 Data Sources

Analyzed multiple MBFF test flights:
- `mbff_test_flight_1.csv` - 11.1s, roll maneuvers
- `mbff_test_flight_2.csv` - 35.0s, roll + pitch maneuvers
- `mbff_test_flight_3___learning.csv` - 35.2s, roll + pitch maneuvers  
- `mbff_test_flight_4___learning.csv` - 18.5s, roll maneuvers

Sample rate: ~1000 Hz, motor utilization: 27% average, 71% max.

### 12.2 Key Discovery: Two-Stage Physical Relationship

Analysis revealed a **two-stage causal chain** from motor command to angular acceleration:

```
Relationship 1: delta(gyro) ~ eRPM²_diff        (R² = 0.89-0.99)
Relationship 2: eRPM_diff ~ delta(motor_diff)  (R² = 0.62-0.90)
```

**Physics interpretation:**

1. **Thrust ∝ RPM²** — From propeller aerodynamics, thrust scales with the square of rotational speed. Therefore angular acceleration is proportional to the *squared* eRPM differential, not linear eRPM.

2. **Motor inertia dynamics** — Motor command doesn't instantly set motor speed. The motor has inertia, so motor command controls motor *acceleration*. This means `eRPM_diff ∝ delta(motor_diff)` — the change in motor command drives the motor speed.

**Combined chain:**
```
delta(motor_diff) → eRPM_diff → eRPM²_diff → delta(gyro)
```

### 12.3 Gating Conditions (Critical for High R²)

Proper gating is **essential** to achieve high correlation. The following conditions filter for valid physics:

| Condition | Threshold | Reason |
|-----------|-----------|--------|
| Active maneuvering | `|setpoint[axis]| > 400 deg/s` | Excludes hover noise |
| Not crashed | `|gyro[0] × gyro[1] × gyro[2]| < 1e6` | Excludes tumble data |
| Accelerating | `|delta(gyro)| > 15` | Ensures measurable response |

**Important:** The `delta(gyro) > 15` threshold is **sample-to-sample difference**, NOT time-scaled derivative. Using `d(gyro)/dt > 15` gives poor results because the actual values are ~10,000 deg/s².

### 12.4 Motor/eRPM Differential Definitions

Motor mapping (Betaflight Quad X):
```
M0 = rear-right (CW)    M1 = front-right (CCW)
M2 = rear-left (CCW)    M3 = front-left (CW)
```

Differential formulas:
```
Roll:   motor_diff = (M0+M1) - (M2+M3)    eRPM²_diff = (eRPM0²+eRPM1²) - (eRPM2²+eRPM3²)
Pitch:  motor_diff = (M0+M2) - (M1+M3)    eRPM²_diff = (eRPM0²+eRPM2²) - (eRPM1²+eRPM3²)
Yaw:    motor_diff = (M0+M3) - (M1+M2)    eRPM²_diff = (eRPM0²+eRPM3²) - (eRPM1²+eRPM2²)
```

### 12.5 Correlation Results

#### Relationship 1: delta(gyro) ~ eRPM²_diff

| Log File | Axis | R² | Slope | Samples |
|----------|------|-------|-------|---------|
| flight_1 | Roll | **0.89** | -0.0060 | 54 |
| flight_2 | Roll | 0.98* | -0.0054 | 129 |
| flight_3 | Roll | **0.96** | -0.0056 | 130 |
| flight_3 | Pitch | **0.99** | +0.0028 | 20 |

*flight_2 roll data had an anomaly in one subset

#### Relationship 2: eRPM_diff ~ delta(motor_diff)

| Log File | Axis | R² | Slope | Samples |
|----------|------|-------|-------|---------|
| flight_1 | Roll | **0.75** | -1.28 | 54 |
| flight_2 | Roll | **0.71** | -3.87 | 129 |
| flight_3 | Roll | **0.62** | -3.04 | 130 |
| flight_3 | Pitch | **0.90** | -8.36 | 20 |

**Interpretation:**
- Roll slope is negative, pitch slope (Rel 1) is positive — reflects motor geometry and gyro sign conventions
- Pitch achieves higher R² with fewer samples — stronger signal-to-noise ratio
- Slopes are consistent across flights, validating the linear model

### 12.6 Implications for MBFF Implementation

#### Option A: Use eRPM Directly (Recommended if Bidirectional DSHOT Available)

If the flight controller has access to real-time eRPM telemetry:

```c
// Direct feedforward using eRPM² relationship
float eRPM_sq_diff = computeERPMSquaredDiff(axis);
float alpha_predicted = E_rpm2 * eRPM_sq_diff;
float ff_output = alpha_predicted / mixer_authority;
```

**Advantages:**
- Highest accuracy (R² = 0.89-0.99)
- Bypasses ESC response time and thrust curve nonlinearities
- eRPM² directly represents thrust

**Disadvantages:**
- Requires bidirectional DSHOT (not all setups)
- eRPM telemetry latency must be characterized

#### Option B: Use Motor Command with Dynamic Model

If eRPM is unavailable, use the two-stage relationship to derive motor-command-based E:

```
E_combined = Slope1 × Slope2

Where:
  Slope1 = delta(gyro) per eRPM²_diff ≈ 0.005-0.006
  Slope2 = eRPM_diff per delta(motor_diff) ≈ 1-8

  E_combined ≈ 0.007 - 0.023 deg/s² per unit delta(motor_cmd_diff)
```

**Critical insight:** The relationship is **dynamic**, not static:
```
delta(gyro) ∝ delta(motor_cmd_diff)   NOT   delta(gyro) ∝ motor_cmd_diff
```

This means the original plant model `α = E·u - b·ω` needs modification:
```
α = E·(du/dt) - b·ω    (motor derivative, not motor command)
```

Or equivalently, integrate to get:
```
ω = E·u - ∫(b·ω)dt
```

### 12.7 Recommended Implementation Path

**Phase 1: Validate with eRPM (if available)**
1. Implement eRPM²_diff → delta(gyro) feedforward
2. Use Slope1 ≈ -0.006 for roll, +0.003 for pitch as starting values
3. Implement online learning to refine Slope1

**Phase 2: Motor-command fallback**
1. If eRPM unavailable, use motor command derivative
2. Start with E_combined ≈ 0.015
3. Be aware that R² will be lower (0.62-0.90 vs 0.89-0.99)

**Phase 3: Damping coefficient (b)**
1. The damping term `b·ω` was not strongly characterized in this analysis
2. Start with `b = 0` and let online learning find it
3. Alternatively, add `b·ω` to the regression once the E term is validated

### 12.8 Recommended Starting Values

```
mbff_E_rpm2_roll   = 0.006    // |delta(gyro)| per unit eRPM²_diff
mbff_E_rpm2_pitch  = 0.003    // (positive due to sign convention)
mbff_E_cmd_roll    = 0.015    // Combined motor-command E (fallback)
mbff_E_cmd_pitch   = 0.015    
mbff_damping       = 0.0      // Start at zero, let learning find it
mbff_lambda        = 0.995    // Forgetting factor for RLS
```

### 12.9 Flight Data Quality Recommendations

For best plant model identification:
- Include aggressive flips/rolls (>500 deg/s) ✓
- Include pitch flips (not just roll) ✓  
- Include yaw spins for yaw axis data
- 30-60 seconds of active flying
- Use bidirectional DSHOT for eRPM telemetry ✓
- Avoid crash/tumble data (gated automatically)
---

## 13. Implementation Guide

### 13.1 Where to Begin

The recommended starting point is a **minimal eRPM²-based feedforward** implementation:

```c
// In mbff.c - core feedforward computation

float mbffComputeFF(int axis, float setpointRate, float setpointDelta, float dT) {
    // 1. Compute eRPM² differential for this axis
    float eRPM_sq_diff = computeERPMSquaredDiff(axis);
    
    // 2. Predict angular acceleration from eRPM²
    float alpha_predicted = mbffConfig()->E_rpm2[axis] * eRPM_sq_diff;
    
    // 3. Convert to motor command units (normalized to mixer output)
    // For now, simple scaling - will be refined with learning
    float ff_output = alpha_predicted * mbffConfig()->ff_scale;
    
    // 4. Limit output
    return constrainf(ff_output, -mbffConfig()->ff_limit, mbffConfig()->ff_limit);
}

// eRPM² differential calculation
float computeERPMSquaredDiff(int axis) {
    float eRPM[4];
    for (int i = 0; i < 4; i++) {
        eRPM[i] = (float)getDshotTelemetry(i);  // Get eRPM from DSHOT telemetry
    }
    
    switch (axis) {
        case FD_ROLL:
            // Right (M0+M1) vs Left (M2+M3)
            return (sq(eRPM[0]) + sq(eRPM[1])) - (sq(eRPM[2]) + sq(eRPM[3]));
        case FD_PITCH:
            // Rear (M0+M2) vs Front (M1+M3)
            return (sq(eRPM[0]) + sq(eRPM[2])) - (sq(eRPM[1]) + sq(eRPM[3]));
        case FD_YAW:
            // CW (M0+M3) vs CCW (M1+M2)
            return (sq(eRPM[0]) + sq(eRPM[3])) - (sq(eRPM[1]) + sq(eRPM[2]));
        default:
            return 0.0f;
    }
}
```

### 13.2 Gating for Online Learning

When implementing online learning, use these gating conditions:

```c
bool mbffShouldLearn(int axis, float setpointRate, float gyroDelta) {
    // 1. Active maneuvering
    if (fabsf(setpointRate) < 400.0f) return false;
    
    // 2. Not crashed/tumbling (check gyro product)
    float gyroProduct = gyro.gyroADCf[0] * gyro.gyroADCf[1] * gyro.gyroADCf[2];
    if (fabsf(gyroProduct) > 1e6f) return false;
    
    // 3. Significant acceleration (sample-to-sample delta, NOT time-scaled!)
    if (fabsf(gyroDelta) < 15.0f) return false;
    
    return true;
}
```

**Critical:** The `gyroDelta` threshold of 15 is for **sample-to-sample difference**, not a time-scaled derivative. The raw sample-to-sample delta is what correlates with eRPM²_diff.

### 13.3 File Locations

Existing MBFF v1 code to update:
- [mbff.c](../../src/main/flight/mbff.c) — Main algorithm
- [mbff.h](../../src/main/flight/mbff.h) — API and types

Integration point in PID:
- `pid.c` — Replace `pidData[axis].F` calculation

DSHOT telemetry access:
- `dshot.h` — `getDshotTelemetry(motorIndex)` for eRPM values

### 13.4 Configuration Parameters to Add

```c
typedef struct mbffConfig_s {
    uint8_t  enabled;                    // Master enable
    float    E_rpm2[3];                  // eRPM² effectiveness per axis
    float    damping;                    // Aerodynamic damping (start at 0)
    float    ff_scale;                   // Output scaling
    float    ff_limit;                   // Max FF contribution
    uint8_t  learning_enabled;           // Enable online learning
    float    lambda;                     // RLS forgetting factor
} mbffConfig_t;
```

### 13.5 Testing Checklist

**Phase 1: Verify eRPM² calculation**
- [ ] Log eRPM²_diff values during flight
- [ ] Confirm sign conventions match expected roll/pitch directions
- [ ] Verify eRPM telemetry is updating at loop rate

**Phase 2: Basic feedforward**
- [ ] Start with small ff_scale (0.1)
- [ ] Fly gentle rolls/pitches
- [ ] Check if FF output correlates with stick input
- [ ] Gradually increase ff_scale

**Phase 3: Tune E values**
- [ ] If response feels weak, increase E_rpm2
- [ ] If response overshoots, decrease E_rpm2
- [ ] Roll and pitch may need different values

**Phase 4: Online learning**
- [ ] Enable learning
- [ ] Fly aggressive maneuvers to trigger gating
- [ ] Monitor learned E values in blackbox
- [ ] Verify convergence and stability

---

## 14. Analysis Scripts

The following Python scripts in `bb logs/` were used for this analysis:

| Script | Purpose |
|--------|---------|
| `analyze_erpm_squared.py` | Verify eRPM² relationships for roll axis |
| `analyze_erpm_squared_both_axes.py` | Analyze both roll and pitch axes |
| `analyze_erpm_all_logs.py` | Multi-log eRPM analysis with gating |

To reproduce the analysis:
```bash
cd "bb logs"
python analyze_erpm_squared_both_axes.py
```

Output plots are saved to `erpm_squared_both_axes.png`.