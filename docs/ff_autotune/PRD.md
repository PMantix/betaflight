# FF Autotune v3 - Product Requirements Document

## Overview

FF Autotune v3 is a simplified, empirical approach to feedforward gain tuning that abandons complex physics-based models in favor of direct setpoint tracking measurement.

### Background

Previous attempts (MBFF v1, v2) tried to learn a forward model of motor dynamics and invert it to compute feedforward. These approaches failed due to:

1. **Scaling/sign confusion**: The forward model learned from gyro dynamics, not FF requirements
2. **Anti-correlation**: Phase 2 forward model had -0.44 correlation with classic FF (wrong direction!)
3. **Complexity**: E-learning, eRPM differentials, and plant inversion introduced too many failure modes

### The v3 Philosophy

**"Measure what matters, adjust what works."**

Instead of trying to model the physics, we directly measure setpoint tracking quality during maneuvers and adjust FF gain until tracking is optimal. This is the same approach a human tuner would use.

---

## Functional Requirements

### FR1: Error-Based D-Term

**Critical Change:** When FF Autotune is active, switch D-term from gyro-based (default BF) to error-based.

**Default Betaflight:**
```
D_term = Kd * derivative(gyro_rate)
```

**With FF Autotune Active:**
```
D_term = Kd * derivative(setpoint - gyro_rate)
```

**Rationale:**
- Gyro-based D only fights disturbances, not tracking errors
- Error-based D actively corrects tracking lag/lead
- Combined with FF tuning, this gives better setpoint following
- Error-based D can be noisier, but during tuning we need accurate tracking measurement

### FR2: Setpoint Tracking Monitor

During rapid maneuvers, measure the tracking error between gyro rate and setpoint.

**Tracking Window Criteria:**
- Setpoint magnitude: 100 - 600 deg/s (configurable)
- Minimum setpoint acceleration: 10,000 deg/s² (configurable)
- This window captures the "ramp up" portion of maneuvers where FF matters most

**Error Measurement:**
```
tracking_error = gyro_rate - setpoint
```
- Negative error = gyro lagging setpoint = need MORE FF
- Positive error = gyro leading/overshooting = need LESS FF

### FR3: Gain Adjustment Logic

At the end of each maneuver (when setpoint drops below tracking window):

1. Calculate average tracking error over the maneuver
2. If average error < -deadband: increase FF gain by step
3. If average error > +deadband: decrease FF gain by step
4. If within deadband: no change (optimal)

**Safety Limits:**
- gain_min: 0 (default)
- gain_max: 200 (default, configurable)
- gain_step: 2 (default, configurable)
- error_deadband: 10 deg/s (default, configurable)

### FR4: Per-Axis Learning

- Roll and Pitch have independent FF gains
- Yaw is excluded (different dynamics, typically doesn't need FF tuning)
- Each axis tracks its own maneuver state and adjusts independently

### FR5: Flight Mode Control

**Activation:**
- Controlled via AUX switch mapped to `BOXFFAUTOTUNE`
- Only active when: switch ON + armed + feature enabled

**Learning Behavior:**
- While mode active: continuously monitor and make small adjustments
- Adjustments happen at end of each qualifying maneuver
- No limit on number of adjustments per flight

### FR6: EEPROM Save Policy

**Critical Requirement:** Only save to EEPROM when mode switches from ACTIVE to INACTIVE.

Rationale:
- Prevents flash wear from continuous writes
- Gives pilot control over when gains are committed
- If gains go bad, pilot can disarm without saving

**Save Flow:**
1. Pilot activates FF Autotune mode (switch ON)
2. Learning occurs during flight
3. Pilot deactivates mode (switch OFF)
4. If gains changed → write to EEPROM + confirmation beep
5. If gains unchanged → no write

### FR7: Debug Output

Use DEBUG_FF_AUTOTUNE mode with persistent, meaningful channels for blackbox analysis.

**Debug Channels (per debug axis):**

| Channel | Name | Description | Units |
|---------|------|-------------|-------|
| 0 | `gain` | Current FF gain for this axis | 0-255 |
| 1 | `tracking_error` | Instantaneous (gyro - setpoint) | deg/s |
| 2 | `window_state` | Tracking window state machine | enum |
| 3 | `assessment` | Current assessment: -1=lag, 0=optimal, +1=lead | signed |
| 4 | `avg_error` | Average error from last completed maneuver | deg/s × 10 |
| 5 | `maneuver_samples` | Sample count in current/last maneuver | count |
| 6 | `history_idx` | Current position in history ring buffer | 0-7 |
| 7 | `bracket_state` | Bracketing state: 0=searching, 1=bracketed, 2=converged | enum |

**Window State Values (channel 2):**
- 0 = IDLE (setpoint below threshold)
- 1 = RISING (in tracking window, acceleration positive)
- 2 = PEAK (in window, acceleration slowing)
- 3 = FALLING (exiting window)
- 4 = SETTLING (post-maneuver settle time)
- 5 = ADJUSTING (applying gain change)

**Assessment Values (channel 3):**
- -100 to -1 = LAG (gyro behind setpoint, need more FF)
- 0 = OPTIMAL (within deadband)
- +1 to +100 = LEAD (gyro ahead/overshoot, need less FF)

This allows seeing in blackbox exactly when tracking windows are detected, the real-time assessment, and how the system is converging.

### FR8: Performance History & Bracketing

Maintain a history of recent gain settings and their tracking performance to enable intelligent bracketing convergence.

**History Ring Buffer:**
- Store last 8 (gain, avg_error) pairs per axis
- Updated after each qualifying maneuver
- Persists across mode activations within same flight (not across reboots)

**Bracketing Logic:**

Instead of blindly stepping up/down, use history to bracket the optimal gain:

```
Phase 1: SEARCHING (no bracket established)
  - Step in direction indicated by error
  - If we see error sign flip (lag → lead or lead → lag):
    → Bracket established between last two gains

Phase 2: BRACKETED (optimal is between two known gains)
  - Binary search within bracket
  - Test midpoint gain
  - Narrow bracket based on result
  - Continue until bracket width ≤ gain_step

Phase 3: CONVERGED
  - Bracket is narrow enough
  - Use midpoint as final gain
  - Stop adjusting (or switch to fine-tuning mode with tiny steps)
```

**History Structure:**
```c
typedef struct {
    uint8_t gain;           // Gain setting used
    int16_t avgError;       // Average tracking error (×10 for precision)
    uint16_t sampleCount;   // How many samples in this measurement
    uint8_t valid;          // Entry is valid
} ffAutotuneHistoryEntry_t;

typedef struct {
    ffAutotuneHistoryEntry_t entries[8];
    uint8_t count;          // Number of valid entries
    
    // Bracket state
    uint8_t lowerGain;      // Known gain that causes lag (best lag sample)
    uint8_t upperGain;      // Known gain that causes lead (best lead sample)
    int16_t lowerError;     // Error at lower bound (negative = lag)
    int16_t upperError;     // Error at upper bound (positive = lead)
    bool bracketed;         // Bracket established
    bool converged;         // Converged to optimal
} ffAutotuneHistory_t;
```

**Intelligent History Management:**

When adding a new sample to a full buffer, use smart eviction:

```
1. Count samples by type:
   - lag_count = entries with avgError < -deadband
   - optimal_count = entries with |avgError| <= deadband  
   - lead_count = entries with avgError > +deadband

2. Determine new sample type (lag/optimal/lead)

3. Eviction priority (evict from SAME type as new sample):
   - If new sample is LAG: evict oldest LAG (keep lead samples!)
   - If new sample is LEAD: evict oldest LEAD (keep lag samples!)
   - If new sample is OPTIMAL: evict oldest OPTIMAL
   
4. Exception - preserve bracket bounds:
   - NEVER evict the sample that defines lowerGain (best lag)
   - NEVER evict the sample that defines upperGain (best lead)
   
5. Fallback if same-type eviction not possible:
   - Evict oldest sample that is NOT a bracket bound
```

**Example - Why This Matters:**

```
History (full, 8 entries):
  [0] gain=5,  error=-40 (LAG)  ← Only lag sample! Defines lower bracket!
  [1] gain=15, error=+12 (LEAD)
  [2] gain=20, error=+18 (LEAD)
  [3] gain=25, error=+22 (LEAD)
  [4] gain=30, error=+15 (LEAD)
  [5] gain=35, error=+20 (LEAD)
  [6] gain=40, error=+25 (LEAD)
  [7] gain=45, error=+30 (LEAD)

New sample: gain=50, error=+35 (LEAD)

WRONG (simple ring buffer): Evict [0] → lose only lag sample → bracket broken!

RIGHT (intelligent eviction): 
  - New sample is LEAD
  - Find oldest LEAD that is NOT upperGain
  - upperGain=15 (lowest lead = tightest bound)
  - Evict [2] (gain=20, oldest non-bracket LEAD)
  - Bracket preserved!
```

**Bracket Bound Updates:**

When adding a new sample, also update bracket bounds if it improves them:

```c
// For LAG samples (negative error): want HIGHEST gain that still lags
if (newError < -deadband && newGain > lowerGain) {
    lowerGain = newGain;
    lowerError = newError;
}

// For LEAD samples (positive error): want LOWEST gain that leads
if (newError > +deadband && newGain < upperGain) {
    upperGain = newGain;
    upperError = newError;
}

// Check if bracket is now established
if (lowerGain > 0 && upperGain < 255 && lowerGain < upperGain) {
    bracketed = true;
}
```

**Benefits:**
1. **Faster convergence**: Binary search is O(log n) vs linear O(n)
2. **Avoids oscillation**: Won't keep stepping past optimal
3. **Robust**: If a maneuver gives noisy data, history provides context
4. **Debuggable**: Can see exactly how system bracketed in blackbox

**Example Convergence:**
```
Maneuver 1: gain=0,   error=-45 (lag)     → step up
Maneuver 2: gain=10,  error=-30 (lag)     → step up  
Maneuver 3: gain=20,  error=-15 (lag)     → step up
Maneuver 4: gain=30,  error=+8  (lead)    → BRACKET [20,30]
Maneuver 5: gain=25,  error=-5  (lag)     → narrow to [25,30]
Maneuver 6: gain=27,  error=+2  (optimal) → CONVERGED at 27
```

---

## Configuration Parameters

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `ff_autotune_enabled` | bool | OFF | OFF/ON | Master enable |
| `ff_autotune_setpoint_low` | uint16 | 100 | 50-300 | Min setpoint for tracking window (deg/s) |
| `ff_autotune_setpoint_high` | uint16 | 600 | 300-1000 | Max setpoint for tracking window (deg/s) |
| `ff_autotune_min_accel` | uint16 | 100 | 50-500 | Min accel for tracking (×100 = deg/s²) |
| `ff_autotune_gain_step` | uint8 | 5 | 1-20 | Initial gain step (larger for faster search) |
| `ff_autotune_gain_max` | uint8 | 200 | 50-255 | Maximum allowed FF gain |
| `ff_autotune_gain_min` | uint8 | 0 | 0-50 | Minimum allowed FF gain |
| `ff_autotune_error_deadband` | uint8 | 10 | 5-50 | Error deadband for "optimal" (deg/s) |
| `ff_autotune_converge_threshold` | uint8 | 3 | 1-10 | Bracket width to declare converged |
| `ff_autotune_gain_roll` | uint8 | 0 | 0-255 | Learned roll FF gain |
| `ff_autotune_gain_pitch` | uint8 | 0 | 0-255 | Learned pitch FF gain |

---

## Implementation Architecture

### New Files

```
src/main/flight/ff_autotune.h    - Public API
src/main/flight/ff_autotune.c    - Core implementation
src/main/pg/ff_autotune.h        - Config structure
src/main/pg/ff_autotune.c        - PG registration
```

### Modified Files

```
src/main/target/common_pre.h     - Add USE_FF_AUTOTUNE
src/main/fc/rc_modes.h           - Add BOXFFAUTOTUNE
src/main/msp/msp_box.c           - Register box mode
src/main/pg/pg_ids.h             - Add PG_FF_AUTOTUNE_CONFIG
src/main/flight/pid.c            - Call ffAutotuneUpdate(), switch D-term to error-based when active
src/main/cli/settings.c          - CLI parameters
src/main/build/debug.h           - Add DEBUG_FF_AUTOTUNE
```

### D-Term Modification in pid.c

When `ffAutotuneIsActive()` returns true, the D-term calculation changes:

```c
// Normal BF behavior (D on gyro):
float delta = -(gyroRateDterm[axis] - previousGyroRateDterm[axis]);

// With FF Autotune active (D on error):
float error = currentPidSetpoint - gyroRateDterm[axis];
float previousError = previousPidSetpoint[axis] - previousGyroRateDterm[axis];
float delta = error - previousError;
```

This requires storing `previousPidSetpoint[axis]` in addition to `previousGyroRateDterm[axis]`.

### Call Flow

```
pidController() [each loop iteration]
    └── ffAutotuneUpdate(axis, setpoint, gyroRate, setpointDelta, currentTimeUs)
            ├── Check mode activation/deactivation
            │       └── On deactivation: save gains if modified
            ├── If active and in tracking window:
            │       └── Accumulate tracking error
            └── If maneuver just ended:
                    └── Calculate avg error, apply adjustment
```

---

## Pilot Usage Guide

### Setup

1. Enable feature: `set ff_autotune_enabled = ON`
2. Set initial FF to 0: `set feedforward_roll = 0`, `set feedforward_pitch = 0`
3. Assign AUX switch to "FF AUTOTUNE" mode in Configurator
4. Save and reboot

### Flight Procedure

1. Take off and establish stable hover
2. Flip FF Autotune switch ON
3. Perform aggressive maneuvers:
   - Fast rolls and flips
   - Quick pitch pumps
   - Vary throttle during maneuvers
4. Continue for 30-60 seconds of aggressive flying
5. Flip FF Autotune switch OFF (gains save automatically)
6. Land and check learned values in CLI

### Verifying Results

```
# Check learned gains
get ff_autotune_gain_roll
get ff_autotune_gain_pitch

# Apply to main PID if satisfied
set feedforward_roll = <learned_value>
set feedforward_pitch = <learned_value>
save
```

---

## Success Criteria

1. **Fast Convergence**: Should establish bracket within 4-6 maneuvers, converge within 8-12 total
2. **Stability**: Learned gains should not cause oscillation
3. **Tracking**: With learned gains, tracking error should be within deadband during maneuvers
4. **Repeatability**: Multiple flights should converge to similar values (±5 gain units)
5. **Debuggability**: Blackbox should clearly show window detection, assessment, and bracketing progress

---

## Future Enhancements (Out of Scope for v3)

- Automatic transfer of learned gains to feedforward_roll/pitch
- Adaptive step size (larger when far from optimal, smaller when close)
- Integration with existing PID autotuning
- Yaw axis support

---

## Revision History

| Date | Version | Author | Changes |
|------|---------|--------|---------|
| 2026-02-05 | 3.0 | PMantix | Initial PRD for tracking-based FF autotune |
