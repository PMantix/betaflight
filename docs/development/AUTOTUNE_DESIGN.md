# Betaflight In-Flight Autotuning Design Document

## Executive Summary

This document describes the design of a comprehensive in-flight autotuning system for Betaflight.
The system goes beyond traditional PID tuning to automatically calibrate all flight-critical
parameters including filtering, feedforward, and advanced control features.

### Quick Reference: Tunable Parameters Summary

| Category | Parameters | Tune Time | Difficulty |
|----------|------------|-----------|------------|
| **Core PID** | P, I, D per axis (9 params) | ~60s | Low |
| **Feedforward** | F, boost, smooth, jitter (8 params) | ~45s | Medium |
| **D-term Filters** | LPF1, LPF2, dynamic range (9 params) | ~30s | Medium |
| **Gyro Filters** | LPF1, LPF2, notches (10 params) | ~30s | Medium |
| **RPM Filter** | harmonics, Q, LPF, weights (7 params) | ~20s | High |
| **Dynamic Notch** | count, Q, min/max hz (4 params) | ~20s | High |
| **RC Smoothing** | cutoffs, auto factors (5 params) | ~10s | Low |
| **I-term Controls** | relax, windup, rotation (6 params) | ~30s | Medium |
| **TPA** | mode, rate, breakpoints (5 params) | ~30s | Medium |
| **Anti-Gravity** | gain, cutoff, P gain (3 params) | ~15s | Low |
| **Dynamic Idle** | min RPM, PID gains (5 params) | ~30s | Medium |
| **D_max** | per-axis D_max, gain, advance (5 params) | ~30s | Medium |
| **Motor Characteristics** | KV, poles, idle, thrust lin (4 params) | ~45s | High |
| **TOTAL** | **~80 parameters** | **~6-10 min** | - |

## Background

### Current State
Betaflight currently has:
- **CHIRP mode**: Injects a frequency sweep signal for offline system identification
  - Located in `src/main/common/chirp.c`
  - Logs data to blackbox for post-flight analysis
  - Does NOT automatically adjust PIDs

### Problem Statement
- Manual PID tuning requires expertise and many test flights
- Build-specific issues (motor timing, prop resonance, frame flex) are hard to diagnose
- Small drones are particularly challenging due to different dynamics
- Existing "simplified tuning" helps but doesn't adapt to actual flight characteristics

### Goals
1. One-switch autotuning that works in-flight
2. Automatic optimization of P, I, D, and feedforward gains
3. Per-axis tuning capability
4. Safe operation with pilot override
5. Support for different frame sizes and motor characteristics
6. **Multivariable optimization**: Tune PIDs AND filtering together for optimal performance

### The Fundamental Tradeoff: The Three-Way Problem

**The Core Challenge:**
Multirotor tuning is not a simple 1D optimization - it's a complex multivariable problem with interdependent parameters:

```
┌─────────────────────────────────────────────────────────────┐
│                    THE TUNING TRIANGLE                       │
│                                                              │
│                    RESPONSIVENESS                            │
│                          ▲                                   │
│                         / \                                  │
│                        /   \                                 │
│                       /     \                                │
│                      /       \                               │
│                     /         \                              │
│                    /           \                             │
│                   /             \                            │
│                  /               \                           │
│                 /                 \                          │
│                /                   \                         │
│               /                     \                        │
│              /        SWEET          \                       │
│             /         SPOT            \                      │
│            /            ◆              \                     │
│           /                             \                    │
│          /                               \                   │
│         /                                 \                  │
│        /                                   \                 │
│       /                                     \                │
│      /                                       \               │
│     /__________________________________________\              │
│   EFFICIENCY/                                  DAMPING/      │
│   LOW NOISE                                    STABILITY     │
│                                                              │
└─────────────────────────────────────────────────────────────┘

You can't maximize all three simultaneously - it's always a tradeoff:

1. More D gain → Better damping → BUT amplifies gyro noise → hot motors, poor efficiency
2. More gyro filtering → Cleaner signal → Can use more D → BUT phase lag → sluggish response
3. Less filtering + More P → Responsive feel → BUT limited D (noise) → more overshoot

The pilot's dilemma:
- Racing pilots: Prioritize responsiveness, tolerate noise/heat
- Freestyle pilots: Balance all three
- Cinematic pilots: Prioritize smoothness, accept slower response
- Long-range pilots: Prioritize efficiency, minimize heat
```

**What Makes This Hard:**
- PIDs and filtering are **intimately coupled** - you can't tune one without affecting the other
- Different hardware has different noise floors (gyro quality, frame stiffness, motor/bearing quality)
- Props create resonances at specific frequencies (need notch filters)
- The optimal point changes with battery voltage, temperature, prop wear
- Manual tuning takes 10-50+ flights to find the sweet spot

**Autotune's Value Proposition:**
If autotune can optimize **BOTH** PID gains **AND** filtering parameters together, it solves the problem that burns thousands of hours of pilot time worldwide. This is the real benefit - not just getting PIDs close, but finding the global optimum in the multidimensional parameter space.

---

## System Architecture

### High-Level Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                         AUTOTUNE STATE MACHINE                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐       │
│  │  IDLE    │───>│  SETUP   │───>│  EXCITE  │───>│ ANALYZE  │       │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘       │
│       ^                                               │              │
│       │         ┌──────────┐    ┌──────────┐         │              │
│       └─────────│ COMPLETE │<───│  ADJUST  │<────────┘              │
│                 └──────────┘    └──────────┘                        │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘
```

### State Descriptions

1. **IDLE**: Autotune inactive, normal flight
2. **SETUP**: Stabilize hover, check safety conditions
3. **EXCITE**: Inject test signals (step, doublet, or chirp)
4. **ANALYZE**: Process response data, calculate metrics
5. **ADJUST**: Modify PID gains based on analysis
6. **COMPLETE**: Save results, return to normal flight

---

## Excitation Methods

### Method 1: Step Response Analysis (Primary)

Inject a step setpoint change and measure:
- **Rise time** (10% to 90% of target)
- **Overshoot** (peak beyond target)
- **Settling time** (time to stay within 5% of target)
- **Oscillation frequency** (if any)

```c
typedef struct {
    float riseTime;          // seconds
    float overshootPercent;  // 0-100%
    float settlingTime;      // seconds
    float oscillationFreq;   // Hz, 0 if none
    float steadyStateError;  // degrees/sec
} stepResponseMetrics_t;
```

### Method 2: Doublet Injection (IMPLEMENTED ✅)

**Implementation Status:** Complete in Milestone 2

Two opposing pulses to measure symmetric response with zero net displacement:
```
Bidirectional Doublet Pattern: +A, -2A, +2A, -2A, +2A, -A

Phase 1: +A     ────┐
Phase 2: -2A        │        ┌────
Phase 3: +2A        │    ┌───┘
Phase 4: -2A        │    │   ┌────
Phase 5: +2A        │    │   │
Phase 6: -A         └────┴───┴────

Net displacement: A - 2A + 2A - 2A + 2A - A = 0
Duration: 6 × step_duration_ms (default 600ms)
```

**Advantages:**
- ✅ Returns to original attitude naturally (zero net displacement)
- ✅ Tests both directions symmetrically (detects asymmetries)
- ✅ Safer for in-flight use (no accumulated angle error)
- ✅ More data from single maneuver (multiple transitions)
- ✅ Strong, visible aircraft response for validation

**Design Evolution:**
1. Simple doublet: +A, -A (doesn't guarantee return to origin)
2. 3-2-1-1 doublet: +A, -2A, +A (single direction dominant)
3. Bidirectional weak: +A, -2A, +A, -A, +2A, -A (weak middle)
4. **Final bidirectional**: +A, -2A, +2A, -2A, +2A, -A ✅ (strong symmetric)

**Configuration:**
- `autotune_step_amplitude`: Excitation magnitude (default 100 deg/s)
- `autotune_step_duration_ms`: Duration per phase (default 100ms)
- Total test time: 6× duration + 3× duration settling = 900ms

**Sample Collection:**
- 200Hz sample rate (PID frequency independent)
- ~120 samples during excitation
- ~60 samples during settling observation
- ~181 total samples per axis test

### Method 3: Relay Feedback (Åström-Hägglund Method)

Classic autotuning approach:
1. Apply relay control (bang-bang) around setpoint
2. System oscillates at ultimate frequency
3. Measure amplitude and period
4. Calculate Ku (ultimate gain) and Tu (ultimate period)
5. Apply Ziegler-Nichols or similar tuning rules

### Method 4: Frequency Sweep (Enhanced CHIRP)

Extend existing CHIRP to:
1. Inject chirp signal at controlled amplitude
2. Measure gain and phase at multiple frequencies
3. Identify system transfer function
4. Calculate optimal PID from frequency response

---

## Implementation Details

### New Files Required

```
src/main/flight/autotune.c      - Main autotune logic
src/main/flight/autotune.h      - Public interface
src/main/pg/autotune.c          - Parameter group
src/main/pg/autotune.h          - Configuration structure
```

### Configuration Structure

```c
typedef struct autotuneConfig_s {
    uint8_t  autotune_mode;           // 0=off, 1=step, 2=doublet, 3=relay, 4=chirp
    uint8_t  autotune_axes;           // Bitmask: bit0=roll, bit1=pitch, bit2=yaw
    uint8_t  step_amplitude;          // Step size in degrees/sec (10-200)
    uint16_t step_duration_ms;        // Duration of step (50-500ms)
    uint8_t  target_overshoot;        // Desired overshoot % (0-30)
    uint8_t  target_settling_loops;   // Desired settling in PID loops
    uint8_t  max_iterations;          // Max tune iterations per axis (1-20)
    uint8_t  safety_margin;           // Attitude limit during tune (degrees)
    uint8_t  tune_d_first;            // 1=tune D before P, 0=tune P first
    uint8_t  save_on_complete;        // 1=auto-save, 0=require manual save
} autotuneConfig_t;

PG_DECLARE(autotuneConfig_t, autotuneConfig);
```

### Runtime State

```c
typedef enum {
    AUTOTUNE_STATE_IDLE,
    AUTOTUNE_STATE_SETUP,
    AUTOTUNE_STATE_WAIT_STABLE,
    AUTOTUNE_STATE_EXCITE,
    AUTOTUNE_STATE_MEASURE,
    AUTOTUNE_STATE_ANALYZE,
    AUTOTUNE_STATE_ADJUST,
    AUTOTUNE_STATE_ITERATE,
    AUTOTUNE_STATE_COMPLETE,
    AUTOTUNE_STATE_FAILED,
    AUTOTUNE_STATE_ABORTED
} autotuneState_e;

typedef struct autotuneRuntime_s {
    autotuneState_e state;
    uint8_t currentAxis;
    uint8_t iteration;
    timeUs_t stateEnteredAt;
    timeUs_t excitationStartTime;
    
    // Excitation parameters
    float excitationAmplitude;
    float excitationDuration;
    
    // Measurement buffers (circular)
    float gyroHistory[AUTOTUNE_SAMPLE_COUNT];
    float setpointHistory[AUTOTUNE_SAMPLE_COUNT];
    uint16_t sampleIndex;
    
    // Calculated metrics
    stepResponseMetrics_t metrics;
    
    // PID state during tuning
    float originalP;
    float originalI;
    float originalD;
    float originalF;
    float testP;
    float testI;
    float testD;
    float testF;
    
    // Results
    float bestP;
    float bestI;
    float bestD;
    float bestF;
    float bestScore;
    
} autotuneRuntime_t;
```

### Core Algorithm: Step Response Tuning

```c
void autotuneAnalyzeStepResponse(autotuneRuntime_t *at)
{
    const int axis = at->currentAxis;
    
    // 1. Find step start index in history
    int stepStartIdx = findStepStart(at);
    
    // 2. Calculate rise time (10% to 90%)
    float target = at->excitationAmplitude;
    int idx10 = findCrossingIndex(at, stepStartIdx, target * 0.1f);
    int idx90 = findCrossingIndex(at, stepStartIdx, target * 0.9f);
    at->metrics.riseTime = (idx90 - idx10) * pidRuntime.dT;
    
    // 3. Find peak and calculate overshoot
    float peak = findPeakValue(at, stepStartIdx);
    at->metrics.overshootPercent = 100.0f * (peak - target) / target;
    
    // 4. Calculate settling time (within 5% of target)
    int settledIdx = findSettlingIndex(at, stepStartIdx, target, 0.05f);
    at->metrics.settlingTime = (settledIdx - stepStartIdx) * pidRuntime.dT;
    
    // 5. Check for oscillation
    at->metrics.oscillationFreq = detectOscillationFrequency(at, settledIdx);
    
    // 6. Calculate score
    at->metrics.score = calculateTuneScore(&at->metrics, autotuneConfig());
}

float calculateTuneScore(stepResponseMetrics_t *m, const autotuneConfig_t *config)
{
    // Lower is better
    float score = 0.0f;
    
    // Penalize overshoot beyond target
    float overshootError = fabsf(m->overshootPercent - config->target_overshoot);
    score += overshootError * 2.0f;
    
    // Penalize slow rise time
    score += m->riseTime * 100.0f;
    
    // Penalize long settling
    score += m->settlingTime * 50.0f;
    
    // Heavily penalize oscillation
    if (m->oscillationFreq > 0) {
        score += 100.0f;
    }
    
    return score;
}
```

### PID Adjustment Algorithm

```c
void autotuneAdjustPID(autotuneRuntime_t *at)
{
    const stepResponseMetrics_t *m = &at->metrics;
    
    // Use gradient-based adjustment
    float pAdjust = 1.0f;
    float dAdjust = 1.0f;
    float iAdjust = 1.0f;
    
    // Too much overshoot -> reduce P, increase D
    if (m->overshootPercent > autotuneConfig()->target_overshoot + 5.0f) {
        pAdjust = 0.9f;
        dAdjust = 1.1f;
    }
    // Too little response -> increase P
    else if (m->overshootPercent < autotuneConfig()->target_overshoot - 5.0f) {
        pAdjust = 1.1f;
    }
    
    // Oscillation detected -> reduce P and D, or increase D depending on frequency
    if (m->oscillationFreq > 0) {
        if (m->oscillationFreq > 50.0f) {
            // High frequency oscillation - D too high or filter issue
            dAdjust = 0.8f;
        } else {
            // Low frequency oscillation - P too high
            pAdjust = 0.85f;
        }
    }
    
    // Slow settling -> adjust I
    if (m->settlingTime > 0.3f && m->steadyStateError > 5.0f) {
        iAdjust = 1.1f;
    }
    
    // Apply adjustments
    at->testP = constrainf(at->testP * pAdjust, 10, 250);
    at->testD = constrainf(at->testD * dAdjust, 0, 100);
    at->testI = constrainf(at->testI * iAdjust, 10, 250);
    
    // Apply to live PIDs
    pidProfile_t *profile = currentPidProfile;
    profile->pid[at->currentAxis].P = lrintf(at->testP);
    profile->pid[at->currentAxis].D = lrintf(at->testD);
    profile->pid[at->currentAxis].I = lrintf(at->testI);
    
    // Reinitialize PID with new values
    pidInitConfig(profile);
}
```

### Safety Features

```c
bool autotuneSafetyCheck(void)
{
    // 1. Check attitude limits
    if (fabsf(attitude.values.roll) > autotuneConfig()->safety_margin * 10 ||
        fabsf(attitude.values.pitch) > autotuneConfig()->safety_margin * 10) {
        return false;
    }
    
    // 2. Check gyro rates not excessive
    for (int axis = 0; axis < 3; axis++) {
        if (fabsf(gyro.gyroADCf[axis]) > AUTOTUNE_MAX_RATE) {
            return false;
        }
    }
    
    // 3. Check throttle is in reasonable range (hover)
    float throttle = mixerGetThrottle();
    if (throttle < 0.2f || throttle > 0.8f) {
        return false;
    }
    
    // 4. Check no stick input (pilot not commanding)
    if (fabsf(getRcDeflection(FD_ROLL)) > 0.1f ||
        fabsf(getRcDeflection(FD_PITCH)) > 0.1f ||
        fabsf(getRcDeflection(FD_YAW)) > 0.1f) {
        return false;
    }
    
    return true;
}

void autotuneAbortIfUnsafe(autotuneRuntime_t *at)
{
    if (!autotuneSafetyCheck()) {
        // Restore original PIDs
        pidProfile_t *profile = currentPidProfile;
        profile->pid[at->currentAxis].P = lrintf(at->originalP);
        profile->pid[at->currentAxis].D = lrintf(at->originalD);
        profile->pid[at->currentAxis].I = lrintf(at->originalI);
        pidInitConfig(profile);
        
        at->state = AUTOTUNE_STATE_ABORTED;
        beeper(BEEPER_AUTOTUNE_FAIL);
    }
}
```

### Integration with Existing Systems

#### Box Mode Addition
```c
// In src/main/fc/rc_modes.h
typedef enum {
    // ... existing modes ...
    BOXAUTOTUNE,
} boxId_e;

// In src/main/msp/msp_box.c
{ .boxId = BOXAUTOTUNE, .boxName = "AUTOTUNE", .permanentId = 56 }
```

#### Runtime Config Flag
```c
// In src/main/fc/runtime_config.h
typedef enum {
    // ... existing flags ...
    AUTOTUNE_MODE  = (1 << 8),
} flightModeFlags_e;
```

#### PID Loop Integration
```c
// In pidController() - src/main/flight/pid.c
#ifdef USE_AUTOTUNE
    if (FLIGHT_MODE(AUTOTUNE_MODE)) {
        autotuneUpdate(currentTimeUs);
        // Autotune may modify setpoint for excitation
        currentPidSetpoint = autotuneModifySetpoint(axis, currentPidSetpoint);
    }
#endif
```

---

## User Interface

### Activation
1. User hovers drone in stable position
2. User flips AUTOTUNE switch
3. OSD shows "AUTOTUNE ROLL" (or current axis)
4. Drone performs test maneuvers automatically
5. OSD shows progress: "TUNE 3/10 P:45 D:32"
6. When complete: "AUTOTUNE DONE - SAVE?"
7. User flips switch off
8. If save_on_complete=1, saves automatically
9. If not, user must `save` in CLI

### OSD Elements
```c
// New OSD element: OSD_AUTOTUNE_STATUS
void osdElementAutotuneStatus(osdElementParms_t *element)
{
    if (FLIGHT_MODE(AUTOTUNE_MODE)) {
        autotuneRuntime_t *at = getAutotuneRuntime();
        switch (at->state) {
            case AUTOTUNE_STATE_SETUP:
                tfp_sprintf(element->buff, "TUNE SETUP");
                break;
            case AUTOTUNE_STATE_EXCITE:
                tfp_sprintf(element->buff, "%s %d/%d", 
                    axisNames[at->currentAxis],
                    at->iteration,
                    autotuneConfig()->max_iterations);
                break;
            case AUTOTUNE_STATE_COMPLETE:
                tfp_sprintf(element->buff, "TUNE DONE");
                break;
            // ... etc
        }
    }
}
```

### Beeper Feedback
- **Autotune start**: Short rising tone
- **Axis complete**: Double beep
- **All complete**: Success melody
- **Aborted/Failed**: Warning beeps

---

## Testing Strategy

### Simulation Testing
1. Use SITL (Software In The Loop) with motor models
2. Inject known system dynamics
3. Verify autotune converges to expected PIDs

### Hardware Testing Progression
1. **Bench test**: Props off, verify state machine and safety
2. **Tethered test**: Constrained flight with safety line
3. **Open test (small)**: 65mm whoop in safe area
4. **Open test (standard)**: 5" quad with experienced pilot

### Metrics for Success
- Autotune should complete in < 60 seconds per axis
- Resulting tune should have:
  - Overshoot within 5% of target
  - No sustained oscillation
  - Settling time < 200ms
- Pilot should rate tune as "acceptable" or better

---

## CLI Commands

```
# Autotune configuration
set autotune_mode = STEP          # STEP, DOUBLET, RELAY, CHIRP
set autotune_axes = RP            # R, P, Y, RP, RPY
set autotune_step_amplitude = 100 # deg/sec
set autotune_step_duration = 100  # ms
set autotune_target_overshoot = 10
set autotune_max_iterations = 10
set autotune_safety_margin = 30   # degrees
set autotune_save_on_complete = ON

# View autotune results (after tuning)
autotune status

# Clear autotune results
autotune reset

# Apply last autotune results without flying
autotune apply
```

---

## Expanded Autotuning Scope

This section defines the complete set of parameters that can be automatically tuned,
organized by tuning phase. The goal is to move beyond just PID tuning to calibrate
all the "hidden settings" that significantly affect flight performance.

### Complete Tunable Parameter Inventory

#### Category 1: Core PID Gains (Phase 1 - Basic Autotune)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `pid[ROLL].P` | pidProfile_t | Step response | 46 |
| `pid[ROLL].I` | pidProfile_t | Steady-state error | 90 |
| `pid[ROLL].D` | pidProfile_t | Oscillation damping | 42 |
| `pid[PITCH].P` | pidProfile_t | Step response | 50 |
| `pid[PITCH].I` | pidProfile_t | Steady-state error | 95 |
| `pid[PITCH].D` | pidProfile_t | Oscillation damping | 46 |
| `pid[YAW].P` | pidProfile_t | Step response | 45 |
| `pid[YAW].I` | pidProfile_t | Steady-state error | 90 |
| `pid[YAW].D` | pidProfile_t | Oscillation damping | 0 |

**Tuning Approach**: Step/doublet injection, measure rise time, overshoot, settling

---

#### Category 2: Feedforward System (Phase 2)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `pid[axis].F` | pidProfile_t | Setpoint tracking lag | 115/120/60 |
| `feedforward_boost` | pidProfile_t | Acceleration response | 15 |
| `feedforward_smooth_factor` | pidProfile_t | Response smoothness | 25 |
| `feedforward_jitter_factor` | pidProfile_t | Noise rejection | 7 |
| `feedforward_transition` | pidProfile_t | Center stick blending | 0 |
| `feedforward_averaging` | pidProfile_t | RC noise smoothing | 0 |
| `feedforward_max_rate_limit` | pidProfile_t | Rate saturation protection | 90 |
| `feedforward_yaw_hold_gain` | pidProfile_t | Yaw stability | 0 |
| `feedforward_yaw_hold_time` | pidProfile_t | Yaw stability decay | 25 |

**Tuning Approach**:
1. Inject fast step RC commands
2. Measure motor response lag vs setpoint
3. Optimize F gain for minimum delay while avoiding overshoot
4. Calibrate boost by measuring acceleration response
5. Measure jitter at hover to set optimal jitter_factor

```c
typedef struct feedforwardTuneResult_s {
    float optimalF[3];           // Optimal F gain per axis
    float optimalBoost;          // Optimal acceleration boost
    float motorLagMs;            // Measured motor response lag
    float rcNoiseBandwidth;      // Measured RC noise bandwidth
    uint8_t suggestedAveraging;  // Recommended averaging mode
} feedforwardTuneResult_t;
```

---

#### Category 3: D-Term Filtering (Phase 3)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `dterm_lpf1_type` | pidProfile_t | Noise analysis | PT1 |
| `dterm_lpf1_static_hz` | pidProfile_t | Noise floor | 75 |
| `dterm_lpf1_dyn_min_hz` | pidProfile_t | Throttle sweep | 75 |
| `dterm_lpf1_dyn_max_hz` | pidProfile_t | Throttle sweep | 150 |
| `dterm_lpf1_dyn_expo` | pidProfile_t | Throttle response | 5 |
| `dterm_lpf2_type` | pidProfile_t | Noise analysis | PT1 |
| `dterm_lpf2_static_hz` | pidProfile_t | High-freq noise | 150 |
| `dterm_notch_hz` | pidProfile_t | FFT analysis | 0 |
| `dterm_notch_cutoff` | pidProfile_t | FFT analysis | 0 |

**Tuning Approach**:
1. Record D-term at various throttle levels
2. Perform FFT to identify noise floor
3. Set LPF cutoffs just above motor noise
4. Maximize cutoff while keeping D-term noise below threshold
5. If specific resonance found, configure notch filter

```c
typedef struct dtermFilterTuneResult_s {
    float noiseFloorHz;          // Frequency where noise starts
    float noiseFloorLevel;       // Noise amplitude at floor
    float recommendedLpf1;       // Optimal LPF1 cutoff
    float recommendedLpf2;       // Optimal LPF2 cutoff
    float resonanceHz;           // If frame resonance detected
    float delayMs;               // Total filter delay
} dtermFilterTuneResult_t;
```

---

#### Category 4: Gyro Filtering (Phase 4)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `gyro_lpf1_type` | gyroConfig_t | Noise analysis | PT1 |
| `gyro_lpf1_static_hz` | gyroConfig_t | Noise floor | 250 |
| `gyro_lpf1_dyn_min_hz` | gyroConfig_t | Throttle sweep | 250 |
| `gyro_lpf1_dyn_max_hz` | gyroConfig_t | Throttle sweep | 500 |
| `gyro_lpf1_dyn_expo` | gyroConfig_t | Throttle response | 5 |
| `gyro_lpf2_type` | gyroConfig_t | Noise analysis | PT1 |
| `gyro_lpf2_static_hz` | gyroConfig_t | High-freq noise | 500 |
| `gyro_soft_notch_hz_1` | gyroConfig_t | FFT analysis | 0 |
| `gyro_soft_notch_cutoff_1` | gyroConfig_t | FFT analysis | 0 |
| `gyro_soft_notch_hz_2` | gyroConfig_t | FFT analysis | 0 |
| `gyro_soft_notch_cutoff_2` | gyroConfig_t | FFT analysis | 0 |

**Tuning Approach**:
1. Record gyro at various throttle levels
2. FFT analysis to identify noise spectrum
3. Find optimal LPF cutoffs balancing latency vs noise
4. Detect frame resonances for notch configuration

---

#### Category 5: RPM Filter (Phase 5)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `rpm_filter_harmonics` | rpmFilterConfig_t | Motor noise FFT | 3 |
| `rpm_filter_min_hz` | rpmFilterConfig_t | Idle RPM analysis | 100 |
| `rpm_filter_fade_range_hz` | rpmFilterConfig_t | Low-throttle analysis | 50 |
| `rpm_filter_q` | rpmFilterConfig_t | Notch effectiveness | 500 |
| `rpm_filter_lpf_hz` | rpmFilterConfig_t | Motor KV/responsiveness | 150 |
| `rpm_filter_weights[0-2]` | rpmFilterConfig_t | Per-harmonic noise | 100 |

**Tuning Approach**:
1. Measure motor idle RPM → calculate min_hz
2. Analyze motor KV and responsiveness → set lpf_hz
3. FFT motor noise at various RPMs
4. Identify which harmonics need filtering
5. Adjust weights based on harmonic strength

```c
typedef struct rpmFilterTuneResult_s {
    uint16_t measuredIdleRpm;    // Actual idle motor RPM
    uint16_t estimatedKv;        // Calculated motor KV
    uint8_t activeHarmonics;     // Which harmonics have noise
    uint8_t harmonicWeights[3];  // Optimal weights per harmonic
    uint16_t optimalLpfHz;       // Optimal motor RPM LPF
    uint16_t optimalMinHz;       // Optimal minimum frequency
} rpmFilterTuneResult_t;
```

---

#### Category 6: Dynamic Notch (Phase 6)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `dyn_notch_count` | dynNotchConfig_t | FFT analysis | 3 |
| `dyn_notch_q` | dynNotchConfig_t | Notch width tuning | 300 |
| `dyn_notch_min_hz` | dynNotchConfig_t | Low-throttle noise | 100 |
| `dyn_notch_max_hz` | dynNotchConfig_t | High-throttle noise | 600 |

**Tuning Approach**:
1. Sweep throttle, record noise spectrum
2. Identify how many distinct noise peaks exist
3. Measure peak width to set Q
4. Find min/max frequencies of noise peaks

---

#### Category 7: RC Smoothing (Phase 7)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `rc_smoothing` | rxConfig_t | N/A (enable/disable) | 1 |
| `rc_smoothing_setpoint_cutoff` | rxConfig_t | RC rate analysis | 0 (auto) |
| `rc_smoothing_throttle_cutoff` | rxConfig_t | Throttle smoothness | 0 (auto) |
| `rc_smoothing_auto_factor_rpy` | rxConfig_t | Response vs smoothness | 30 |
| `rc_smoothing_auto_factor_throttle` | rxConfig_t | Throttle smoothness | 30 |

**Tuning Approach**:
1. Analyze incoming RC signal rate
2. Measure RC signal jitter/noise
3. Set cutoffs to remove jitter while preserving response
4. Lower auto_factor for more aggressive feel

---

#### Category 8: I-Term Controls (Phase 8)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `iterm_relax` | pidProfile_t | Quick flip analysis | ITERM_RELAX_RP |
| `iterm_relax_type` | pidProfile_t | Bounce-back test | ITERM_RELAX_SETPOINT |
| `iterm_relax_cutoff` | pidProfile_t | Wind-up detection | 15 |
| `itermWindup` | pidProfile_t | Saturation analysis | 100 |
| `itermLimit` | pidProfile_t | Saturation analysis | 400 |
| `iterm_rotation` | pidProfile_t | Coordinated maneuvers | 1 |

**Tuning Approach**:
1. Perform quick flips/rolls
2. Measure I-term wind-up during maneuver
3. Measure bounce-back on stop
4. Adjust relax cutoff to minimize bounce without losing authority

---

#### Category 9: TPA (Throttle PID Attenuation) (Phase 9)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `tpa_mode` | pidProfile_t | High-throttle stability | TPA_MODE_D |
| `tpa_rate` | pidProfile_t | Full-throttle oscillation | 65 |
| `tpa_breakpoint` | pidProfile_t | Oscillation onset | 1350 |
| `tpa_low_rate` | pidProfile_t | Low-throttle handling | 20 |
| `tpa_low_breakpoint` | pidProfile_t | Low-throttle threshold | 1050 |

**Tuning Approach**:
1. Inject test signals at various throttle levels
2. Measure stability margin at each level
3. Find where oscillation starts
4. Set TPA to reduce gains proportionally

---

#### Category 10: Anti-Gravity (Phase 10)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `anti_gravity_gain` | pidProfile_t | Punch-out test | 80 |
| `anti_gravity_cutoff_hz` | pidProfile_t | Throttle transient | 15 |
| `anti_gravity_p_gain` | pidProfile_t | P-term boost during AG | 100 |

**Tuning Approach**:
1. Perform throttle punch-out
2. Measure attitude disturbance
3. Adjust anti-gravity to minimize disturbance

---

#### Category 11: Dynamic Idle / RPM Limiter (Phase 11)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `dyn_idle_min_rpm` | pidProfile_t | Idle stability test | 0 |
| `dyn_idle_p_gain` | pidProfile_t | RPM control stability | 50 |
| `dyn_idle_i_gain` | pidProfile_t | RPM steady-state | 50 |
| `dyn_idle_d_gain` | pidProfile_t | RPM transient | 50 |
| `dyn_idle_max_increase` | pidProfile_t | Max authority | 150 |

**Tuning Approach**:
1. Measure motor idle RPM stability
2. Test desync recovery
3. Optimize PID for quick RPM recovery without overshoot

---

#### Category 12: D_max System (Phase 12)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `d_max[ROLL]` | pidProfile_t | High-rate stability | 42 |
| `d_max[PITCH]` | pidProfile_t | High-rate stability | 46 |
| `d_max[YAW]` | pidProfile_t | High-rate stability | 0 |
| `d_max_gain` | pidProfile_t | D boost response | 37 |
| `d_max_advance` | pidProfile_t | D boost timing | 20 |

**Tuning Approach**:
1. Test at low rates → measure optimal base D
2. Test at high rates → measure optimal D_max
3. Analyze setpoint acceleration to tune gain/advance

---

#### Category 13: Motor Characteristics (Phase 13)

| Parameter | Config Location | Measurement Method | Default |
|-----------|-----------------|-------------------|---------|
| `motor_kv` | motorConfig_t | RPM/voltage analysis | 0 |
| `motorPoleCount` | motorConfig_t | eRPM analysis | 14 |
| `motorIdle` | motorConfig_t | Idle stability | 550 |
| `thrustLinearization` | pidProfile_t | Thrust curve analysis | 0 |

**Tuning Approach**:
1. Measure actual RPM vs commanded throttle
2. Calculate motor KV from voltage and RPM
3. Measure thrust response linearity
4. Calibrate thrust linearization curve

---

### Automated Tuning Phases

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      COMPREHENSIVE AUTOTUNE PHASES                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  PHASE 1: SENSOR CHARACTERIZATION (Ground - motors off)                    │
│  ├─ Gyro noise floor measurement                                           │
│  ├─ Accelerometer calibration verification                                 │
│  └─ RC link rate and jitter analysis                                       │
│                                                                             │
│  PHASE 2: MOTOR CHARACTERIZATION (Ground - props on, tethered)             │
│  ├─ Motor KV estimation                                                    │
│  ├─ Idle RPM measurement                                                   │
│  ├─ RPM filter frequency analysis                                          │
│  └─ Motor noise spectrum at various throttles                              │
│                                                                             │
│  PHASE 3: FILTER OPTIMIZATION (Hover)                                      │
│  ├─ Gyro LPF cutoff optimization                                           │
│  ├─ D-term LPF cutoff optimization                                         │
│  ├─ Dynamic notch min/max calibration                                      │
│  └─ RPM filter weight optimization                                         │
│                                                                             │
│  PHASE 4: PID TUNING (Hover)                                               │
│  ├─ P gain optimization (step response)                                    │
│  ├─ D gain optimization (oscillation damping)                              │
│  ├─ I gain optimization (steady-state error)                               │
│  └─ Per-axis iteration until convergence                                   │
│                                                                             │
│  PHASE 5: FEEDFORWARD CALIBRATION (Aggressive maneuvers)                   │
│  ├─ F gain optimization (setpoint tracking)                                │
│  ├─ FF boost optimization (acceleration response)                          │
│  ├─ FF smooth_factor calibration                                           │
│  └─ Yaw hold calibration                                                   │
│                                                                             │
│  PHASE 6: THROTTLE RESPONSE (Throttle sweeps)                              │
│  ├─ TPA curve optimization                                                 │
│  ├─ Anti-gravity calibration                                               │
│  ├─ Throttle boost optimization                                            │
│  └─ Dynamic filter range verification                                      │
│                                                                             │
│  PHASE 7: EDGE CASE TUNING (Specific maneuvers)                            │
│  ├─ I-term relax calibration (quick flips)                                 │
│  ├─ D_max calibration (high-rate maneuvers)                                │
│  ├─ Dynamic idle verification (throttle chops)                             │
│  └─ Crash recovery verification                                            │
│                                                                             │
│  PHASE 8: VALIDATION (Free flight)                                         │
│  ├─ Full flight test with tuned parameters                                 │
│  ├─ Stability verification at all throttles                                │
│  ├─ Response quality assessment                                            │
│  └─ Final parameter save                                                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Measurement Techniques

### Real-Time FFT Analysis

For filter tuning, we need spectral analysis of gyro and D-term signals. This can be
implemented efficiently using a sliding window FFT:

```c
#define FFT_WINDOW_SIZE 256
#define FFT_SAMPLE_RATE (1000000 / gyro.targetLooptime)  // Based on PID loop rate

typedef struct fftAnalysis_s {
    float samples[FFT_WINDOW_SIZE];
    uint16_t sampleIndex;
    float magnitudes[FFT_WINDOW_SIZE / 2];
    float peakFrequencies[8];        // Top 8 peaks
    float peakMagnitudes[8];
    float noiseFloorHz;              // Where noise starts dominating
    float totalNoisePower;           // RMS of high-frequency content
} fftAnalysis_t;

void fftAnalysisUpdate(fftAnalysis_t *fft, float sample)
{
    fft->samples[fft->sampleIndex++] = sample;
    if (fft->sampleIndex >= FFT_WINDOW_SIZE) {
        fft->sampleIndex = 0;
        
        // Apply Hanning window and compute FFT
        applyHanningWindow(fft->samples, FFT_WINDOW_SIZE);
        computeRealFFT(fft->samples, fft->magnitudes, FFT_WINDOW_SIZE);
        
        // Find peaks and noise floor
        findSpectralPeaks(fft);
        calculateNoiseFloor(fft);
    }
}

float getOptimalLpfCutoff(fftAnalysis_t *fft, float targetNoiseReduction)
{
    // Find frequency where we achieve target noise reduction
    float totalPower = 0;
    for (int i = 0; i < FFT_WINDOW_SIZE / 2; i++) {
        totalPower += fft->magnitudes[i] * fft->magnitudes[i];
    }
    
    float accumulatedPower = 0;
    for (int i = FFT_WINDOW_SIZE / 2 - 1; i >= 0; i--) {
        accumulatedPower += fft->magnitudes[i] * fft->magnitudes[i];
        float reductionPercent = 100.0f * accumulatedPower / totalPower;
        if (reductionPercent >= targetNoiseReduction) {
            return (float)i * FFT_SAMPLE_RATE / FFT_WINDOW_SIZE;
        }
    }
    return FFT_SAMPLE_RATE / 2;  // Nyquist
}
```

### Step Response Measurement

Core technique for PID tuning - inject a setpoint step and measure response:

```c
typedef struct stepMeasurement_s {
    float setpointHistory[STEP_SAMPLE_COUNT];
    float gyroHistory[STEP_SAMPLE_COUNT];
    float pidOutputHistory[STEP_SAMPLE_COUNT];
    timeUs_t timestamps[STEP_SAMPLE_COUNT];
    uint16_t sampleCount;
    
    // Calculated metrics
    float riseTime10to90;            // Time from 10% to 90% of target
    float overshootPercent;          // Peak beyond target
    float settlingTime5Percent;      // Time to stay within 5%
    float steadyStateError;          // Final error
    float oscillationFrequency;      // If oscillating
    float dampingRatio;              // Estimated damping
} stepMeasurement_t;

void analyzeStepResponse(stepMeasurement_t *m, float targetRate)
{
    // Find when we cross 10% and 90% of target
    int idx10 = -1, idx90 = -1, idxPeak = 0;
    float peakValue = 0;
    
    for (int i = 0; i < m->sampleCount; i++) {
        float normalized = m->gyroHistory[i] / targetRate;
        
        if (idx10 < 0 && normalized >= 0.1f) idx10 = i;
        if (idx90 < 0 && normalized >= 0.9f) idx90 = i;
        if (m->gyroHistory[i] > peakValue) {
            peakValue = m->gyroHistory[i];
            idxPeak = i;
        }
    }
    
    // Calculate metrics
    m->riseTime10to90 = (m->timestamps[idx90] - m->timestamps[idx10]) * 1e-6f;
    m->overshootPercent = 100.0f * (peakValue - targetRate) / targetRate;
    
    // Find settling time (last time we exit ±5% band)
    float band = 0.05f * targetRate;
    for (int i = m->sampleCount - 1; i >= 0; i--) {
        if (fabsf(m->gyroHistory[i] - targetRate) > band) {
            m->settlingTime5Percent = (m->timestamps[i] - m->timestamps[idx10]) * 1e-6f;
            break;
        }
    }
    
    // Detect oscillation by counting zero crossings after peak
    int zeroCrossings = 0;
    float lastError = m->gyroHistory[idxPeak] - targetRate;
    for (int i = idxPeak + 1; i < m->sampleCount; i++) {
        float error = m->gyroHistory[i] - targetRate;
        if (error * lastError < 0) {
            zeroCrossings++;
        }
        lastError = error;
    }
    
    if (zeroCrossings >= 2) {
        // Estimate oscillation frequency from zero crossing rate
        float period = 2.0f * (m->timestamps[m->sampleCount-1] - m->timestamps[idxPeak]) 
                       * 1e-6f / zeroCrossings;
        m->oscillationFrequency = 1.0f / period;
    } else {
        m->oscillationFrequency = 0;
    }
}
```

### Motor/Prop Noise Characterization

For RPM filter and dynamic notch tuning:

```c
typedef struct motorNoiseProfile_s {
    // Per-motor data
    float motorRpm[4];
    float motorNoiseHz[4][RPM_FILTER_HARMONICS_MAX];
    float harmonicStrength[4][RPM_FILTER_HARMONICS_MAX];
    
    // Aggregate analysis
    float dominantNoiseHz;           // Strongest noise frequency
    uint8_t dominantHarmonic;        // Which harmonic dominates
    float noiseVsThrottle[11];       // Noise at 0%, 10%, ..., 100% throttle
    float frequencyVsThrottle[11];   // Dominant freq at each throttle
} motorNoiseProfile_t;

void characterizeMotorNoise(motorNoiseProfile_t *profile, float throttle)
{
    // Sample gyro at current throttle for ~0.5 seconds
    fftAnalysis_t fft;
    initFftAnalysis(&fft);
    
    for (int sample = 0; sample < FFT_WINDOW_SIZE * 2; sample++) {
        fftAnalysisUpdate(&fft, gyro.gyroADCf[FD_ROLL]);
        delayMicroseconds(gyro.targetLooptime);
    }
    
    // Find motor frequency from dshot telemetry
    for (int motor = 0; motor < 4; motor++) {
        profile->motorRpm[motor] = getDshotRpm(motor);
        float fundamentalHz = profile->motorRpm[motor] / 60.0f;
        
        // Check strength of each harmonic
        for (int h = 0; h < RPM_FILTER_HARMONICS_MAX; h++) {
            float harmonicHz = fundamentalHz * (h + 1);
            profile->motorNoiseHz[motor][h] = harmonicHz;
            profile->harmonicStrength[motor][h] = 
                getMagnitudeAtFrequency(&fft, harmonicHz);
        }
    }
    
    // Store throttle-indexed data
    int throttleIdx = constrainf(throttle * 10, 0, 10);
    profile->noiseVsThrottle[throttleIdx] = fft.totalNoisePower;
    profile->frequencyVsThrottle[throttleIdx] = fft.peakFrequencies[0];
}
```

### Filter Delay Measurement

Critical for balancing noise reduction vs latency:

```c
typedef struct filterDelayMeasurement_s {
    float unfilteredGyro[DELAY_SAMPLE_COUNT];
    float filteredGyro[DELAY_SAMPLE_COUNT];
    float dtermUnfiltered[DELAY_SAMPLE_COUNT];
    float dtermFiltered[DELAY_SAMPLE_COUNT];
    
    float gyroDelayMs;
    float dtermDelayMs;
    float totalDelayMs;
} filterDelayMeasurement_t;

float measureFilterDelay(float *unfiltered, float *filtered, int count, float samplePeriodUs)
{
    // Cross-correlation to find delay
    float maxCorrelation = 0;
    int bestLag = 0;
    
    for (int lag = 0; lag < count / 4; lag++) {
        float correlation = 0;
        for (int i = 0; i < count - lag; i++) {
            correlation += unfiltered[i] * filtered[i + lag];
        }
        if (correlation > maxCorrelation) {
            maxCorrelation = correlation;
            bestLag = lag;
        }
    }
    
    return bestLag * samplePeriodUs / 1000.0f;  // Return in milliseconds
}
```

---

### User Selectable Tune Depth

```c
typedef enum {
    AUTOTUNE_DEPTH_QUICK,      // PID only, ~1 minute
    AUTOTUNE_DEPTH_STANDARD,   // PID + feedforward + filters, ~3 minutes  
    AUTOTUNE_DEPTH_COMPLETE,   // Everything, ~10 minutes
    AUTOTUNE_DEPTH_CUSTOM,     // User-selected phases
} autotuneDepth_e;

typedef struct autotuneConfigExpanded_s {
    uint8_t  tune_depth;              // autotuneDepth_e
    uint16_t phase_enable_mask;       // Bitmask of enabled phases
    
    // Phase 1-2: Characterization settings
    uint8_t  motor_characterization;  // 0=skip, 1=basic, 2=full
    
    // Phase 3: Filter tuning targets
    uint8_t  target_filter_delay_ms;  // Max acceptable filter delay
    uint8_t  target_noise_reduction;  // Noise reduction target (%)
    
    // Phase 4: PID tuning targets
    uint8_t  target_overshoot;        // 0-30%
    uint8_t  target_rise_time_ms;     // Target rise time
    uint8_t  response_feel;           // 0=soft, 50=balanced, 100=locked-in
    
    // Phase 5: Feedforward targets
    uint8_t  ff_aggressiveness;       // 0=smooth, 100=instant
    
    // Phase 6-7: Advanced tuning
    uint8_t  tune_tpa;                // Enable TPA tuning
    uint8_t  tune_antigravity;        // Enable anti-gravity tuning
    uint8_t  tune_iterm_relax;        // Enable I-term relax tuning
    uint8_t  tune_dmax;               // Enable D_max tuning
    
} autotuneConfigExpanded_t;
```

### Comprehensive Results Structure

```c
typedef struct autotuneResults_s {
    // Metadata
    uint32_t tuneTimestamp;
    uint8_t  tuneDepth;
    uint16_t totalTuneTimeSeconds;
    
    // Motor characteristics (discovered)
    uint16_t measuredMotorKv;
    uint16_t measuredIdleRpm;
    uint8_t  measuredPoleCount;
    
    // Noise characteristics (discovered)
    float    gyroNoiseFloorHz;
    float    dtermNoiseFloorHz;
    float    motorNoiseHz[4];          // Per-motor noise frequencies
    uint8_t  frameResonanceCount;
    float    frameResonanceHz[3];
    
    // Tuned PID values
    pidf_t   tunedPid[3];              // Roll, Pitch, Yaw
    
    // Tuned filter values
    uint16_t tunedGyroLpf1Hz;
    uint16_t tunedGyroLpf2Hz;
    uint16_t tunedDtermLpf1Hz;
    uint16_t tunedDtermLpf2Hz;
    uint16_t tunedDynNotchMinHz;
    uint16_t tunedDynNotchMaxHz;
    uint16_t tunedRpmFilterLpfHz;
    
    // Tuned feedforward
    uint16_t tunedF[3];
    uint8_t  tunedFfBoost;
    uint8_t  tunedFfSmoothFactor;
    
    // Tuned throttle response
    uint8_t  tunedTpaRate;
    uint16_t tunedTpaBreakpoint;
    uint8_t  tunedAntiGravityGain;
    
    // Tuned I-term
    uint8_t  tunedItermRelaxCutoff;
    
    // Tuned D_max
    uint8_t  tunedDmax[3];
    
    // Quality metrics
    float    finalOvershoot[3];        // Achieved overshoot per axis
    float    finalSettlingMs[3];       // Achieved settling time
    float    noiseReduction;           // % noise reduction achieved
    float    filterDelayMs;            // Total filter delay
    uint8_t  overallScore;             // 0-100 tune quality
    
} autotuneResults_t;
```

---

## Future Enhancements

### Phase 2 (Post-MVP): Advanced Filter Autotuning
- Fully automated gyro and D-term filter optimization
- Frame resonance detection and notch configuration
- Per-motor noise analysis for imbalanced props

### Phase 3: Adaptive Tuning
- Continuous background monitoring
- Detect tune degradation over time
- Suggest re-tune when performance drops

### Phase 4: Frame-Aware Tuning
- Detect frame resonance frequencies
- Adjust dynamic notch parameters
- Warn of problematic frame/prop combinations

### Phase 5: Machine Learning
- Collect tuning data from many flights
- Train model to predict good starting PIDs
- Use as initialization for autotune

---

## Hardware Feasibility Analysis

This section documents the hardware constraints and feasibility assessment for running
autotune algorithms live on STM32 MCUs commonly used in Betaflight flight controllers.

### Target Hardware Constraints

| MCU | RAM | Flash | Clock | Feasibility |
|-----|-----|-------|-------|-------------|
| STM32F411 | 128KB | 512KB | 100MHz | ⚠️ Limited - QUICK mode only |
| STM32F405 | 192KB | 512KB | 168MHz | ✅ Full support |
| STM32F722 | 256KB | 512KB | 216MHz | ✅ Full support |
| STM32F745 | 320KB | 1MB | 216MHz | ✅ Full support |
| STM32H743 | 1MB | 2MB | 480MHz | ✅ Full support with extras |
| AT32F435 | 384KB | 1MB | 288MHz | ✅ Full support |

### Computational Feasibility by Method

#### ✅ Fully Feasible (Low Overhead)

1. **Step Response Analysis**
   - Simple buffer tracking (~200 samples × 4 bytes = 800 bytes)
   - O(n) scan for rise time, overshoot, settling time
   - Can run within normal PID loop timing budget
   - Similar to existing ArduPilot/dRonin implementations

2. **Doublet Injection**
   - Same computational requirements as step response
   - Returns to original state - safer for in-flight

3. **Relay Feedback (Åström-Hägglund)**
   - Minimal computation: bang-bang control + period measurement
   - <1KB RAM, negligible CPU

4. **PID Adjustment Algorithms**
   - Simple gradient-based adjustments
   - Runs only after measurement phase completes

#### ⚠️ Feasible with Optimization

1. **Real-Time Spectral Analysis**
   - **DO NOT** implement new 256-point FFT
   - **REUSE** existing SDFT infrastructure from `src/main/common/sdft.c`
   - Betaflight already runs SDFT at 8kHz PID rates for dynamic notch
   - SDFT is O(N) per sample vs O(N log N) for FFT
   - Work is spread across PID loops (12 loops for full 3-axis update)

2. **Filter Optimization**
   - Leverage existing `dyn_notch_filter.c` spectral data
   - Don't create parallel analysis - tap into existing infrastructure
   - Time-slice expensive operations across multiple PID loops

### Key Implementation Constraints

1. **Memory Budget**
   ```
   Autotune runtime state:     ~500 bytes
   Sample history buffers:     ~2KB (can be reduced for F411)
   SDFT reuse (no new alloc):  0 bytes (already allocated)
   Results storage:            ~200 bytes
   Total new RAM required:     ~3KB typical, ~1.5KB minimum
   ```

2. **CPU Budget**
   - PID loop runs at 8kHz (125µs period) on most targets
   - Typical PID loop takes 20-40µs
   - Budget for autotune: ~10-20µs when active
   - Solution: Spread work across multiple loops (follow dyn_notch pattern)

3. **Existing Infrastructure to Reuse**
   - `src/main/common/sdft.c` - Sliding DFT (72 samples, 36 bins)
   - `src/main/flight/dyn_notch_filter.c` - Spectral analysis framework
   - `src/main/common/chirp.c` - Existing signal injection
   - `src/main/common/filter.c` - Biquad filter implementations

### Design Decisions Based on Hardware Analysis

1. **Conditional Compilation**: Use `#ifdef USE_AUTOTUNE` to exclude from F411 builds
2. **Tune Depth Scaling**: QUICK mode for all targets, COMPLETE mode for F4/F7/H7 only
3. **Buffer Size Configuration**: Reduce `AUTOTUNE_SAMPLE_COUNT` on constrained targets
4. **Leverage Existing SDFT**: No new FFT implementation - reuse dyn_notch infrastructure
5. **Time-Sliced Execution**: Follow the 4-step × 3-axis pattern from dyn_notch_filter

---

## Implementation Plan

This section defines a step-by-step implementation plan that can be tested incrementally.
Each milestone produces working, testable code before proceeding to the next phase.

### Milestone 0: Baseline Validation (Week 0) ✅ COMPLETE

**Goal**: Establish a known-good baseline by building and flying unmodified Betaflight source.

#### 0.1 Development Environment Setup
- [x] Clone Betaflight repository
- [x] Install ARM toolchain (arm-none-eabi-gcc v13.3.1)
- [x] Install build dependencies (make v4.4.1, python v3.14.2, Git Bash v2.52.0)
- [x] Verify `make` completes without errors for target board
- [x] Hydrate configs (`make configs`)

#### 0.2 Build Verification
- [x] Identify test hardware: BETAFPVG473 (STM32G47X) - BETAFPV Air G4 5in1
- [x] Build firmware for test hardware (`make CONFIG=BETAFPVG473`) - **COMPLETED**
- [x] Verify binary size is within flash limits (1,060,441 bytes - OK for STM32G47X)
- [x] Flash and verify on actual hardware

#### 0.3 Flash and Configure
- [x] Install DFU drivers (ImpulseRC Driver Fixer)
- [x] Flash built firmware to test drone
- [x] Configure via Betaflight Configurator
- [x] Verify all sensors working (gyro, acc, baro if present)
- [x] Verify RC link and motor outputs
- [x] Verify OSD working (if applicable)

#### 0.4 Flight Validation
- [x] Perform test hover
- [x] Verify stable flight with default/existing tune
- [x] Verify blackbox logging functional
- [x] Capture baseline blackbox log for later comparison

#### 0.5 SITL Setup (Optional but Recommended)
- [ ] Build SITL target (`make TARGET=SITL`)
- [ ] Verify SITL runs and connects to Configurator
- [ ] Test basic SITL flight simulation

**Test Criteria**: ✅ ALL PASSED
- Self-built firmware flies identically to official release
- Blackbox logs capture all expected data
- Development environment fully functional
- Ready to begin code modifications

**Notes**: Slower boot time observed on alpha firmware (expected for development builds)

---

### Milestone 1: Infrastructure & State Machine (Week 1-2) ✅ COMPLETE

**Goal**: Basic autotune framework that compiles and can be activated, but doesn't tune yet.

#### 1.1 Create Core Files ✅
```
src/main/flight/autotune.c      - Main state machine
src/main/flight/autotune.h      - Public interface
src/main/pg/autotune.c          - Parameter group
src/main/pg/autotune.h          - Configuration structure
```

#### 1.2 Implement State Machine Shell ✅
- [x] Define `autotuneState_e` enum (IDLE, SETUP, WAIT_STABLE, EXCITE, MEASURE, ANALYZE, ADJUST, COMPLETE, ABORTED)
- [x] Create `autotuneRuntime_t` structure with timing and state tracking
- [x] Implement `autotuneInit()`, `autotuneUpdate()`, `autotuneIsActive()`, `autotuneGetState()`
- [x] Add state transition logic with `changeState()` helper
- [x] Added `stateJustEntered` flag for reliable state initialization

#### 1.3 Add Box Mode Integration ✅
- [x] Add `BOXAUTOTUNE` to `src/main/fc/rc_modes.h`
- [x] Add mode definition in `src/main/msp/msp_box.c`
- [x] Hook into flight mode handling with `IS_RC_MODE_ACTIVE(BOXAUTOTUNE)`
- [x] Add beeper feedback (start, done, fail)

#### 1.4 Add Basic Safety Checks ✅
- [x] Attitude limit checks (±30° configurable)
- [x] Gyro rate validation (<500 deg/s)
- [x] Throttle range validation (20-80%, uses `calculateThrottlePercent()`)
- [x] Stick input detection with 20% deflection threshold
- [x] Immediate abort and state transition to ABORTED on safety violation
- [x] Continuous safety monitoring in all active states

#### 1.5 Additional Features Implemented ✅
- [x] Comprehensive debug output (8 channels) via `DEBUG_AUTOTUNE`
- [x] CLI commands for configuration (`get autotune`, `set autotune_*`)
- [x] Parameter group with sensible defaults
- [x] Build system integration with `USE_AUTOTUNE` flag
- [x] Integration with PID controller via `autotuneModifySetpoint()`

**Test Criteria**: ✅ ALL PASSED
- Autotune mode appears in Configurator
- Switch activates/deactivates mode correctly
- State machine transitions visible in debug[0]
- Safety abort works correctly (tested with excessive stick input)
- Flight validated: States progress 1→2→3→4→5→6→7
- Throttle bug fixed (calculateThrottlePercent vs manual calculation)

**Implementation Notes:**
- State initialization bug discovered and fixed with `stateJustEntered` flag
- Safety checks designed with 2s SETUP delay for stabilization
- Debug output proven invaluable for remote troubleshooting

---

### Milestone 2: Step Response Measurement (Week 3-4) ✅ COMPLETE

**Goal**: Inject step setpoint and measure response metrics accurately.

#### 2.1 Implement Excitation Injection ✅
- [x] Additive setpoint modification in PID loop (after acceleration limiting)
- [x] Configurable step amplitude (`autotune_step_amplitude`, default 100 deg/s)
- [x] Configurable step duration (`autotune_step_duration_ms`, default 100ms)
- [x] **Bidirectional doublet**: +A, -2A, +2A, -2A, +2A, -A (zero net displacement)
- [x] Total duration: 6× step_duration (600ms with defaults)
- [x] Proper timing using `cmpTimeUs()` with `timeDelta_t` types
- [x] Excitation only applied during EXCITE state on tested axis

#### 2.2 Implement Sample Collection ✅
- [x] Fixed-size buffers: `gyroHistory[200]`, `setpointHistory[200]`
- [x] Dynamic sample interval based on PID frequency (`AUTOTUNE_TARGET_SAMPLE_RATE_HZ = 200`)
- [x] Calculation: `sampleIntervalLoops = pidFrequency / targetSampleRate`
- [x] Subsample every N PID loops to maintain 200Hz regardless of PID loop rate
- [x] Synchronized collection during EXCITE and MEASURE states
- [x] Sample count tracking and buffer management

#### 2.3 Sample Collection Performance ✅
- EXCITE state: ~120 samples over 600ms doublet
- MEASURE state: ~60 samples over 300ms settling period
- Total: ~181 samples collected per test
- Memory: ~1.6KB (200 samples × 2 arrays × 4 bytes)

#### 2.4 Add Debug Output ✅
- [x] Debug mode `DEBUG_AUTOTUNE` with 8 channels:
  - debug[0]: State number
  - debug[1]: Sample count / State markers
  - debug[2]: Gyro rate (current axis)
  - debug[3]: Excitation signal (doublet pattern visible)
  - debug[4]: Safety check status
  - debug[5]: Throttle percentage
  - debug[6]: Stick deflection percentage  
  - debug[7]: Time elapsed in doublet (ms)

**Test Criteria**: ✅ ALL PASSED
- Bidirectional doublet injection visible in blackbox (clear +100/-200/+200/-200/+200/-100 pattern)
- Aircraft responds visibly to excitation (felt and seen in flight)
- 181 samples collected successfully (validated in blackbox)
- Sample rate consistent at 200Hz regardless of PID frequency
- State 3 (EXCITE) no longer skipped (initialization bug fixed)
- Natural return to hover after doublet (zero net displacement confirmed)

**Implementation Notes:**
- Evolved from simple doublet to bidirectional for better identification
- Fixed state initialization bug (exact time comparison → flag-based)
- Fixed type mismatch in duration comparisons (`uint32_t` → `timeDelta_t`)
- Dynamic sample rate calculation prevents issues when PID frequency changes
- Additive control model preserves pilot authority throughout tuning

**Key Design Decisions:**
- Bidirectional doublet chosen over single step for symmetry and return to origin
- Additive control model (pilot + autotune) vs override model for safety
- Sample buffers sized for ~1 second of data at 200Hz
- MEASURE state captures settling behavior after excitation stops

---

### Milestone 3: Basic PID Adjustment (Week 5-6) 🔄 IN PROGRESS

**Goal**: Complete single-axis P/D tuning loop that improves tune quality.

#### 3.1 Implement Step Response Analysis ⏳
- [ ] `findStepEdges()` - Detect transitions in setpointHistory[]
- [ ] `calculateRiseTime()` - Time from 10% to 90% of target
- [ ] `calculateOvershoot()` - Peak detection and percentage calculation
- [ ] `calculateSettlingTime()` - Time to stay within ±5% of target
- [ ] `detectOscillation()` - Zero-crossing detection for frequency
- [ ] `calculateGyroNoise()` - RMS noise measurement in stable regions
- [ ] Metric validation and sanity checking

#### 3.2 Implement Tune Score Calculation ⏳
- [ ] Weighted scoring function
- [ ] Penalize overshoot deviation from target
- [ ] Penalize slow rise time
- [ ] Penalize long settling time
- [ ] Heavily penalize oscillation
- [ ] Track best score across iterations

#### 3.3 Implement P/D Adjustment Algorithm ⏳
- [ ] **Independent P tuning** based on rise time and steady-state error
- [ ] **Independent D tuning** based on overshoot with noise constraint
- [ ] **Noise-limited tuning**: Block D increases if gyro noise too high
- [ ] Alternative: Reduce P when noise-limited instead of increasing D
- [ ] Gain limiting (min/max constraints)
- [ ] Step size limiting (max 20-30% change per iteration)
- [ ] Flag `noiseLimited` condition for user awareness

#### 3.4 Implement I-Term Adjustment ⏳
- [ ] Measure steady-state error after settling
- [ ] Adjust I based on residual tracking error
- [ ] Prevent I wind-up issues

#### 3.5 Implement Gain Application ⏳
- [ ] Store original gains before tuning
- [ ] Write calculated gains to `pidProfile()->pid[axis].P/I/D`
- [ ] Reinitialize PID controller with new gains
- [ ] Log old vs new gains for comparison

**Test Criteria**:
- Rise time calculation matches manual blackbox analysis
- Overshoot detection accurate
- Gyro noise RMS calculated correctly from stable regions
- Calculated gains are reasonable (compare to typical values)
- Gains successfully applied to PID controller
- Flight test shows improved response
- Noise-limited condition properly detected

**Key Features:**
- **Multivariable awareness**: P and D tuned independently with different metrics
- **Noise constraint**: D increases blocked if gyro RMS > threshold
- **Three-way tradeoff**: Documents responsiveness-efficiency-damping balance
- **Foundation for M5+**: PID tuning with fixed filtering, prepares for joint optimization

---

### Milestone 4: Multi-Axis & I-Term Tuning (Week 7-8)

**Goal**: Complete PID tuning for all axes including I-term.

#### 4.1 Implement Axis Sequencing
- [ ] Configure which axes to tune (R, P, Y, RP, RPY)
- [ ] Sequential axis progression
- [ ] Per-axis state tracking
- [ ] Proper timing between axes

#### 4.2 Implement I-Term Tuning
- [ ] Measure steady-state error after step
- [ ] Adjust I based on residual error
- [ ] Implement I-term specific test (sustained offset)

#### 4.3 Implement Yaw-Specific Handling
- [ ] Different step amplitude for yaw
- [ ] Different convergence criteria
- [ ] Handle yaw's typically lower D requirement

#### 4.4 Add User Feedback
- [ ] OSD progress display ("ROLL 3/10 P:45 D:32")
- [ ] Beeper feedback (start, axis complete, done, abort)
- [ ] LED indication if available

**Test Criteria**:
- All three axes tune sequentially
- Yaw tune produces sensible values
- Total tune time < 90 seconds for RPY
- OSD/beeper feedback works correctly

---

### Milestone 5: Flight Testing & Refinement (Week 9-10)

**Goal**: Validate on real hardware, refine algorithms based on flight data.

#### 5.1 Bench Testing
- [ ] Verify on STM32F405 target
- [ ] Verify on STM32F722 target
- [ ] Verify CPU usage acceptable
- [ ] Verify memory usage acceptable

#### 5.2 Tethered Flight Testing
- [ ] Test on 5" quad with safety tether
- [ ] Verify safety abort works
- [ ] Verify tune produces flyable result
- [ ] Compare to manual tune quality

#### 5.3 Open Flight Testing
- [ ] Test on multiple frame sizes (3", 5", 7")
- [ ] Test on different motor/prop combinations
- [ ] Collect success/failure data
- [ ] Tune algorithm parameters based on results

#### 5.4 Documentation
- [ ] User documentation
- [ ] CLI command reference
- [ ] Troubleshooting guide

**Test Criteria**:
- Successful tune on 5 different builds
- No crashes or flyaways during tuning
- Resulting tune rated "acceptable" by test pilots
- CPU overhead < 5% during tune

---

### Milestone 6: Feedforward Tuning (Week 11-12)

**Goal**: Add feedforward optimization to improve response tracking.

#### 6.1 Implement FF Measurement
- [ ] Fast RC step injection
- [ ] Measure motor response lag vs setpoint
- [ ] Measure acceleration response

#### 6.2 Implement FF Adjustment
- [ ] Optimize F gain for minimum tracking delay
- [ ] Optimize FF boost for acceleration response
- [ ] Calculate optimal smooth_factor based on RC noise

#### 6.3 Integration Testing
- [ ] Verify FF tune improves stick response
- [ ] Verify no adverse interaction with PID tune
- [ ] Add to tune sequence (after PID)

**Test Criteria**:
- Measurable improvement in setpoint tracking
- No increase in motor noise/heat
- FF values within reasonable range

---

### Milestone 7: Filter Optimization (Week 13-16)

**Goal**: Automatic filter tuning using existing SDFT infrastructure.

#### 7.1 Tap Into SDFT Data
- [ ] Access existing `dynNotch` spectral data
- [ ] Add hooks for autotune to read frequency bins
- [ ] Implement throttle-indexed noise profiling

#### 7.2 Implement D-Term Filter Tuning
- [ ] Measure D-term noise at various throttles
- [ ] Calculate optimal LPF1/LPF2 cutoffs
- [ ] Balance noise reduction vs latency

#### 7.3 Implement Gyro Filter Tuning
- [ ] Analyze gyro noise spectrum
- [ ] Optimize gyro LPF cutoffs
- [ ] Adjust dynamic notch min/max range

#### 7.4 Implement RPM Filter Optimization
- [ ] Measure idle RPM for min_hz
- [ ] Analyze harmonic strength for weights
- [ ] Optimize Q factor

**Test Criteria**:
- Filter tune reduces noise without hurting response
- Motor temperatures not increased
- Prop wash handling maintained or improved

---

### Future Milestones (Post-MVP)

- **Milestone 8**: TPA and Anti-Gravity tuning
- **Milestone 9**: I-term relax and D_max tuning
- **Milestone 10**: Adaptive/continuous tuning
- **Milestone 11**: Frame resonance detection
- **Milestone 12**: Machine learning initialization

---

## Development Guidelines

### Code Style
- Follow existing Betaflight coding conventions
- Use `FAST_CODE` for hot path functions
- Use `FAST_DATA_ZERO_INIT` for frequently accessed runtime data

### Testing Requirements
- Unit tests for all analysis functions
- SITL testing before hardware testing
- Blackbox logging for all tune sessions

### Safety Requirements
- Always store original PIDs before modification
- Immediate restore on any safety violation
- Never apply gains outside safe limits
- Pilot stick input always overrides

### Memory Optimization
- Use fixed-point where float not required
- Reuse buffers between phases
- Clear buffers when phase complete

---

## References

1. Åström, K. J., & Hägglund, T. (1984). Automatic tuning of simple regulators with specifications on phase and amplitude margins. Automatica, 20(5), 645-651.

2. ArduPilot AutoTune: https://ardupilot.org/copter/docs/autotune.html

3. dRonin AutoTune: https://github.com/d-ronin/dRonin/wiki/AutoTune

4. Betaflight CHIRP implementation: `src/main/common/chirp.c`

5. Ziegler, J. G., & Nichols, N. B. (1942). Optimum settings for automatic controllers. Transactions of the ASME, 64(11).

6. Betaflight SDFT implementation: `src/main/common/sdft.c`

7. Betaflight Dynamic Notch: `src/main/flight/dyn_notch_filter.c`

---

## Appendix: Comparison with Other Implementations

| Feature | ArduPilot | dRonin | Proposed BF |
|---------|-----------|--------|-------------|
| Method | Relay feedback | Step response | Multiple |
| In-flight | Yes | Yes | Yes |
| Per-axis | Yes | Yes | Yes |
| Filter tune | Limited | No | Planned |
| FF tune | No | Yes | Planned |
| Small quad optimized | No | No | Yes |
| Time to tune | 5-10 min | 2-5 min | 1-3 min target |
