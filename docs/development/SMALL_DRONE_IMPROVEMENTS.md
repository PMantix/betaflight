# Small Drone PID Controller Improvements Plan

## Overview
This document outlines proposed improvements to the Betaflight PID controller and related systems
to better support small drones (whoops, 2-3" builds) which have fundamentally different dynamics
than the 5" quads the codebase is primarily tuned for.

## Problem Statement
Small drones suffer from:
- Much lower rotational inertia (faster response)
- Higher motor KV and RPM ranges
- Higher frequency noise characteristics
- Different thrust-to-weight ratios
- More sensitivity to filter delay

---

## Phase 1: RPM Filter Improvements

### 1.1 Dynamic RPM Filter LPF Cutoff
**File:** `src/main/drivers/dshot.c`
**Current:** Fixed 150Hz LPF on motor frequency
**Problem:** Too slow for high-KV small motors (500-1200Hz fundamental)
**Solution:** Scale LPF cutoff based on motor characteristics

```c
// Proposed change in initDshotTelemetry()
float scaledCutoff = rpmFilterConfig()->rpm_filter_lpf_hz;
if (motorConfig()->motorKv > 0) {
    scaledCutoff *= constrainf(motorConfig()->motorKv / 2000.0f, 0.5f, 2.5f);
}
pt1FilterInit(&motorFreqLpf[i], pt1FilterGain(scaledCutoff, looptimeUs * 1e-6f));
```

**Status:** [ ] Not Started

### 1.2 Adjustable RPM Filter Min Hz
**File:** `src/main/pg/rpm_filter.c`
**Current:** Default min_hz = 100
**Problem:** May be too low for small drone idle RPMs
**Solution:** Add CLI parameter or auto-detect based on observed idle RPM

**Status:** [ ] Not Started

### 1.3 Faster Notch Fade Range
**File:** `src/main/flight/rpm_filter.c`
**Current:** 50Hz fade range
**Problem:** Notches linger during rapid throttle cuts on low-inertia motors
**Solution:** Make fade range proportional to motor deceleration rate

**Status:** [ ] Not Started

---

## Phase 2: RPM Limiter Fixes

### 2.1 Increase RPM Averaging Filter Speed
**File:** `src/main/flight/mixer_init.c`
**Line:** 383
**Current:** 6Hz PT1 filter on RPM average
**Problem:** Causes massive overshoot on fast-responding motors
**Solution:** Increase to 20-30Hz, or make configurable

```c
// Current
pt1FilterInit(&mixerRuntime.rpmLimiterAverageRpmFilter, pt1FilterGain(6.0f, pidGetDT()));

// Proposed
pt1FilterInit(&mixerRuntime.rpmLimiterAverageRpmFilter, pt1FilterGain(25.0f, pidGetDT()));
```

**Status:** [ ] Not Started

### 2.2 Rate-Based D-Term for RPM Limiter
**File:** `src/main/flight/mixer.c`
**Current:** D-term uses error derivative (filtered)
**Problem:** D response is too slow due to filtering
**Solution:** Use actual RPM rate of change

```c
// Proposed change in applyRpmLimiter()
static float prevAverageRpm = 0.0f;
const float rpmRate = (averageRpm - prevAverageRpm) * pidGetPidFrequency();
const float d = rpmRate * mixer->rpmLimiterDGain;
prevAverageRpm = averageRpm;
```

**Status:** [ ] Not Started

### 2.3 Add RPM Limiter Feedforward
**File:** `src/main/flight/mixer.c`
**Problem:** Limiter is purely reactive
**Solution:** Add feedforward from throttle command to anticipate RPM changes

**Status:** [ ] Not Started

### 2.4 Configurable RPM Limiter Gains
**File:** `src/main/flight/mixer.h`
**Problem:** Fixed scaling factors don't suit all motor types
**Solution:** Expose gain scaling in CLI for advanced users

**Status:** [ ] Not Started

---

## Phase 3: D-Term Filter Optimization

### 3.1 Small Drone Filter Presets
**File:** `src/main/flight/pid_init.c`
**Problem:** Default filter cutoffs tuned for 5" builds
**Solution:** Add frame-size aware presets

Suggested small drone defaults:
- dterm_lpf1_dyn_min_hz = 100 (was 75)
- dterm_lpf1_dyn_max_hz = 250 (was 150)
- dterm_lpf2_static_hz = 200 (was 150)

**Status:** [ ] Not Started

### 3.2 PT3 as Default for Small Builds
**File:** `src/main/flight/pid_init.c`
**Problem:** PT1/Biquad have worse phase characteristics
**Solution:** Recommend or default to PT3 for D-term on small builds

**Status:** [ ] Not Started

---

## Phase 4: PID Scaling Improvements

### 4.1 Frame-Size Aware Scaling Factors
**File:** `src/main/flight/pid.h`
**Problem:** PTERM_SCALE, ITERM_SCALE, DTERM_SCALE fixed for 5" quads
**Solution:** Optional scaling multiplier based on declared frame size

**Status:** [ ] Not Started

### 4.2 I-Term Relax Threshold Scaling
**File:** `src/main/flight/pid.c`
**Problem:** Fixed 40 deg/sec threshold doesn't scale with max rates
**Solution:** Scale threshold relative to configured max rate

```c
// Proposed
float itermRelaxThreshold = ITERM_RELAX_SETPOINT_THRESHOLD * 
    (currentControlRateProfile->rates[axis] / 70.0f); // normalize to "typical" rate
```

**Status:** [ ] Not Started

### 4.3 Anti-Gravity TWR Awareness
**File:** `src/main/flight/pid.c`
**Problem:** Fixed anti-gravity gains assume typical TWR
**Solution:** Scale based on declared or measured TWR

**Status:** [ ] Not Started

---

## Phase 5: Feedforward Improvements

### 5.1 Rate-Relative Jitter Attenuation
**File:** `src/main/fc/rc.c`
**Problem:** Jitter factor doesn't scale with configured rates
**Solution:** Normalize jitter calculation to max rate

**Status:** [ ] Not Started

### 5.2 Higher Rate RC Link Normalization
**File:** `src/main/flight/pid_init.c`
**Problem:** Smoothing normalized to 250Hz (ELRS default)
**Solution:** Auto-detect or make configurable for 500Hz+ links

**Status:** [ ] Not Started

---

## Phase 6: Autotuning Feature (NEW - See AUTOTUNE_DESIGN.md)

### 6.1 In-Flight PID Autotuning System
**New Files Required:**
- `src/main/flight/autotune.c`
- `src/main/flight/autotune.h`
- `src/main/pg/autotune.h`

**Status:** [ ] Design Phase

---

## Testing Plan

### Unit Tests
- [ ] RPM filter tracking accuracy at high frequencies
- [ ] RPM limiter step response
- [ ] D-term filter phase delay measurements

### Flight Tests
- [ ] 65mm whoop (1S)
- [ ] 75mm whoop (1S)
- [ ] 2" build (2S)
- [ ] 3" build (2-3S)
- [ ] Compare against 5" reference

---

## CLI Parameters (Proposed New/Modified)

```
# RPM Filter
rpm_filter_lpf_hz       # Existing - consider higher default
rpm_filter_lpf_auto     # NEW: Auto-scale based on motor KV

# RPM Limiter
rpm_limit_filter_hz     # NEW: RPM averaging filter cutoff (default 25)
rpm_limit_p_scale       # NEW: P gain scaling
rpm_limit_d_mode        # NEW: 0=error, 1=rpm_rate

# Frame Size
frame_class             # NEW: 0=auto, 1=whoop, 2=micro, 3=mini, 4=standard
```

---

## References
- Betaflight GitHub Issues related to small quad tuning
- RotorFlight (helicopter fork) filter implementations
- Academic papers on multirotor PID autotuning
