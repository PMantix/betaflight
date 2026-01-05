# Milestone 3 Plan: Step Response Analysis & PID Gain Calculation

**Target Timeline:** Week 5-6  
**Status:** 🔄 PLANNING

## Overview
Analyze the collected step response data (gyro + setpoint samples) to extract dynamic characteristics and calculate optimal PID gains.

## Prerequisites
- ✅ Milestone 2 complete: 181 samples collected (gyroHistory[], setpointHistory[])
- ✅ Bidirectional doublet excitation working
- ✅ Clean step response data available

## Goals
1. Extract step response metrics from sample buffers
2. Calculate optimal PID gains based on response characteristics
3. Apply new gains to PID controller
4. Prepare for iterative testing (Milestone 4)

## Technical Approach

### Phase 1: Step Response Analysis (ANALYZE State)

#### Metrics to Extract:

1. **Rise Time**
   - Time from 10% to 90% of final value
   - Indicates system responsiveness
   - Faster rise time → higher P gain beneficial

2. **Overshoot Percentage**
   - Maximum deviation beyond target / target × 100%
   - Indicates oscillation tendency
   - High overshoot → reduce P, increase D

3. **Settling Time**
   - Time to reach and stay within ±5% of target
   - Indicates stability
   - Long settling → need better damping

4. **Oscillation Frequency**
   - Dominant frequency of ringing/oscillation
   - Useful for D-term tuning
   - Can use FFT or zero-crossing detection

5. **Steady-State Error**
   - Final error after settling
   - Indicates I-term effectiveness
   - Non-zero error → increase I

6. **Gyro Noise Floor** ⚠️ CRITICAL
   - RMS gyro noise during stable hover (before/after doublet)
   - Measure noise amplification through D-term
   - Key constraint: don't increase D if noise already high
   - Motor heating and efficiency depend on this

#### Implementation Strategy:

```c
// In ANALYZE state:
void autotuneAnalyzeResponse(void)
{
    // 1. Find step transitions in setpointHistory[]
    // 2. For each step, analyze corresponding gyroHistory[]
    // 3. Calculate metrics for each phase of doublet
    // 4. Average metrics across multiple steps
    // 5. Store results in runtime structure
}
```

**Key Functions Needed:**
- `findStepEdges()` - Detect transitions in setpoint
- `calculateRiseTime()` - 10-90% time
- `calculateOvershoot()` - Peak detection
- `calculateSettlingTime()` - ±5% band convergence
- `detectOscillation()` - Frequency analysis (simple zero-crossing)
- `calculateGyroNoise()` - RMS noise in stable regions (pre/post doublet)

### Phase 2: PID Gain Calculation (ADJUST State)

#### Independent P and D Tuning Strategy:

**P Gain Tuning (Responsiveness):**
- Primary metric: Rise time
- Secondary: Steady-state tracking error
- Independent of D-term

**D Gain Tuning (Damping vs Noise):**
- Primary metric: Overshoot percentage
- Critical constraint: Gyro noise floor
- Trade-off: Damping vs motor heating

#### Tuning Rules to Implement:

**Step 1: Tune P Gain (Responsiveness)**
```c
// P affects rise time and tracking
if (rise_time > target_rise_time * 1.1) {
    // Too slow, need more P
    P_gain *= 1.1;
} else if (rise_time < target_rise_time * 0.9) {
    // Too aggressive, reduce P
    P_gain *= 0.95;
}

if (steady_state_error > threshold) {
    // Poor tracking
    P_gain *= 1.05;
}
```

**Step 2: Tune D Gain (Damping with Noise Constraint)**
```c
// D affects damping but amplifies noise
if (overshoot > target_overshoot + margin) {
    // Underdamped, need more D
    if (gyro_noise_rms < noise_threshold) {
        // Safe to increase D
        D_gain *= 1.1;
    } else {
        // Noise too high, can't increase D
        // Reduce P instead for less oscillation
        P_gain *= 0.95;
        // Flag: noise-limited tuning
    }
} else if (overshoot < target_overshoot - margin && gyro_noise_rms < noise_threshold * 0.5) {
    // Overdamped and noise is low, can reduce D
    D_gain *= 0.95;
}

if (oscillation_detected && gyro_noise_rms < noise_threshold) {
    // Underdamped, increase D
    D_gain *= 1.15;
}
```

**Step 3: Tune I Gain (Steady-State)**
```c
}

**Step 3: Tune I Gain (Steady-State)**
```c
// I affects steady-state error and wind-up
if (steady_state_error > threshold) {
    // Need more integral action
    I_gain *= 1.1;
} else if (steady_state_error < threshold * 0.5) {
    // May have too much I (potential wind-up)
    I_gain *= 0.95;
}
```

**Key Insight: Noise-Limited Tuning**
- If gyro noise is high, we hit a fundamental limit
- Can't increase D without overheating motors
- Must accept more overshoot OR reduce P
- This is why filtering (gyro lowpass, D-term lowpass) is critical
- Flag this condition to user: "Tuning limited by gyro noise"
```

**Constraints:**
- Minimum/maximum gain limits (safety)
- Rate of change limits (max 20-30% per iteration)
- Noise threshold (e.g., gyro RMS < 3 deg/s for D increases)
- Store original gains for comparison
- P and D tuned independently based on different metrics

#### Gain Application:
- Write to `pidProfile()->pid[axis].P/I/D`
- Mark profile as modified
- Log old vs new gains for comparison

### Phase 3: State Machine Updates

**ANALYZE State:**
```c
case AUTOTUNE_STATE_ANALYZE:
    if (runtime.stateJustEntered) {
        runtime.stateJustEntered = false;
        // Start analysis
        autotuneAnalyzeResponse();
        // Analysis complete, move to adjust
        changeState(AUTOTUNE_STATE_ADJUST, currentTimeUs);
    }
    break;
```

**ADJUST State:**
```c
case AUTOTUNE_STATE_ADJUST:
    if (runtime.stateJustEntered) {
        runtime.stateJustEntered = false;
        // Calculate new gains
        autotuneCalculateGains();
        // Apply gains
        autotuneApplyGains();
        // Log results
        DEBUG_SET(DEBUG_AUTOTUNE, 1, 6666); // Adjustment marker
        // For now, move to complete
        changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
        beeper(BEEPER_AUTOTUNE_DONE);
    }
    break;
```

## Data Structures

### Analysis Results:
```c
typedef struct autotuneAnalysis_s {
    float riseTime;           // microseconds
    float overshoot;          // percentage
    float settlingTime;       // microseconds
    float oscillationFreq;    // Hz
    float steadyStateError;   // deg/s
    float gyroNoiseRms;       // deg/s RMS in stable regions
    bool noiseLimited;        // true if tuning constrained by noise
    bool valid;               // analysis succeeded
} autotuneAnalysis_t;

// Add to runtime structure:
autotuneAnalysis_t analysis;
pidGains_t originalGains;  // Store for comparison
pidGains_t calculatedGains; // New gains to apply
```

## Algorithm Details

### Rise Time Calculation:
```c
float calculateRiseTime(float* gyro, float* setpoint, uint16_t count)
{
    // Find first major step in setpoint
    int stepIndex = findStepEdge(setpoint, count);
    float stepTarget = setpoint[stepIndex];
    
    // Find 10% and 90% points in gyro response
    float threshold10 = stepTarget * 0.1;
    float threshold90 = stepTarget * 0.9;
    
    int index10 = findThresholdCrossing(gyro, count, stepIndex, threshold10);
    int index90 = findThresholdCrossing(gyro, count, stepIndex, threshold90);
    
    // Rise time = time difference
    float riseTime = (index90 - index10) * samplePeriod;
    return riseTime;
}
```

### Overshoot Calculation:
```c
float calculateOvershoot(float* gyro, float* setpoint, uint16_t count, int stepIndex)
{
    float target = setpoint[stepIndex];
    
    // Find peak after step
    float peak = 0;
    for (int i = stepIndex; i < count; i++) {
        if (fabsf(gyro[i]) > fabsf(peak)) {
            peak = gyro[i];
        }
    }
    
    // Overshoot percentage
    float overshoot = (fabsf(peak) - fabsf(target)) / fabsf(target) * 100.0f;
    return overshoot;
}
```

### Gyro Noise Calculation:
```c
float calculateGyroNoise(float* gyro, uint16_t count, int startIdx, int endIdx)
{
    // Calculate RMS noise in stable region (before or after doublet)
    // Stable region = setpoint near zero, no excitation
    
    float sumSquares = 0;
    int samples = 0;
    
    for (int i = startIdx; i < endIdx; i++) {
        sumSquares += gyro[i] * gyro[i];
        samples++;
    }
    
    float rms = sqrtf(sumSquares / samples);
    return rms;  // deg/s RMS
}

// Noise thresholds for D-term increase decisions:
// < 2 deg/s RMS: Excellent, safe to increase D
// 2-4 deg/s RMS: Good, can increase D cautiously
// 4-8 deg/s RMS: Marginal, avoid increasing D
// > 8 deg/s RMS: Poor, noise-limited, consider filtering improvements
```

## Debug Output Updates

Add to debug channels during ANALYZE/ADJUST:
- **debug[1]:** Analysis step progress (marker 6666 = gains adjusted)
- **debug[2]:** Rise time (ms)
- **debug[3]:** Overshoot (%)
- **debug[4]:** Gyro noise RMS × 10 (for visibility)
- **debug[5]:** Old P gain (scaled for display)
- **debug[6]:** New P gain (scaled for display)
- **debug[7]:** Noise-limited flag (0 = no, 1 = yes)

## Testing Strategy

### Bench Test (Props Off):
1. Run autotune with known "bad" gains (e.g., very low P)
2. Verify analysis detects slow rise time
3. Confirm calculated gains are higher
4. Check gains applied to PID structure

### Initial Flight Test:
1. Start with moderately detuned quad
2. Run autotune
3. **DO NOT AUTO-SAVE** - manual review first
4. Compare old vs new gains
5. Sanity check (reasonable values)
6. Apply manually and test fly

### Validation:
- Blackbox before/after comparison
- Step response should show improvement
- Overshoot should approach target (10%)
- Rise time should be acceptable

## Safety Considerations

1. **Gain Limits:**
   - Absolute maximum P/I/D values
   - Prevent runaway multiplication
   - Sanity checks before applying

2. **Change Rate Limits:**
   - Maximum % change per iteration (e.g., 30%)
   - Gradual convergence safer than large jumps

3. **Validation:**
   - Check calculated gains against known-good ranges
   - Flag suspicious values
   - Require manual confirmation for extreme changes

4. **Rollback:**
   - Store original gains
   - CLI command to restore previous values
   - Auto-restore on crash detection?

## Success Criteria

- ✅ Analysis extracts valid metrics from sample data
- ✅ Rise time calculated correctly (compare with manual blackbox analysis)
- ✅ Overshoot detection accurate
- ✅ Calculated gains are reasonable (compare to typical values)
- ✅ Gains successfully applied to PID controller
- ✅ Flight test shows improved response
- ✅ No stability issues with new gains

## Potential Challenges

1. **Noisy Data:**
   - Real flight has vibration, wind disturbances
   - May need filtering or averaging
   - Multiple iterations help (Milestone 4)

2. **Edge Detection:**
   - Setpoint steps may not be perfectly square
   - Gyro response may have pre-ringing
   - Need robust thresholding

3. **Metric Interpretation:**
   - Different axes may need different tuning
   - Roll/pitch may respond differently
   - Need axis-specific logic

4. **Gain Mapping:**
   - Relationship between metrics and gains not perfectly linear
   - May need empirical tuning of the tuning algorithm
   - Start conservative, refine later

5. **The Multivariable Problem:** ⚠️ FUTURE CONSIDERATION
   - PID gains and filtering are intimately coupled
   - High D amplifies noise → need more filtering → phase lag → poor response
   - Low filtering → responsive → noisy D → can't increase D → limited damping
   - **Three-way tradeoff:** Responsiveness vs Efficiency vs Damping
   - **Milestone 3 scope:** Tune PIDs with existing filtering
   - **Future milestone:** Joint optimization of PIDs + gyro filters + D-term filters
   - This is the real value - finding global optimum in multidimensional space
   - Pilots currently spend 10-50+ flights finding this balance manually

## Next Steps After Milestone 3

**Milestone 4:** Iteration & Convergence
- Test new gains
- Re-run doublet with updated PIDs
- Compare metrics
- Iterate until target performance achieved
- Multi-axis support (roll, pitch, yaw)

**Milestone 5+:** Multivariable Optimization (The Real Magic)
- Joint optimization of PIDs + gyro filtering + D-term filtering
- Measure noise floor at different filter settings
- Test response with different filter/PID combinations
- Find optimal point in 3D space: Responsiveness-Efficiency-Damping
- This is what separates good tuning from exceptional tuning
- Potential for automated filter sweep + PID re-tune
- Could save 10-50 flights of manual iteration per quad

## Files to Modify

- `src/main/flight/autotune.c`
  - Add analysis functions
  - Implement gain calculation
  - Update ANALYZE and ADJUST states
  
- `src/main/flight/autotune.h`
  - Add analysis result structure
  - Declare new functions

- `src/main/pg/autotune.c/h`
  - Add tuning parameters (target overshoot, rise time limits, etc.)

## Estimated Effort

- **Analysis Implementation:** 2-3 hours
- **Gain Calculation:** 2-3 hours
- **Testing & Debugging:** 3-4 hours
- **Flight Validation:** 1-2 hours
- **Documentation:** 1 hour

**Total:** ~10-13 hours over 1-2 weeks

## Open Questions

1. Should we use FFT for oscillation detection or simpler zero-crossing?
   - **Decision:** Start with zero-crossing (simpler, faster)

2. How aggressive should gain adjustments be?
   - **Decision:** Conservative 10-20% changes per iteration

3. Should we tune all axes simultaneously or one at a time?
   - **Decision:** One at a time (current design), multi-axis in M4

4. What's the target overshoot percentage?
   - **Decision:** Use `target_overshoot` from config (default 10%)

5. How to handle failed analysis (noisy/invalid data)?
   - **Decision:** Mark as invalid, abort to ABORTED state, keep original gains

## References

- Ziegler-Nichols tuning method
- PID controller theory (rise time, overshoot relationships)
- Betaflight PID scaling and ranges
- Step response analysis techniques

## Conclusion

Milestone 3 will transform the collected sample data into actionable PID gains. The analysis phase extracts key metrics (rise time, overshoot, settling time) and the adjustment phase calculates improved gains using empirical tuning rules. Success will be measured by improved flight characteristics visible in blackbox logs.

**Ready to begin implementation once Milestone 2 documentation is complete.**
