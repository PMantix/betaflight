# Milestone 3 Review: Critical Issues & Proposed Redesign

**Date:** January 5, 2026  
**Status:** 🔴 NEEDS REDESIGN

---

## Executive Summary

After reviewing the M3 implementation and flight test data, there are **fundamental issues** with the current approach:

1. **Overshoot calculation is buggy** - Step direction/magnitude not properly handled
2. **Gain adjustment strategy is backwards** - Not aligned with proven tuning methodology
3. **Convergence is slow/unreliable** - Takes too many iterations without getting dialed in
4. **No proper P/D balance sequence** - Trying to optimize both simultaneously

### The Core Problem

**Current approach:** Try to adjust P and D simultaneously based on complex priority rules
**Your proven approach:** Sequential methodology that builds upon itself

---

## Issue 1: Overshoot Calculation Bugs

### Current Code Analysis

```c
static float calculateOvershoot(uint16_t stepIdx, float newSetpoint)
{
    // Get the previous setpoint to calculate step magnitude
    const float prevSetpoint = (stepIdx > 0) ? runtime.setpointHistory[stepIdx - 1] : 0.0f;
    const float stepMagnitude = fabsf(newSetpoint - prevSetpoint);
    
    // Get initial gyro position
    const float initialGyro = runtime.gyroHistory[stepIdx];
    const float stepDirection = (newSetpoint > prevSetpoint) ? 1.0f : -1.0f;
    
    // Search for peak in the direction of the step
    float peakGyro = initialGyro;
    for (uint16_t i = stepIdx; i < searchEnd; i++) {
        if (stepDirection > 0) {
            if (runtime.gyroHistory[i] > peakGyro) {
                peakGyro = runtime.gyroHistory[i];
            }
        } else {
            if (runtime.gyroHistory[i] < peakGyro) {
                peakGyro = runtime.gyroHistory[i];
            }
        }
    }
    
    // Calculate how much the gyro traveled in the step direction
    const float gyroTravel = (peakGyro - initialGyro) * stepDirection;
    
    // Overshoot percentage: how much travel exceeds step magnitude
    if (gyroTravel > stepMagnitude) {
        return ((gyroTravel - stepMagnitude) / stepMagnitude) * 100.0f;
    }
    return 0.0f;
}
```

### Problems Identified

**Problem 1: Bidirectional doublet confusion**
- Bidirectional pattern: `+A, -2A, +2A, -2A, +2A, -A`
- Example step: `+100 → -200` (step magnitude = 300 deg/s)
- If gyro goes from `+50 → -250`, that's 300 deg/s travel = **0% overshoot** ✅
- But if gyro goes from `+50 → -280`, that's 330 deg/s travel = **10% overshoot** ✅
- **BUG**: Code may not handle starting position correctly for negative steps

**Problem 2: Initial gyro position assumption**
- Assumes `initialGyro` at step edge is the "starting point"
- But system has momentum - gyro might already be moving
- For bidirectional doublet, previous phase affects this phase
- **Need**: Look back to find actual stable starting point (pre-step baseline)

**Problem 3: Peak search direction logic**
- Correctly searches in step direction
- BUT: What if system oscillates and peak is in wrong direction?
- Example: Step is -200 deg/s, but system bounces positive first
- **Missing**: Overshoot in opposite direction of step

**Problem 4: Magnitude scaling issues**
```c
const float gyroTravel = (peakGyro - initialGyro) * stepDirection;
```
- This makes travel always positive (absolute value)
- But then dividing by `stepMagnitude` assumes same units
- For large negative steps, this creates weird percentages

### Proposed Fix

```c
static float calculateOvershoot(uint16_t stepIdx, float newSetpoint)
{
    if (stepIdx < 5 || stepIdx >= runtime.sampleCount - 10) {
        return 0.0f;
    }
    
    // Get previous setpoint to calculate step magnitude and direction
    const float prevSetpoint = runtime.setpointHistory[stepIdx - 1];
    const float stepMagnitude = fabsf(newSetpoint - prevSetpoint);
    const float stepDirection = (newSetpoint > prevSetpoint) ? 1.0f : -1.0f;
    
    if (stepMagnitude < 10.0f) {  // Ignore small changes
        return 0.0f;
    }
    
    // Find stable baseline BEFORE the step (look back 5-10 samples)
    float baselineGyro = 0.0f;
    const uint16_t baselineStart = stepIdx - 5;
    for (uint16_t i = baselineStart; i < stepIdx; i++) {
        baselineGyro += runtime.gyroHistory[i];
    }
    baselineGyro /= 5.0f;  // Average of 5 samples before step
    
    // Search for peak response in step direction
    const uint16_t searchSamples = (autotuneConfig()->step_duration_ms * AUTOTUNE_TARGET_SAMPLE_RATE_HZ) / 1000;
    const uint16_t searchEnd = MIN(stepIdx + searchSamples, runtime.sampleCount);
    
    float peakGyro = baselineGyro;
    float extremeGyro = baselineGyro;  // Track opposite direction too
    
    for (uint16_t i = stepIdx; i < searchEnd; i++) {
        const float gyro = runtime.gyroHistory[i];
        
        // Track peak in step direction
        if (stepDirection > 0) {
            if (gyro > peakGyro) peakGyro = gyro;
            if (gyro < extremeGyro) extremeGyro = gyro;  // Opposite direction
        } else {
            if (gyro < peakGyro) peakGyro = gyro;
            if (gyro > extremeGyro) extremeGyro = gyro;  // Opposite direction
        }
    }
    
    // Calculate actual gyro travel in step direction
    const float gyroTravel = fabsf(peakGyro - baselineGyro);
    const float targetTravel = stepMagnitude;  // This is what we wanted
    
    // Overshoot is excess travel beyond the step magnitude
    if (gyroTravel > targetTravel) {
        const float overshootAmount = gyroTravel - targetTravel;
        return (overshootAmount / targetTravel) * 100.0f;
    }
    
    // Check for reverse overshoot (oscillation bouncing back)
    const float reverseTravel = fabsf(extremeGyro - baselineGyro);
    if (reverseTravel > targetTravel * 0.1f) {  // More than 10% in wrong direction
        return (reverseTravel / targetTravel) * 100.0f;
    }
    
    return 0.0f;
}
```

---

## Issue 2: Gain Adjustment Strategy is Wrong

### Your Proven Methodology (Manual Tuning)

```
Step 1: Find P/D Balance
├─ Start with reasonable ratio (e.g., D = 0.6-0.8 × P)
├─ Ensure system is critically damped or slightly overdamped
└─ Goal: No oscillation, minimal overshoot

Step 2: Push Gains Together
├─ Increase P and D proportionally (maintaining ratio)
├─ Watch for D-term oscillation (30-40Hz noise)
├─ Stop when you see high-frequency D noise
└─ Goal: Maximum gains without motor heating

Step 3: Back Off Slightly
├─ Reduce D by 10-20% to get below noise threshold
├─ Optionally reduce P by 5-10%
└─ Goal: Clean, efficient, responsive

Step 4: Add Feedforward
├─ Increase F gain until stick response is crisp
├─ Typically can achieve 7ms full stick response time
└─ Goal: Minimize tracking error, maximize feel
```

**Key Insight:** You establish the **P/D ratio FIRST**, then scale it up together.

### Current Implementation Problems

```c
// Priority 1: Both oscillations → P *= 0.90, D *= 0.95
// Priority 2: D-term osc >15Hz  → D *= 0.80
// Priority 3: D-term osc >0Hz   → D *= 0.90, P *= 1.05
// Priority 4: Gyro osc >0Hz     → D *= 1.15
// Priority 5: High overshoot    → D *= 1.15 or P *= 0.95
// Priority 6: Clean response    → P *= 1.10
```

**Problems:**
1. **No P/D ratio establishment** - P and D move independently without coordination
2. **Priority conflicts** - "Clean response → increase P" vs "Overshoot → increase D"
3. **Slow convergence** - Small 5-15% changes take many iterations
4. **D can collapse** - Aggressive D reduction (0.80×) drives D too low (4-8)
5. **P gets stuck** - Not increasing aggressively enough when safe to do so

### Flight Test Evidence (btfl_011.bbl.csv)

```
Iteration 0: P=45, D=30 → D-term osc 33Hz
Iteration 1: P=45, D=24 → D-term osc 40Hz  (D reduced, but still oscillating)
Iteration 2: P=45, D=19 → Invalid analysis  (D keeps dropping)
...
Iteration 5: P=45, D=8  → Both oscillations (D collapsed, P never moved)
Iteration 6: P=40, D=6  → System broken     (D too low, barely damping)
```

**Observation:** D kept getting reduced while P never increased. System got worse, not better.

---

## Proposed Redesign: Sequential Tuning Strategy

### Phase 1: Establish P/D Balance (Iterations 0-3)

**Goal:** Find the optimal P/D ratio for this frame

```c
// Target: Critically damped or slightly overdamped
// Metric: ~5-15% overshoot, no sustained oscillation

if (iteration == 0) {
    // Start with conservative ratio
    float ratio = 0.7f;  // D = 0.7 × P
    pidProfile->pid[axis].D = pidProfile->pid[axis].P * ratio;
}

// Adjust ratio based on response
if (gyroOscillationFreq > 0 || overshoot > 20.0f) {
    // Underdamped - need more D relative to P
    D_to_P_ratio *= 1.15f;  // Increase ratio
} else if (overshoot < 5.0f && riseTime > targetRiseTime * 1.2f) {
    // Overdamped - can reduce D relative to P
    D_to_P_ratio *= 0.90f;  // Decrease ratio
}

// Apply ratio (keep P constant during this phase)
pidProfile->pid[axis].D = pidProfile->pid[axis].P * D_to_P_ratio;
```

### Phase 2: Scale Up Gains (Iterations 4-8)

**Goal:** Push P and D together until hitting noise limit

```c
// Maintain the ratio established in Phase 1
// Increase both P and D proportionally

if (phase1Complete) {
    if (dtermOscillationFreq > 0) {
        // Hit noise limit - back off
        P *= 0.85f;
        D *= 0.85f;
        phase2Complete = true;
    } else if (!noiseLimited && overshoot < targetOvershoot + 5.0f) {
        // Still clean - push harder
        P *= 1.20f;  // Aggressive increase
        D *= 1.20f;  // Maintain ratio
    } else {
        // Approaching limit - increase cautiously
        P *= 1.10f;
        D *= 1.10f;
    }
}
```

### Phase 3: Fine-Tune (Iterations 9-12)

**Goal:** Back off slightly from noise limit

```c
if (phase2Complete) {
    if (dtermOscillationFreq > 15.0f) {
        // High-frequency noise - reduce D only
        D *= 0.90f;
    } else if (dtermOscillationFreq > 0) {
        // Moderate noise - reduce both slightly
        P *= 0.95f;
        D *= 0.95f;
    } else {
        // Tuning complete
        tuningComplete = true;
    }
}
```

### Phase 4: Feedforward (Future - M6)

**Goal:** Optimize stick response tracking

```c
// After PID is solid, add feedforward
// Measure stick-to-response lag
// Increase F until lag is minimized
```

---

## Proposed Code Changes

### New State Machine Phases

```c
typedef enum {
    TUNE_PHASE_ESTABLISH_RATIO = 0,  // Find optimal P/D balance (3-4 iterations)
    TUNE_PHASE_SCALE_UP = 1,         // Push gains up together (4-6 iterations)
    TUNE_PHASE_FINE_TUNE = 2,        // Back off from noise limit (2-3 iterations)
    TUNE_PHASE_COMPLETE = 3          // Done
} tunePhase_e;

typedef struct {
    tunePhase_e phase;
    uint8_t phaseIteration;
    float establishedRatio;          // D/P ratio from Phase 1
    float maxCleanP;                 // Highest P before noise
    float maxCleanD;                 // Highest D before noise
    bool noiseHit;                   // Hit noise limit in Phase 2
} tuneProgress_t;
```

### Revised Gain Adjustment Logic

```c
static void autotuneAdjustGains(void)
{
    const autotuneAnalysis_t *a = &runtime.analysis;
    tuneProgress_t *progress = &runtime.tuneProgress;
    
    if (!a->valid) {
        return;
    }
    
    pidProfile_t *pidProfile = currentPidProfile;
    const uint8_t axis = runtime.currentAxis;
    
    float P = pidProfile->pid[axis].P;
    float I = pidProfile->pid[axis].I;
    float D = pidProfile->pid[axis].D;
    
    const float targetOvershoot = autotuneConfig()->target_overshoot;
    const float targetRiseTime = autotuneConfig()->target_rise_time_ms * 1000.0f;  // Convert to µs
    
    // Store originals on first iteration
    if (runtime.iteration == 0) {
        runtime.originalP = P;
        runtime.originalI = I;
        runtime.originalD = D;
        progress->phase = TUNE_PHASE_ESTABLISH_RATIO;
        progress->establishedRatio = (D / P);  // Current ratio
    }
    
    switch (progress->phase) {
        case TUNE_PHASE_ESTABLISH_RATIO:
        {
            // Goal: Find P/D ratio that gives critical damping
            // Keep P constant, adjust D only
            
            if (a->gyroOscillationFreq > 0 || a->overshoot > 20.0f) {
                // Underdamped - need MORE D relative to P
                progress->establishedRatio *= 1.15f;
                D = P * progress->establishedRatio;
            } else if (a->overshoot < 5.0f && a->riseTime > targetRiseTime * 1.2f) {
                // Overdamped - can reduce D relative to P
                progress->establishedRatio *= 0.90f;
                D = P * progress->establishedRatio;
            } else {
                // Ratio is good - move to Phase 2
                progress->phase = TUNE_PHASE_SCALE_UP;
                progress->phaseIteration = 0;
                progress->maxCleanP = P;
                progress->maxCleanD = D;
            }
            
            progress->phaseIteration++;
            if (progress->phaseIteration >= 4) {
                // Force move to Phase 2 after 4 iterations
                progress->phase = TUNE_PHASE_SCALE_UP;
                progress->phaseIteration = 0;
            }
            break;
        }
        
        case TUNE_PHASE_SCALE_UP:
        {
            // Goal: Push P and D up together (maintaining ratio) until hitting noise limit
            
            if (a->dtermOscillationFreq > 0) {
                // Hit noise limit - back off and move to Phase 3
                P *= 0.85f;
                D *= 0.85f;
                progress->phase = TUNE_PHASE_FINE_TUNE;
                progress->phaseIteration = 0;
                progress->noiseHit = true;
            } else if (!a->noiseLimited && a->overshoot < targetOvershoot + 5.0f) {
                // Still clean - push aggressively
                progress->maxCleanP = P;
                progress->maxCleanD = D;
                P *= 1.20f;
                D *= 1.20f;
            } else {
                // Approaching limits - increase cautiously
                progress->maxCleanP = P;
                progress->maxCleanD = D;
                P *= 1.10f;
                D *= 1.10f;
            }
            
            progress->phaseIteration++;
            if (progress->phaseIteration >= 8) {
                // Max iterations - move to fine-tune
                progress->phase = TUNE_PHASE_FINE_TUNE;
                progress->phaseIteration = 0;
            }
            break;
        }
        
        case TUNE_PHASE_FINE_TUNE:
        {
            // Goal: Back off slightly from noise limit for clean operation
            
            if (a->dtermOscillationFreq > 15.0f) {
                // High-frequency noise - reduce D more
                D *= 0.85f;
            } else if (a->dtermOscillationFreq > 0) {
                // Moderate noise - small reduction
                D *= 0.95f;
            } else if (progress->noiseHit) {
                // Was noisy, now clean - done
                progress->phase = TUNE_PHASE_COMPLETE;
            } else {
                // Never hit noise - try pushing more
                P *= 1.05f;
                D *= 1.05f;
            }
            
            progress->phaseIteration++;
            if (progress->phaseIteration >= 3) {
                // Done with fine-tuning
                progress->phase = TUNE_PHASE_COMPLETE;
            }
            break;
        }
        
        case TUNE_PHASE_COMPLETE:
        {
            // Tuning complete for this axis
            // Move to next axis or finish
            runtime.axisComplete[axis] = true;
            break;
        }
    }
    
    // I-term: Scale proportionally with P
    I = runtime.originalI * (P / runtime.originalP);
    
    // Apply limits
    P = constrainf(P, 10, 250);
    I = constrainf(I, 10, 250);
    D = constrainf(D, 5, 100);  // Don't let D go below 5
    
    // Apply new gains
    pidProfile->pid[axis].P = lrintf(P);
    pidProfile->pid[axis].I = lrintf(I);
    pidProfile->pid[axis].D = lrintf(D);
    
    // Reinitialize PID
    pidInitConfig(currentPidProfile);
    
    // Debug output
    DEBUG_SET(DEBUG_AUTOTUNE, 5, progress->phase);
    DEBUG_SET(DEBUG_AUTOTUNE, 6, progress->phaseIteration);
}
```

---

## Expected Outcomes

### With Current Approach
- Takes 10-20+ iterations per axis
- P often gets stuck, D collapses
- May not converge at all
- Final tune may be worse than starting

### With Sequential Approach
- Phase 1: 3-4 iterations to establish ratio
- Phase 2: 4-6 iterations to scale up
- Phase 3: 2-3 iterations to fine-tune
- **Total: 9-13 iterations** (~10-15 seconds per axis)
- Predictable, reliable convergence
- Always produces usable result

---

## Additional Improvements Needed

### 1. Better Invalid Iteration Detection
```c
static bool isValidAnalysis(const autotuneAnalysis_t *a)
{
    // Reject clearly bad data
    if (a->riseTime < 0 || a->riseTime > 200000.0f) return false;  // <0 or >200ms
    if (a->overshoot < -10.0f || a->overshoot > 300.0f) return false;  // Crazy values
    if (a->gyroNoiseRms > 50.0f) return false;  // Extremely noisy
    
    // Check for pilot interference (large stick input during test)
    // TODO: Monitor stick inputs, reject if deflection > 10% during excitation
    
    return true;
}
```

### 2. Stick Interference Detection
```c
// In EXCITE state, monitor RC command
static bool detectPilotInterference(void)
{
    const uint8_t axis = runtime.currentAxis;
    const float rcCommand = getRcDeflection(axis);
    
    // If pilot moves stick > 10% during test, abort iteration
    if (fabsf(rcCommand) > 0.10f) {
        return true;
    }
    
    return false;
}
```

### 3. Convergence Detection
```c
static bool checkConvergence(void)
{
    if (runtime.iteration < 3) return false;  // Need at least 3 iterations
    
    // Check if gains have stabilized (< 5% change for 2 iterations)
    const uint8_t axis = runtime.currentAxis;
    const pidProfile_t *pidProfile = currentPidProfile;
    
    float currentP = pidProfile->pid[axis].P;
    float currentD = pidProfile->pid[axis].D;
    
    // Compare to 2 iterations ago
    float prevP = runtime.history[runtime.iteration - 2].P;
    float prevD = runtime.history[runtime.iteration - 2].D;
    
    float pChange = fabsf((currentP - prevP) / prevP);
    float dChange = fabsf((currentD - prevD) / prevD);
    
    if (pChange < 0.05f && dChange < 0.05f) {
        return true;  // Converged
    }
    
    return false;
}
```

---

## Recommendation

**Implement the sequential 3-phase approach** as outlined above. This matches proven manual tuning methodology and will:

1. ✅ Converge predictably in 9-13 iterations
2. ✅ Establish proper P/D balance first
3. ✅ Push gains aggressively when safe
4. ✅ Back off gracefully when hitting limits
5. ✅ Produce consistent, reliable results

The current priority-based approach is trying to be too clever and ends up fighting itself. Your instinct is correct - **we need a more procedural, sequential approach** that builds upon each phase.

---

## Next Steps

1. Fix overshoot calculation (baseline averaging, proper direction handling)
2. Implement 3-phase tuning strategy
3. Add invalid iteration filtering
4. Add stick interference detection
5. Test on hardware with blackbox logging
6. Iterate on phase thresholds based on real data

This should get us to a tuning system that actually works reliably and produces good results in reasonable time.
