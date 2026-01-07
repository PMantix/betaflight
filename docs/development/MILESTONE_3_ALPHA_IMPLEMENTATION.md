# Milestone 3 Alpha: Sequential Tuning Implementation Plan

**Target:** Complete redesign with 3-phase sequential approach  
**Status:** 🔄 PLANNING  
**Date:** January 5, 2026

---

## Overview

Implement a sequential, phase-based tuning approach that matches proven manual tuning methodology:

1. **Phase 1: Establish P/D Balance** - Find optimal ratio at conservative gain levels
2. **Phase 2: Scale Up Gains** - Push both P and D together until hitting limits
3. **Phase 3: Fine-Tune** - Back off slightly for clean operation

---

## Phase 1: Establish P/D Balance (Conservative Start)

### Objective

Find the **optimal P/D ratio** that produces critical damping, using **intentionally low starting gains** to ensure safety.

### Key Insight: Start Conservative

**Your suggestion is critical for safety:**
- Start with P and D values that are **deliberately low** (70% of expected final values)
- This guarantees we won't be too aggressive while finding the ratio
- Once ratio is established, we can safely scale up in Phase 2
- Better to start soft and ramp up than start aggressive and cause crashes

### Critical Damping Criteria

**Critical damping** is achieved when:
1. Minimal overshoot (5-15% target)
2. No sustained oscillation (gyro oscillation frequency = 0 Hz). Should overshoot every so slightly, but not oscillate. 
3. Fast settling time
4. Good step tracking (gyro reaches setpoint). This might require I tuning, so mainly important that it stabilizes

**Metrics to evaluate:**
```c
// Ideal critically damped response:
overshoot:              5-15%  (sweet spot ~10%)
gyroOscillationFreq:    0 Hz   (no ringing)
riseTime:               40-80ms (reasonable speed at low gains)
settlingTime:           <150ms (quick stabilization)
```

**Overdamped indicators** (D too high relative to P):
```c
overshoot:              <5%    (too sluggish, or even no overshoot!)
riseTime:               >100ms (very slow)
gyroResponse:           Lags behind setpoint significantly
```

**Underdamped indicators** (D too low relative to P):
```c
overshoot:              >20%   (excessive)
gyroOscillationFreq:    >0 Hz  (ringing present)
settlingTime:           >200ms (oscillating for too long)
```

### Starting Point Strategy

```c
// Configuration parameter (CLI settable)
typedef struct {
    uint8_t phase1_p_start_percent;  // Default: 70 (70% of current P)
    uint8_t phase1_d_start_percent;  // Default: 70 (70% of current D)
    uint8_t phase1_ratio_start;      // Default: 70 (D = 0.70 × P initially)
} autotunePhase1Config_t;
```

**Rationale:**
- Starting at 70% of current PIDs ensures conservative beginning while being aggressive enough
- For a stock quad with P=45, D=30, this gives P=31, D=21 as starting point
- At these gains, even if ratio is wrong, quad remains stable and responsive
- Ratio of 0.70 is a reasonable starting assumption (typical range is 0.6-0.8)

### Algorithm: Phase 1

```c
static void phase1_establishRatio(void)
{
    tuneProgress_t *progress = &runtime.tuneProgress;
    const autotuneAnalysis_t *a = &runtime.analysis;
    pidProfile_t *pidProfile = currentPidProfile;
    const uint8_t axis = runtime.currentAxis;
    
    // First iteration: Set conservative starting point
    if (progress->phaseIteration == 0) {
        // Store absolute original gains
        runtime.originalP = pidProfile->pid[axis].P;
        runtime.originalD = pidProfile->pid[axis].D;
        runtime.originalI = pidProfile->pid[axis].I;
        
        // Calculate conservative starting gains (70% of original)
        float startP = runtime.originalP * 0.70f;
        float startD = startP * 0.70f;  // Initial ratio assumption
        float startI = runtime.originalI * 0.70f;
        
        // Apply starting gains
        pidProfile->pid[axis].P = lrintf(startP);
        pidProfile->pid[axis].D = lrintf(startD);
        pidProfile->pid[axis].I = lrintf(startI);
        
        // Store the ratio
        progress->establishedRatio = startD / startP;
        progress->phase1StartP = startP;
        
        pidInitConfig(currentPidProfile);
        return;
    }
    
    // Subsequent iterations: Adjust ratio based on response
    float currentP = pidProfile->pid[axis].P;
    float currentD = pidProfile->pid[axis].D;
    float currentRatio = progress->establishedRatio;
    
    // Evaluate damping state
    bool underdamped = (a->gyroOscillationFreq > 0) || (a->overshoot > 20.0f);
    bool overdamped = (a->overshoot < 5.0f) && (a->riseTime > 80000.0f); // >80ms
    bool criticallyDamped = !underdamped && !overdamped;
    
    if (underdamped) {
        // Need MORE damping (increase D relative to P)
        currentRatio *= 1.15f;  // Increase D/P ratio by 15%
        currentD = currentP * currentRatio;
        
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 1);  // Underdamped flag
    } 
    else if (overdamped) {
        // Too much damping (decrease D relative to P)
        currentRatio *= 0.90f;  // Decrease D/P ratio by 10%
        currentD = currentP * currentRatio;
        
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 2);  // Overdamped flag
    } 
    else {
        // Critically damped - ratio is good!
        progress->phase = TUNE_PHASE_SCALE_UP;
        progress->phaseIteration = 0;
        progress->establishedRatio = currentRatio;
        
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 3);  // Critical damping achieved
        beeper(BEEPER_READY_BEEP);  // Signal phase transition
        return;
    }
    
    // Apply new D (keep P constant during ratio finding)
    pidProfile->pid[axis].D = lrintf(currentD);
    progress->establishedRatio = currentRatio;
    
    pidInitConfig(currentPidProfile);
    
    // Safety limit: Don't let ratio go too extreme
    if (currentRatio > 1.0f) {
        currentRatio = 1.0f;  // D should never exceed P
        progress->establishedRatio = currentRatio;
    }
    if (currentRatio < 0.4f) {
        currentRatio = 0.4f;  // D shouldn't be too low
        progress->establishedRatio = currentRatio;
    }
    
    // Maximum iterations for Phase 1
    progress->phaseIteration++;
    if (progress->phaseIteration >= 5) {
        // Force transition after 5 iterations (good enough)
        progress->phase = TUNE_PHASE_SCALE_UP;
        progress->phaseIteration = 0;
        
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 4);  // Max iterations reached
    }
}
```

### Expected Behavior

**Typical Phase 1 Sequence:**

```
Iteration 0: Set starting point
  P=31 (70% of 45), D=21 (ratio 0.70), I=21
  → Test response

Iteration 1: Evaluate response
  Analysis: Overshoot 25%, Gyro osc 8Hz → Underdamped
  Action: Increase ratio 0.70 → 0.80
  P=31 (unchanged), D=25 (ratio 0.80), I=21
  → Test response

Iteration 2: Evaluate response
  Analysis: Overshoot 12%, Gyro osc 0Hz → Slightly underdamped
  Action: Increase ratio 0.80 → 0.92
  P=31 (unchanged), D=28 (ratio 0.92), I=21
  → Test response

Iteration 3: Evaluate response
  Analysis: Overshoot 8%, Gyro osc 0Hz → Critically damped!
  Action: Lock ratio at 0.92, move to Phase 2
  → Phase 1 complete in 3-4 iterations
```

### Configuration Parameters

```c
// In pg/autotune.h
typedef struct autotuneConfig_s {
    // Existing...
    uint8_t step_amplitude;
    uint16_t step_duration_ms;
    uint8_t target_overshoot;
    
    // NEW: Phase 1 parameters
    uint8_t phase1_start_p_percent;      // Default: 50 (50% of current)
    uint8_t phase1_start_d_percent;      // Default: 50 (50% of current)
    uint8_t phase1_initial_ratio;        // Default: 70 (0.70 × 100)
    uint8_t phase1_target_overshoot_min; // Default: 5 (5%)
    uint8_t phase1_target_overshoot_max; // Default: 15 (15%)
    uint8_t phase1_max_iterations;       // Default: 5
    
} autotuneConfig_t;
```

### CLI Commands

```
# Configure Phase 1 behavior
set autotune_phase1_start_p_percent = 70    # Start at 70% of current P
set autotune_phase1_start_d_percent = 70    # Start at 70% of current D
set autotune_phase1_initial_ratio = 70      # Initial D/P ratio (0.70)
set autotune_phase1_target_overshoot_min = 5
set autotune_phase1_target_overshoot_max = 15
set autotune_phase1_max_iterations = 5
```

### Success Criteria for Phase 1

✅ Phase 1 is complete when:
1. Overshoot is within target range (5-15%)
2. No gyro oscillation detected (gyroOscillationFreq = 0 Hz)
3. Rise time is reasonable (<100ms at conservative gains)

OR:

4. Maximum iterations reached (5 iterations)

At this point, we have a **safe P/D ratio** that can be scaled up aggressively.

---

## Phase 2: Scale Up Gains

### Objective

Push **both P and D together** (maintaining the ratio from Phase 1) until hitting noise or overshoot limits.

### Strategy

**Key principle:** Now that we know the correct damping ratio, we can scale both gains up proportionally without changing the fundamental response character.

```c
static void phase2_scaleUp(void)
{
    tuneProgress_t *progress = &runtime.tuneProgress;
    const autotuneAnalysis_t *a = &runtime.analysis;
    pidProfile_t *pidProfile = currentPidProfile;
    const uint8_t axis = runtime.currentAxis;
    
    float P = pidProfile->pid[axis].P;
    float D = pidProfile->pid[axis].D;
    float I = pidProfile->pid[axis].I;
    
    // Check if we've hit a limit
    bool hitNoiseLimit = (a->dtermOscillationFreq > 0);
    bool approachingOvershoot = (a->overshoot > autotuneConfig()->target_overshoot + 5.0f);
    bool noiseLimited = a->noiseLimited;
    
    if (hitNoiseLimit) {
        // Hit D-term noise - back off and complete
        P *= 0.85f;
        D *= 0.85f;
        I *= 0.85f;
        
        progress->phase = TUNE_PHASE_FINE_TUNE;
        progress->phaseIteration = 0;
        progress->noiseHitIteration = runtime.iteration;
        
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 10);  // Noise limit hit
        beeper(BEEPER_READY_BEEP);
    }
    else if (approachingOvershoot || noiseLimited) {
        // Approaching limits - increase cautiously
        P *= 1.10f;
        D *= 1.10f;
        I *= 1.10f;
        
        progress->maxCleanP = P;
        progress->maxCleanD = D;
        
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 11);  // Cautious increase
    }
    else {
        // Still clean - push hard!
        P *= 1.25f;  // Aggressive 25% increase
        D *= 1.25f;
        I *= 1.25f;
        
        progress->maxCleanP = P;
        progress->maxCleanD = D;
        
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 12);  // Aggressive increase
    }
    
    // Apply new gains
    pidProfile->pid[axis].P = lrintf(constrainf(P, 10, 250));
    pidProfile->pid[axis].D = lrintf(constrainf(D, 5, 100));
    pidProfile->pid[axis].I = lrintf(constrainf(I, 10, 250));
    
    pidInitConfig(currentPidProfile);
    
    // Maximum iterations safety limit
    progress->phaseIteration++;
    if (progress->phaseIteration >= 8) {
        progress->phase = TUNE_PHASE_FINE_TUNE;
        progress->phaseIteration = 0;
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 13);  // Max iterations
    }
}
```

### Expected Behavior

**Typical Phase 2 Sequence:**

```
Iteration 4 (Phase 2 start): 
  P=31, D=28, ratio=0.92 (from Phase 1)
  Analysis: Clean, overshoot 9%
  Action: Aggressive scale 1.25×
  → P=39, D=35, I=26

Iteration 5:
  Analysis: Clean, overshoot 11%
  Action: Aggressive scale 1.25×
  → P=49, D=44, I=33

Iteration 6:
  Analysis: Clean, overshoot 16%, approaching limit
  Action: Cautious scale 1.10×
  → P=54, D=48, I=36

Iteration 7:
  Analysis: D-term oscillation 35Hz detected!
  Action: Back off 0.85×, move to Phase 3
  → P=46, D=41, I=31
```

---

## Phase 3: Fine-Tune

### Objective

Make small adjustments to back off from noise limit and ensure clean operation.

```c
static void phase3_fineTune(void)
{
    tuneProgress_t *progress = &runtime.tuneProgress;
    const autotuneAnalysis_t *a = &runtime.analysis;
    pidProfile_t *pidProfile = currentPidProfile;
    const uint8_t axis = runtime.currentAxis;
    
    float P = pidProfile->pid[axis].P;
    float D = pidProfile->pid[axis].D;
    float I = pidProfile->pid[axis].I;
    
    if (a->dtermOscillationFreq > 15.0f) {
        // High-frequency noise - reduce D more aggressively
        D *= 0.85f;
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 20);
    }
    else if (a->dtermOscillationFreq > 0) {
        // Moderate noise - small D reduction
        D *= 0.95f;
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 21);
    }
    else if (progress->noiseHit && a->dtermOscillationFreq == 0) {
        // Was noisy, now clean - done!
        progress->phase = TUNE_PHASE_COMPLETE;
        progress->phaseIteration = 0;
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 22);
        beeper(BEEPER_AUTOTUNE_DONE);
        return;
    }
    else {
        // Never hit noise - try one more push
        P *= 1.05f;
        D *= 1.05f;
        I *= 1.05f;
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 23);
    }
    
    // Apply gains
    pidProfile->pid[axis].P = lrintf(constrainf(P, 10, 250));
    pidProfile->pid[axis].D = lrintf(constrainf(D, 5, 100));
    pidProfile->pid[axis].I = lrintf(constrainf(I, 10, 250));
    
    pidInitConfig(currentPidProfile);
    
    // Phase 3 is short - max 3 iterations
    progress->phaseIteration++;
    if (progress->phaseIteration >= 3) {
        progress->phase = TUNE_PHASE_COMPLETE;
        progress->phaseIteration = 0;
        beeper(BEEPER_AUTOTUNE_DONE);
    }
}
```

---

## Task Breakdown

### Task 1: Fix Overshoot Calculation
**Priority:** 🔴 CRITICAL  
**Effort:** 2 hours  
**File:** `src/main/flight/autotune.c`

**Changes:**
1. Add baseline averaging (5 samples before step)
2. Fix direction handling for negative steps
3. Add reverse overshoot detection
4. Improve magnitude scaling

**Code location:** `calculateOvershoot()` function (~line 365)

**Test criteria:**
- Bidirectional doublet overshoot readings are consistent
- No more 300% overshoot false readings
- Values match manual blackbox analysis

---

### Task 2: Add Phase State Machine
**Priority:** 🔴 CRITICAL  
**Effort:** 3 hours  
**File:** `src/main/flight/autotune.c`, `src/main/flight/autotune.h`

**Changes:**

1. **Add new types:**
```c
typedef enum {
    TUNE_PHASE_ESTABLISH_RATIO = 0,
    TUNE_PHASE_SCALE_UP = 1,
    TUNE_PHASE_FINE_TUNE = 2,
    TUNE_PHASE_COMPLETE = 3
} tunePhase_e;

typedef struct {
    tunePhase_e phase;
    uint8_t phaseIteration;
    float establishedRatio;
    float phase1StartP;
    float phase1StartD;
    float maxCleanP;
    float maxCleanD;
    uint8_t noiseHitIteration;
    bool noiseHit;
} tuneProgress_t;
```

2. **Add to runtime structure:**
```c
static struct {
    // Existing fields...
    
    // NEW: Phase tracking
    tuneProgress_t tuneProgress;
    
} runtime = {
    .active = false,
    .state = AUTOTUNE_STATE_IDLE,
    .tuneProgress.phase = TUNE_PHASE_ESTABLISH_RATIO,
};
```

3. **Initialize in autotuneInit():**
```c
void autotuneInit(void)
{
    memset(&runtime, 0, sizeof(runtime));
    runtime.state = AUTOTUNE_STATE_IDLE;
    runtime.tuneProgress.phase = TUNE_PHASE_ESTABLISH_RATIO;
    runtime.tuneProgress.establishedRatio = 0.70f;  // Default starting ratio
}
```

**Test criteria:**
- State machine compiles without errors
- Debug channels show phase transitions
- Phase progression is logical

---

### Task 3: Implement Phase 1 Logic
**Priority:** 🔴 CRITICAL  
**Effort:** 4 hours  
**File:** `src/main/flight/autotune.c`

**Changes:**

1. Create `phase1_establishRatio()` function (see detailed code above)
2. Modify `autotuneAdjustGains()` to dispatch to phase functions
3. Add conservative starting point calculation
4. Implement damping evaluation logic
5. Add ratio adjustment rules

**Code structure:**
```c
static void autotuneAdjustGains(void)
{
    const autotuneAnalysis_t *a = &runtime.analysis;
    tuneProgress_t *progress = &runtime.tuneProgress;
    
    if (!a->valid) return;
    
    // Dispatch to phase-specific handler
    switch (progress->phase) {
        case TUNE_PHASE_ESTABLISH_RATIO:
            phase1_establishRatio();
            break;
        case TUNE_PHASE_SCALE_UP:
            phase2_scaleUp();
            break;
        case TUNE_PHASE_FINE_TUNE:
            phase3_fineTune();
            break;
        case TUNE_PHASE_COMPLETE:
            // Axis complete
            break;
    }
    
    runtime.iteration++;
}
```

**Test criteria:**
- Phase 1 finds reasonable ratio (0.6-0.9 range)
- Completes in 3-5 iterations
- P remains constant while D adjusts
- Transitions to Phase 2 when critically damped

---

### Task 4: Implement Phase 2 Logic
**Priority:** 🔴 CRITICAL  
**Effort:** 3 hours  
**File:** `src/main/flight/autotune.c`

**Changes:**

1. Create `phase2_scaleUp()` function (see code above)
2. Implement proportional scaling (maintain ratio)
3. Add aggressive vs cautious increase logic
4. Detect noise limit hit
5. Track maximum clean gains

**Test criteria:**
- P and D increase together maintaining ratio
- Aggressive scaling (1.25×) when clean
- Cautious scaling (1.10×) near limits
- Properly detects D-term noise and transitions

---

### Task 5: Implement Phase 3 Logic
**Priority:** 🟡 HIGH  
**Effort:** 2 hours  
**File:** `src/main/flight/autotune.c`

**Changes:**

1. Create `phase3_fineTune()` function (see code above)
2. Implement D-term noise response
3. Add completion detection
4. Finalize gains

**Test criteria:**
- Backs off from noise appropriately
- Completes in 1-3 iterations
- Produces clean final gains
- Signals completion properly

---

### Task 6: Add Configuration Parameters
**Priority:** 🟡 HIGH  
**Effort:** 2 hours  
**Files:** `src/main/pg/autotune.h`, `src/main/pg/autotune.c`, `src/main/cli/settings.c`

**Changes:**

1. **Add to autotuneConfig_t:**
```c
typedef struct autotuneConfig_s {
    // Existing...
    uint8_t autotune_enabled;
    uint8_t step_amplitude;
    uint16_t step_duration_ms;
    uint8_t target_overshoot;
    
    // NEW: Phase 1 configuration
    uint8_t phase1_start_p_percent;
    uint8_t phase1_start_d_percent;
    uint8_t phase1_initial_ratio;
    uint8_t phase1_target_overshoot_min;
    uint8_t phase1_target_overshoot_max;
    uint8_t phase1_max_iterations;
    
    // NEW: Phase 2 configuration
    uint8_t phase2_aggressive_multiplier;  // Default: 125 (1.25×)
    uint8_t phase2_cautious_multiplier;    // Default: 110 (1.10×)
    uint8_t phase2_max_iterations;         // Default: 8
    
    // NEW: Phase 3 configuration
    uint8_t phase3_max_iterations;         // Default: 3
    
} autotuneConfig_t;

PG_DECLARE(autotuneConfig_t, autotuneConfig);
```

2. **Add defaults in autotune.c:**
```c
PG_RESET_TEMPLATE(autotuneConfig_t, autotuneConfig,
    // Existing...
    .autotune_enabled = 1,
    .step_amplitude = 100,
    .step_duration_ms = 100,
    .target_overshoot = 10,
    
    // NEW: Phase 1 defaults
    .phase1_start_p_percent = 70,
    .phase1_start_d_percent = 70,
    .phase1_initial_ratio = 70,
    .phase1_target_overshoot_min = 5,
    .phase1_target_overshoot_max = 15,
    .phase1_max_iterations = 5,
    
    // NEW: Phase 2 defaults
    .phase2_aggressive_multiplier = 125,
    .phase2_cautious_multiplier = 110,
    .phase2_max_iterations = 8,
    
    // NEW: Phase 3 defaults
    .phase3_max_iterations = 3,
);
```

3. **Add CLI settings:**
```c
// In settings.c
{ "autotune_phase1_start_p_percent",  VAR_UINT8 | MASTER_VALUE, 
  .config.minmaxUnsigned = { 50, 90 }, 
  PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, phase1_start_p_percent) },
  
{ "autotune_phase1_start_d_percent",  VAR_UINT8 | MASTER_VALUE,
  .config.minmaxUnsigned = { 50, 90 },
  PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, phase1_start_d_percent) },
  
{ "autotune_phase1_initial_ratio",  VAR_UINT8 | MASTER_VALUE,
  .config.minmaxUnsigned = { 40, 100 },
  PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, phase1_initial_ratio) },
  
// ... etc for all new parameters
```

**Test criteria:**
- All parameters accessible via CLI
- Defaults produce sensible behavior
- Parameter ranges prevent dangerous values

---

### Task 7: Enhanced Debug Output
**Priority:** 🟡 HIGH  
**Effort:** 1 hour  
**File:** `src/main/flight/autotune.c`

**Changes:**

Update debug channel assignments for better visibility:

```c
// In ANALYZE state:
DEBUG_SET(DEBUG_AUTOTUNE, 0, runtime.state);
DEBUG_SET(DEBUG_AUTOTUNE, 1, runtime.iteration);
DEBUG_SET(DEBUG_AUTOTUNE, 2, (int16_t)(a->riseTime / 1000));  // ms
DEBUG_SET(DEBUG_AUTOTUNE, 3, (int16_t)a->overshoot);          // %
DEBUG_SET(DEBUG_AUTOTUNE, 4, (int16_t)a->dtermOscillationFreq); // Hz
DEBUG_SET(DEBUG_AUTOTUNE, 5, (int16_t)a->gyroOscillationFreq);  // Hz
DEBUG_SET(DEBUG_AUTOTUNE, 6, progress->phase);                // 0-3
DEBUG_SET(DEBUG_AUTOTUNE, 7, progress->phaseIteration);       // Iteration within phase

// In ADJUST state:
DEBUG_SET(DEBUG_AUTOTUNE, 0, runtime.state);
DEBUG_SET(DEBUG_AUTOTUNE, 1, (int16_t)runtime.originalP);
DEBUG_SET(DEBUG_AUTOTUNE, 2, (int16_t)pidProfile->pid[axis].P);  // New P
DEBUG_SET(DEBUG_AUTOTUNE, 3, (int16_t)runtime.originalD);
DEBUG_SET(DEBUG_AUTOTUNE, 4, (int16_t)pidProfile->pid[axis].D);  // New D
DEBUG_SET(DEBUG_AUTOTUNE, 5, (int16_t)(progress->establishedRatio * 100)); // Ratio × 100
DEBUG_SET(DEBUG_AUTOTUNE, 6, progress->phase);
DEBUG_SET(DEBUG_AUTOTUNE, 7, progress->phaseIteration);
```

**Test criteria:**
- Debug channels clearly show phase progression
- Gain changes are visible
- Ratio is trackable

---

### Task 8: Invalid Iteration Filtering
**Priority:** 🟡 HIGH  
**Effort:** 2 hours  
**File:** `src/main/flight/autotune.c`

**Changes:**

1. **Add validation function:**
```c
static bool isValidAnalysis(const autotuneAnalysis_t *a)
{
    // Reject negative or crazy rise times
    if (a->riseTime < 0 || a->riseTime > 200000.0f) {
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 100);  // Invalid rise time
        return false;
    }
    
    // Reject extreme overshoot values
    if (a->overshoot < -10.0f || a->overshoot > 300.0f) {
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 101);  // Invalid overshoot
        return false;
    }
    
    // Reject extremely noisy data
    if (a->gyroNoiseRms > 50.0f) {
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 102);  // Excessive noise
        return false;
    }
    
    return true;
}
```

2. **Use in ANALYZE state:**
```c
case AUTOTUNE_STATE_ANALYZE:
    if (runtime.stateJustEntered) {
        runtime.stateJustEntered = false;
        autotuneAnalyzeResponse();
        
        // Validate analysis before using
        if (!isValidAnalysis(&runtime.analysis)) {
            // Bad data - retry this iteration
            beeper(BEEPER_DISARM_REPEAT);
            changeState(AUTOTUNE_STATE_IDLE_WAIT, currentTimeUs);
            break;
        }
        
        changeState(AUTOTUNE_STATE_ADJUST, currentTimeUs);
    }
    break;
```

**Test criteria:**
- Invalid iterations are detected and rejected
- Beeper sounds on rejection
- State machine retries (returns to IDLE_WAIT)
- Good iterations proceed normally

---

### Task 9: Stick Interference Detection
**Priority:** 🟢 MEDIUM  
**Effort:** 2 hours  
**File:** `src/main/flight/autotune.c`

**Changes:**

1. **Add detection function:**
```c
static bool detectStickInterference(void)
{
    // Check RC command during excitation
    // If pilot moves stick > 10%, abort iteration
    
    const uint8_t axis = runtime.currentAxis;
    
    // Get RC deflection (0.0 to 1.0)
    float rcDeflection = 0.0f;
    
    switch (axis) {
        case FD_ROLL:
            rcDeflection = fabsf(rcCommand[ROLL]) / 500.0f;
            break;
        case FD_PITCH:
            rcDeflection = fabsf(rcCommand[PITCH]) / 500.0f;
            break;
        case FD_YAW:
            rcDeflection = fabsf(rcCommand[YAW]) / 500.0f;
            break;
    }
    
    // Threshold: 10% stick deflection
    if (rcDeflection > 0.10f) {
        DEBUG_SET(DEBUG_AUTOTUNE, 6, 103);  // Stick interference
        return true;
    }
    
    return false;
}
```

2. **Check during EXCITE state:**
```c
case AUTOTUNE_STATE_EXCITE:
    // Check for pilot interference
    if (detectStickInterference()) {
        // Abort this iteration
        beeper(BEEPER_DISARM_REPEAT);
        changeState(AUTOTUNE_STATE_IDLE_WAIT, currentTimeUs);
        break;
    }
    
    // Continue with excitation...
```

**Test criteria:**
- Stick input during test aborts iteration
- Beeper provides feedback
- System returns to stable hover
- Next iteration starts cleanly

---

### Task 10: Convergence Detection
**Priority:** 🟢 MEDIUM  
**Effort:** 2 hours  
**File:** `src/main/flight/autotune.c`

**Changes:**

1. **Add history tracking:**
```c
typedef struct {
    float P;
    float D;
    uint8_t phase;
} iterationHistory_t;

static struct {
    // Existing...
    iterationHistory_t history[20];  // Last 20 iterations
} runtime;
```

2. **Add convergence check:**
```c
static bool checkConvergence(void)
{
    if (runtime.iteration < 3) return false;
    
    const uint8_t axis = runtime.currentAxis;
    pidProfile_t *pidProfile = currentPidProfile;
    
    float currentP = pidProfile->pid[axis].P;
    float currentD = pidProfile->pid[axis].D;
    
    // Look back 2 iterations
    float prevP = runtime.history[runtime.iteration - 2].P;
    float prevD = runtime.history[runtime.iteration - 2].D;
    
    float pChange = fabsf((currentP - prevP) / prevP);
    float dChange = fabsf((currentD - prevD) / prevD);
    
    // Converged if changes < 5% for 2 consecutive iterations
    if (pChange < 0.05f && dChange < 0.05f) {
        return true;
    }
    
    return false;
}
```

3. **Use in ADJUST state:**
```c
// After applying new gains
if (checkConvergence()) {
    runtime.tuneProgress.phase = TUNE_PHASE_COMPLETE;
    beeper(BEEPER_AUTOTUNE_DONE);
}
```

**Test criteria:**
- Detects when gains stabilize
- Avoids infinite iteration loops
- Completes earlier when converged

---

## Testing Strategy

### Unit Testing

For each phase function, test with mock analysis data:

```c
// Test Phase 1: Underdamped response
autotuneAnalysis_t testUnderdamped = {
    .overshoot = 25.0f,
    .gyroOscillationFreq = 8.0f,
    .riseTime = 50000.0f,
    .valid = true
};
// Expected: Ratio increases

// Test Phase 1: Overdamped response
autotuneAnalysis_t testOverdamped = {
    .overshoot = 3.0f,
    .gyroOscillationFreq = 0.0f,
    .riseTime = 120000.0f,
    .valid = true
};
// Expected: Ratio decreases

// Test Phase 1: Critical damping
autotuneAnalysis_t testCritical = {
    .overshoot = 10.0f,
    .gyroOscillationFreq = 0.0f,
    .riseTime = 60000.0f,
    .valid = true
};
// Expected: Transition to Phase 2
```

### Bench Testing

1. **Flash firmware with new code**
2. **Enable autotune debug mode:** `set debug_mode = AUTOTUNE`
3. **Configure blackbox:** High logging rate
4. **Tethered test:** Secure quad, activate autotune
5. **Monitor debug channels in real-time**
6. **Verify phase progressions**

### Flight Testing

**Test Plan:**

1. **Baseline tune:** Start with known-good PIDs
2. **Activate autotune on roll axis only**
3. **Log full flight to blackbox**
4. **Analyze results:**
   - Phase 1 convergence (3-5 iterations)
   - Phase 2 scaling behavior
   - Phase 3 final tuning
   - Total time to completion
5. **Compare final PIDs to baseline**
6. **Fly with new PIDs, evaluate feel**

**Success Criteria:**
- ✅ Completes in 10-15 iterations (~12-18 seconds)
- ✅ Phase 1 finds ratio in 3-5 iterations
- ✅ Phase 2 scales aggressively when clean
- ✅ Phase 3 produces clean, noise-free gains
- ✅ Final PIDs are flyable (no crashes!)
- ✅ Response feels crisp and well-damped

---

## Timeline

**Week 1: Core Implementation**
- Day 1-2: Tasks 1-2 (Overshoot fix + State machine)
- Day 3-4: Task 3 (Phase 1 implementation)
- Day 5: Task 4 (Phase 2 implementation)

**Week 2: Refinement & Testing**
- Day 1: Task 5 (Phase 3 implementation)
- Day 2: Task 6 (Configuration parameters)
- Day 3: Tasks 7-8 (Debug output + validation)
- Day 4: Tasks 9-10 (Interference detection + convergence)
- Day 5: Bench testing & debugging

**Week 3: Flight Testing**
- Day 1-2: Initial flight tests, data collection
- Day 3-4: Refinement based on flight data
- Day 5: Final validation flights

---

## Risk Mitigation

### Risk 1: Phase 1 doesn't converge
**Mitigation:** 
- Maximum iteration limit (5 iterations)
- Force transition even if not perfect
- Wider acceptable overshoot range (5-20%)

### Risk 2: Phase 2 overshoots and causes crash
**Mitigation:**
- Hard PID limits (P<250, D<100)
- Safety checks in every state
- Pilot can disable autotune instantly

### Risk 3: Different frames need different starting points
**Mitigation:**
- Configurable starting percentages (50-90%)
- CLI parameters for frame-specific tuning
- Conservative defaults (70%)
**Mitigation:**
- Sequential approach avoids this
- Each phase has clear objective
- Phase 1 ratio is frame-dependent, Phase 2 finds limit

---

## Future Enhancements (Post-Alpha)

1. **Adaptive starting point:** Use frame size/weight to guess better starting PIDs
2. **Multi-axis optimization:** Tune all axes together, account for coupling
3. **Feedforward integration:** Add Phase 4 for F-term tuning
4. **Filter optimization:** Use Phase 2 noise data to suggest filter changes
5. **OSD feedback:** Display progress on OSD
6. **Persistent learning:** Remember optimal ratios for different frames

---

## Conclusion

This implementation plan provides a **safe, sequential, predictable** approach to autotuning that:

✅ Starts conservatively (70% of current PIDs)  
✅ Finds optimal P/D balance first (Phase 1)  
✅ Scales aggressively when safe (Phase 2)  
✅ Fine-tunes for clean operation (Phase 3)  
✅ Completes in 10-15 iterations (~15 seconds)  
✅ Produces reliable, flyable results  

The key innovation is **separating ratio-finding from gain-scaling**, which mirrors proven manual tuning methodology and avoids the convergence problems of the current simultaneous approach.
