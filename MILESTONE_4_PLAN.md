# Milestone 4 Plan: Calculated Gain Adjustments with Newton's Method

## Overview

Replace fixed-increment gain adjustments with calculated adjustments based on:
1. **Sensitivity estimation** - Use diagnostic test results to estimate how much a parameter affects the metric
2. **Proportional correction** - Calculate adjustment size based on error magnitude and sensitivity
3. **Newton's method** - After several iterations, use gradient-based optimization for fast convergence
4. **Damping factor** - Apply only 90% of calculated change for stability
5. **Sequential multi-variable** - Adjust one parameter at a time, verify each, wiggle when all complete

## Key Design Decisions

1. **Separate history per axis** - Roll, Pitch, Yaw each have independent histories
2. **90% damping factor** - Never apply more than 90% of calculated change (conservative)
3. **Term-specific metrics**:
   - **P term**: Setpoint tracking / responsiveness
   - **D term**: Oscillation detection (motor noise)
   - **I term**: Long-term error / drift
   - **F term**: Setpoint tracking during rapid stick movements
4. **Sequential adjustment** - When multiple parameters need changes, apply one at a time, verify each didn't break the tune, then wiggle when all are complete

## Current Approach (Problems)

```
Current: gain += FIXED_STEP  (e.g., ±5 for PID, ±10Hz for filter)
```

**Issues:**
- Slow convergence - many iterations for large errors
- Can overshoot - steps past optimal point
- No learning - doesn't use information from previous attempts
- One-size-fits-all - same step regardless of error magnitude

## Proposed Approach

### Phase 1: Sensitivity-Based Calculation

From the diagnostic tests, we can calculate **sensitivity** (how much the metric changes per unit parameter change):

```
Diagnostic test: Halve Roll gains (50% reduction)
Result: Motor RMS dropped from 80 to 60 (25% improvement)

Sensitivity = ΔMetric / ΔParameter
           = (80 - 60) / (0.5 × rollP)
           = 20 / (0.5 × 50)
           = 0.8 RMS per unit P

To reach target (15), we need:
ΔNeeded = currentRMS - target = 60 - 15 = 45
ΔParameter = ΔNeeded / sensitivity = 45 / 0.8 = 56

Calculated adjustment: Reduce P by 56 (but clamp to reasonable limits)
```

### Phase 2: History Tracking (Per-Axis)

Track the last N iterations to build a model of the parameter→metric relationship.
**Separate histories for each axis** - Roll, Pitch, Yaw are independent.

```c
#define ADJUSTMENT_HISTORY_SIZE 5

typedef struct {
    float parameterValue;    // e.g., P gain = 45
    float metricValue;       // e.g., motor RMS = 72.5
    timeUs_t timestamp;
} adjustmentHistoryEntry_t;

typedef struct {
    adjustmentHistoryEntry_t entries[ADJUSTMENT_HISTORY_SIZE];
    uint8_t count;
    uint8_t writeIndex;
} adjustmentHistory_t;

// Separate history for each axis AND each tunable parameter type
typedef struct {
    adjustmentHistory_t p;
    adjustmentHistory_t i;
    adjustmentHistory_t d;
    adjustmentHistory_t f;
    adjustmentHistory_t dtermLpf1;
} axisHistory_t;

typedef struct {
    axisHistory_t roll;
    axisHistory_t pitch;
    axisHistory_t yaw;
    adjustmentHistory_t gyroLpf1;  // Common to all axes
} tuneHistory_t;
```

### Phase 2b: Term-Specific Metrics

Different PID terms optimize for different metrics:

```c
typedef enum {
    METRIC_MOTOR_RMS,           // Hover tune - motor noise
    METRIC_OSCILLATION,         // D term - high-frequency oscillation
    METRIC_SETPOINT_TRACKING,   // P term - response to setpoint
    METRIC_STICK_TRACKING,      // F term - tracking during rapid moves
    METRIC_LONG_TERM_ERROR,     // I term - accumulated drift/offset
    METRIC_OVERSHOOT,           // Step response overshoot %
    METRIC_SETTLING_TIME,       // Step response settling time
} tuneMetric_e;

// Target values for each metric
#define TARGET_MOTOR_RMS           15.0f
#define TARGET_OSCILLATION         5.0f   // Low oscillation
#define TARGET_SETPOINT_TRACKING   0.95f  // 95% tracking accuracy
#define TARGET_STICK_TRACKING      0.90f  // 90% during rapid moves
#define TARGET_LONG_TERM_ERROR     2.0f   // Minimal accumulated error
#define TARGET_OVERSHOOT           10.0f  // 10% max overshoot
#define TARGET_SETTLING_TIME       150.0f // 150ms settling

// Which metric applies to which parameter
static tuneMetric_e getMetricForParameter(tuneParameter_e param) {
    switch (param) {
        case PARAM_P:  return METRIC_SETPOINT_TRACKING;
        case PARAM_I:  return METRIC_LONG_TERM_ERROR;
        case PARAM_D:  return METRIC_OSCILLATION;
        case PARAM_F:  return METRIC_STICK_TRACKING;
        case PARAM_DTERM_LPF1: return METRIC_MOTOR_RMS;
        case PARAM_GYRO_LPF1:  return METRIC_MOTOR_RMS;
        default: return METRIC_MOTOR_RMS;
    }
}
```

### Phase 3: Newton's Method

After 2+ data points, use Newton's method for rapid convergence:

```
Newton's method: x_{n+1} = x_n - f(x_n) / f'(x_n)

Where:
  x = parameter value (e.g., P gain)
  f(x) = metric(x) - target (e.g., RMS - 15)
  f'(x) = numerical derivative (slope from history)

Calculation:
  f'(x) ≈ (metric_n - metric_{n-1}) / (param_n - param_{n-1})
  
  newParam = currentParam - (currentMetric - target) / f'(x)
```

**Example:**
```
Iteration 1: P=50, RMS=80
Iteration 2: P=40, RMS=65 (after -10 step)

f'(x) = (65 - 80) / (40 - 50) = -15 / -10 = 1.5

To reach target RMS=15:
newP = 40 - (65 - 15) / 1.5
     = 40 - 50 / 1.5
     = 40 - 33.3
     = 6.7

Clamp to minimum: P = max(6.7, 10) = 10
```

### Safety Clamps

Always apply safety limits with **90% damping factor**:
```c
#define NEWTON_MAX_STEP_PERCENT  50  // Never change more than 50% at once
#define NEWTON_MIN_DERIVATIVE   0.1  // Avoid division by near-zero
#define NEWTON_DAMPING_FACTOR  0.90f // Apply 90% of calculated change

float calculateNewtonAdjustment(float currentParam, float currentMetric, 
                                 float target, adjustmentHistory_t *history) {
    if (history->count < 2) {
        // Not enough data - fall back to proportional estimate
        return calculateProportionalAdjustment(currentParam, currentMetric, target);
    }
    
    // Get previous entry
    uint8_t prevIndex = (history->writeIndex - 1 + ADJUSTMENT_HISTORY_SIZE) % ADJUSTMENT_HISTORY_SIZE;
    float prevParam = history->entries[prevIndex].parameterValue;
    float prevMetric = history->entries[prevIndex].metricValue;
    
    // Calculate numerical derivative
    float deltaParam = currentParam - prevParam;
    float deltaMetric = currentMetric - prevMetric;
    
    if (fabsf(deltaParam) < 0.1f) {
        return 0;  // No meaningful change
    }
    
    float derivative = deltaMetric / deltaParam;
    
    // Avoid division by near-zero
    if (fabsf(derivative) < NEWTON_MIN_DERIVATIVE) {
        derivative = (derivative >= 0) ? NEWTON_MIN_DERIVATIVE : -NEWTON_MIN_DERIVATIVE;
    }
    
    // Newton's method: new = current - f(current) / f'(current)
    float error = currentMetric - target;
    float adjustment = -error / derivative;
    
    // Apply 90% damping factor for stability
    adjustment *= NEWTON_DAMPING_FACTOR;
    
    // Clamp to maximum step size
    float maxStep = currentParam * (NEWTON_MAX_STEP_PERCENT / 100.0f);
    adjustment = constrainf(adjustment, -maxStep, maxStep);
    
    return adjustment;
}
```

## Implementation Plan

### Step 1: Add History Tracking Infrastructure
- Add `adjustmentHistory_t` struct to `autotune_types.h`
- Add `axisHistory_t` and `tuneHistory_t` for per-axis tracking
- Add history arrays to `autotuneRuntime_t`
- Add helper functions: `recordAdjustment()`, `getHistoryEntry()`

### Step 2: Add Term-Specific Metric Infrastructure
- Add `tuneMetric_e` enum for different optimization targets
- Add measurement functions for each metric type:
  - `measureOscillation()` - for D term
  - `measureSetpointTracking()` - for P term
  - `measureStickTracking()` - for F term (during rapid stick moves)
  - `measureLongTermError()` - for I term (accumulated error over time)

### Step 3: Implement Sensitivity Calculation
- After diagnostic tests complete, calculate sensitivity for each parameter
- Store sensitivities for use in calculated adjustments
- Sensitivity = improvement_percent / test_change_percent

### Step 4: Implement Calculated Adjustment
- `calculateProportionalAdjustment()` - uses sensitivity alone
- `calculateNewtonAdjustment()` - uses history + Newton's method with 90% damping
- `selectAdjustmentMethod()` - chooses based on history count

### Step 5: Implement Sequential Multi-Variable Adjustment
- Identify all parameters that need adjustment from diagnostic results
- Queue adjustments in priority order
- Apply ONE adjustment, verify it didn't break the tune
- If verified, move to next parameter in queue
- When queue is empty (all verified), give wiggle signal

### Step 6: Integrate with Hover Diagnostic
- When applying fix, use calculated adjustment instead of fixed step
- Record (parameter, metric) pair after each adjustment
- Build history over successive diagnostic cycles

### Step 7: Extend to PID Tune Mode
- Apply same methodology to step response tuning
- Track overshoot/response metrics vs PID gains
- Use Newton's method for P, I, D, F convergence

## Sequential Multi-Variable Adjustment

When diagnostic tests reveal multiple parameters need adjustment:

```c
typedef struct {
    tuneParameter_e parameter;
    int axis;                    // FD_ROLL, FD_PITCH, FD_YAW or -1 for common
    float calculatedAdjustment;
    float sensitivityEstimate;
    float improvementPercent;
} pendingAdjustment_t;

typedef struct {
    pendingAdjustment_t queue[8];   // Up to 8 pending adjustments
    uint8_t count;
    uint8_t currentIndex;
    float preAdjustMetric;          // Metric before current adjustment
} adjustmentQueue_t;

// State machine for sequential adjustment
typedef enum {
    ADJ_STATE_IDLE,
    ADJ_STATE_APPLY_NEXT,       // Apply next adjustment from queue
    ADJ_STATE_MEASURING,        // Measure result of adjustment
    ADJ_STATE_VERIFY,           // Check if adjustment helped or hurt
    ADJ_STATE_REVERT,           // Revert if adjustment made things worse
    ADJ_STATE_COMPLETE,         // All adjustments verified, wiggle
} adjustmentState_e;

void updateSequentialAdjustment(void) {
    switch (runtime.adjState) {
        case ADJ_STATE_APPLY_NEXT:
            if (runtime.adjQueue.currentIndex >= runtime.adjQueue.count) {
                // All done - wiggle and complete
                runtime.adjState = ADJ_STATE_COMPLETE;
                doWiggleSignal();
                break;
            }
            pendingAdjustment_t *adj = &runtime.adjQueue.queue[runtime.adjQueue.currentIndex];
            
            // Record pre-adjustment metric
            runtime.adjQueue.preAdjustMetric = getCurrentMetric(adj->parameter);
            
            // Apply the adjustment (with 90% damping already applied)
            applyCalculatedAdjustment(adj);
            
            runtime.adjState = ADJ_STATE_MEASURING;
            runtime.phaseStartTime = micros();
            break;
            
        case ADJ_STATE_MEASURING:
            // Wait for measurement window
            if (cmpTimeUs(micros(), runtime.phaseStartTime) < DIAG_MEASURE_TIME_US) {
                break;
            }
            runtime.adjState = ADJ_STATE_VERIFY;
            break;
            
        case ADJ_STATE_VERIFY:
            {
                pendingAdjustment_t *adj = &runtime.adjQueue.queue[runtime.adjQueue.currentIndex];
                float postMetric = getCurrentMetric(adj->parameter);
                float improvement = (runtime.adjQueue.preAdjustMetric - postMetric) 
                                   / runtime.adjQueue.preAdjustMetric * 100.0f;
                
                if (improvement < -5.0f) {
                    // Made things worse by more than 5% - revert
                    runtime.adjState = ADJ_STATE_REVERT;
                } else {
                    // Acceptable or improved - record and move to next
                    recordAdjustmentToHistory(adj, postMetric);
                    runtime.adjQueue.currentIndex++;
                    runtime.adjState = ADJ_STATE_APPLY_NEXT;
                }
            }
            break;
            
        case ADJ_STATE_REVERT:
            {
                pendingAdjustment_t *adj = &runtime.adjQueue.queue[runtime.adjQueue.currentIndex];
                revertAdjustment(adj);
                
                // Skip this parameter, try next
                runtime.adjQueue.currentIndex++;
                runtime.adjState = ADJ_STATE_APPLY_NEXT;
            }
            break;
            
        case ADJ_STATE_COMPLETE:
            // Done - move back to idle or hover diagnostic
            runtime.adjState = ADJ_STATE_IDLE;
            runtime.state = AUTOTUNE_PHASE_MONITORING;
            break;
    }
}
```

## Data Flow

```
Diagnostic Cycle
      │
      ▼
┌─────────────────┐
│ Measure Tests   │ → Roll: -25% RMS, Pitch: -10% RMS, D-osc: -15%, etc.
└─────────────────┘
      │
      ▼
┌─────────────────┐
│ Calc Sensitivity│ → Roll sensitivity = 0.5 RMS/unit P (per-axis)
└─────────────────┘
      │
      ▼
┌──────────────────────────┐
│ Identify ALL Needed Fixes│ → Roll P, Pitch D, Dterm LPF1
└──────────────────────────┘
      │
      ▼
┌─────────────────────────────────────┐
│ Calculate Adjustments (for each)    │
│                                     │
│ if (history.count >= 2)             │
│   → Newton's method × 0.90          │
│ else if (sensitivity known)         │
│   → Proportional × 0.90             │
│ else                                │
│   → Fixed step (fallback)           │
└─────────────────────────────────────┘
      │
      ▼
┌────────────────────────────┐
│ Queue Adjustments          │ → Priority order by improvement %
└────────────────────────────┘
      │
      ▼
┌─────────────────────────────────────┐
│ Sequential Apply & Verify Loop      │
│                                     │
│ for each adjustment in queue:       │
│   1. Apply adjustment               │
│   2. Measure result                 │
│   3. Verify didn't break tune       │
│   4. If worse >5%, revert & skip    │
│   5. If OK, record to history       │
│   6. Next adjustment                │
└─────────────────────────────────────┘
      │
      ▼
┌─────────────────┐
│ All Complete    │ → Wiggle signal to pilot
└─────────────────┘
      │
      ▼
┌─────────────────┐
│ Re-run Diagnostic│ → Check overall improvement, iterate if needed
└─────────────────┘
```

## Expected Benefits

| Metric | Fixed Steps | Calculated + Newton |
|--------|-------------|---------------------|
| Iterations to converge | 5-10 | 2-4 |
| Risk of overshoot | High | Low (90% damping) |
| Adapts to aircraft | No | Yes (measures sensitivity) |
| Uses all data | No | Yes (per-axis history) |
| Multi-variable handling | One at a time blind | Sequential with verification |
| Detects bad changes | No | Yes (reverts if >5% worse) |

## Convergence Comparison (Simulated)

```
Target RMS: 15
Starting RMS: 80

Fixed Steps (Δ=10):
  Iter 1: P=50, RMS=80
  Iter 2: P=45, RMS=72  (-8)
  Iter 3: P=40, RMS=64  (-8)
  Iter 4: P=35, RMS=56  (-8)
  Iter 5: P=30, RMS=48  (-8)
  Iter 6: P=25, RMS=40  (-8)
  Iter 7: P=20, RMS=32  (-8)
  Iter 8: P=15, RMS=24  (-8)
  Iter 9: P=10, RMS=16  (-8) ← At minimum, close to target

Newton's Method (with 90% damping):
  Iter 1: P=50, RMS=80 (initial)
  Iter 2: P=40, RMS=64 (fixed step to get derivative)
  Iter 3: P=10, RMS=18 (Newton: 40 - 0.9×(64-15)/1.6 = 12.4, clamped to 10)
  Iter 4: Done - RMS ≤ target
```

## Term-Specific Tuning Examples

### D Term (Oscillation Metric)
```
Diagnostic: Halve D gain → Oscillation reduced 30%
Sensitivity: 0.6 osc units per D point

Current D=35, Oscillation=25 (target=5)
Newton: ΔD = -0.9 × (25-5) / 0.6 = -30
New D = 35 - 30 = 5 (clamp to minimum 10)
```

### I Term (Long-Term Error Metric)
```
Measure accumulated error over 2 seconds
If drift > threshold, increase I
If oscillation at low frequency, decrease I

Newton uses error integral vs I gain relationship
```

### F Term (Stick Tracking Metric)
```
During rapid stick input (>50% deflection):
Measure lag between setpoint and gyro response

If tracking < 90%, increase F
Newton calculates optimal F from response delay
```

## Design Decisions (Resolved)

1. **History per-axis or global?** 
   - ✅ **Separate histories per axis** - Roll, Pitch, Yaw each have independent histories since they may have different sensitivities

2. **Damping factor?**
   - ✅ **90% of calculated change** - Never apply more than 90% of calculated adjustment for stability

3. **Term-specific metrics?**
   - ✅ **Yes** - Each PID term optimizes for a different metric:
     - P: Setpoint tracking / responsiveness
     - I: Long-term error / drift
     - D: Oscillation detection
     - F: Stick tracking during rapid movements

4. **Multiple parameters at once?**
   - ✅ **Sequential with verification** - Apply one adjustment at a time, verify each didn't make things worse (>5%), wiggle only when all complete

5. **Reset history on mode change?**
   - Recommend: Reset, different modes measure different things

6. **Sensitivity persistence?** 
   - Recommend: No for now, recalculate each session (aircraft may change)

## Next Steps

1. [ ] Implement history tracking infrastructure (per-axis `tuneHistory_t`)
2. [ ] Add term-specific metric measurement functions
3. [ ] Add sensitivity calculation after diagnostic tests
4. [ ] Implement `calculateNewtonAdjustment()` with 90% damping
5. [ ] Implement sequential multi-variable adjustment queue
6. [ ] Integrate with `applyDiagnosticFix()` to use calculated adjustments
7. [ ] Add wiggle signal when all adjustments verified
8. [ ] Test and tune convergence parameters
9. [ ] Extend to PID tune mode
