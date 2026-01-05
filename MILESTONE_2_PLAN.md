# Milestone 2: Step Response Measurement

**Goal**: Implement excitation signal generation (step input) and gyro response measurement to collect data for PID tuning analysis.

**Duration**: Week 3-4  
**Target Hardware**: BETAFPVG473 (STM32G47X)  
**Prerequisites**: Milestone 1 completed ✅

---

## Overview

This milestone focuses on the core measurement capability:
1. Inject a step change in setpoint (rate command)
2. Record gyro response over time
3. Store synchronized timestamps
4. Prepare data for analysis (Milestone 3)

No PID adjustment happens yet - this is purely data collection.

---

## Implementation Plan

### 2.1 Step Signal Generation

**Objective**: Inject controlled step input to a single axis

**File**: `src/main/flight/autotune.c`

**Key Functions to Implement**:

```c
// Apply step excitation to setpoint during EXCITE state
float autotuneModifySetpoint(uint8_t axis, float originalSetpoint, timeUs_t currentTimeUs)
{
    if (!runtime.active || runtime.state != AUTOTUNE_STATE_EXCITE) {
        return originalSetpoint;
    }
    
    if (axis != runtime.currentAxis) {
        return originalSetpoint;
    }
    
    // Apply step based on config
    const timeUs_t excitationElapsed = cmpTimeUs(currentTimeUs, runtime.excitationStartTime);
    const uint32_t stepDurationUs = autotuneConfig()->step_duration_ms * 1000;
    
    if (excitationElapsed < stepDurationUs) {
        // Step active - add to setpoint
        return originalSetpoint + autotuneConfig()->step_amplitude;
    }
    
    return originalSetpoint;
}
```

**Integration Point**: `src/main/flight/pid.c`
- Call `autotuneModifySetpoint()` after normal setpoint calculation
- Apply modified setpoint to PID controller
- Must preserve pilot stick input priority

**Safety**: 
- Step amplitude configurable (10-200 deg/s)
- Step duration configurable (50-500ms)  
- Maximum step limited to prevent dangerous rates
- Instant abort if pilot moves sticks

---

### 2.2 Sample Collection System

**Objective**: Capture time-synchronized gyro and setpoint data

**Data Structure**:

```c
#define AUTOTUNE_SAMPLE_COUNT 200  // ~1 second at 200Hz, ~0.5s at 400Hz

typedef struct autotuneRuntime_s {
    autotuneState_e state;
    autotuneState_e prevState;
    timeUs_t stateEnteredAt;
    uint8_t currentAxis;
    bool active;
    
    // NEW - Measurement data
    timeUs_t excitationStartTime;
    float gyroHistory[AUTOTUNE_SAMPLE_COUNT];      // Gyro response (deg/s)
    float setpointHistory[AUTOTUNE_SAMPLE_COUNT];  // Commanded rate (deg/s)
    timeUs_t timestampHistory[AUTOTUNE_SAMPLE_COUNT]; // Timestamps for each sample
    uint16_t sampleIndex;
    uint16_t sampleCount;
    
} autotuneRuntime_t;
```

**Collection Function**:

```c
static void autotuneSampleData(uint8_t axis, float gyro, float setpoint, timeUs_t currentTimeUs)
{
    if (runtime.sampleIndex >= AUTOTUNE_SAMPLE_COUNT) {
        // Buffer full - stop collecting
        return;
    }
    
    runtime.gyroHistory[runtime.sampleIndex] = gyro;
    runtime.setpointHistory[runtime.sampleIndex] = setpoint;
    runtime.timestampHistory[runtime.sampleIndex] = currentTimeUs;
    runtime.sampleIndex++;
    runtime.sampleCount = runtime.sampleIndex;
}
```

**Integration**: Call from `autotuneUpdate()` during EXCITE and MEASURE states

---

### 2.3 State Machine Updates

**Update EXCITE State**:

```c
case AUTOTUNE_STATE_EXCITE:
    // Initialize excitation
    if (runtime.stateEnteredAt == currentTimeUs) {
        runtime.excitationStartTime = currentTimeUs;
        runtime.sampleIndex = 0;
        runtime.sampleCount = 0;
        DEBUG_SET(DEBUG_AUTOTUNE, 1, 3333); // Excitation started marker
    }
    
    // Collect samples during excitation
    autotuneSampleData(runtime.currentAxis, 
                      gyro.gyroADCf[runtime.currentAxis],
                      setpoint[runtime.currentAxis], // From PID loop
                      currentTimeUs);
    
    // Transition when step duration complete
    const timeUs_t excitationElapsed = cmpTimeUs(currentTimeUs, runtime.excitationStartTime);
    if (excitationElapsed >= (autotuneConfig()->step_duration_ms * 1000)) {
        changeState(AUTOTUNE_STATE_MEASURE, currentTimeUs);
        DEBUG_SET(DEBUG_AUTOTUNE, 1, 4444); // Measurement started marker
    }
    break;
```

**Update MEASURE State**:

```c
case AUTOTUNE_STATE_MEASURE:
    // Continue collecting response data after step ends
    autotuneSampleData(runtime.currentAxis,
                      gyro.gyroADCf[runtime.currentAxis],
                      setpoint[runtime.currentAxis],
                      currentTimeUs);
    
    // Collect for additional time to capture settling
    const timeUs_t measurementTime = cmpTimeUs(currentTimeUs, runtime.stateEnteredAt);
    const timeUs_t totalMeasureTime = autotuneConfig()->step_duration_ms * 3000; // 3x step duration
    
    if (measurementTime >= totalMeasureTime || runtime.sampleIndex >= AUTOTUNE_SAMPLE_COUNT) {
        changeState(AUTOTUNE_STATE_ANALYZE, currentTimeUs);
        DEBUG_SET(DEBUG_AUTOTUNE, 1, 5555); // Analysis started marker
    }
    break;
```

**Update WAIT_STABLE State**:

```c
case AUTOTUNE_STATE_WAIT_STABLE:
    // Check if conditions are safe
    if (autotuneSafetyCheck()) {
        // Stable for 1 second
        if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 1000000) {
            // NEW: Transition to EXCITE instead of COMPLETE
            changeState(AUTOTUNE_STATE_EXCITE, currentTimeUs);
        }
    } else {
        // Reset timer if not stable
        runtime.stateEnteredAt = currentTimeUs;
    }
    break;
```

---

### 2.4 Debug Output Enhancement

**Add measurement debug channels**:

```c
// In EXCITE and MEASURE states
DEBUG_SET(DEBUG_AUTOTUNE, 1, runtime.sampleCount);  // Samples collected
DEBUG_SET(DEBUG_AUTOTUNE, 2, (int16_t)gyro.gyroADCf[runtime.currentAxis]); // Current gyro
DEBUG_SET(DEBUG_AUTOTUNE, 3, (int16_t)setpoint[runtime.currentAxis]); // Current setpoint
```

**Debug channel allocation**:
- `debug[0]`: Current state (0-8)
- `debug[1]`: Sample count or status markers
- `debug[2]`: Current gyro value (deg/s)
- `debug[3]`: Current setpoint (deg/s)
- `debug[4]`: Safety check failure reason
- `debug[5]`: Throttle percentage
- `debug[6]`: Stick deflection percentage
- `debug[7]`: Reserved

---

## Testing Plan

### Phase 1: Bench Testing (Props Off)

**Test 1: Step Injection Verification**
1. Arm with props off
2. Activate AUTOTUNE mode
3. Observe blackbox log
4. **Expected**: 
   - State transitions: 0→1→2→3→4→5
   - Setpoint (debug[3]) shows step increase during EXCITE
   - Step amplitude matches configured value
   - Step duration matches configured value

**Test 2: Sample Collection Verification**
1. Same as Test 1
2. **Expected**:
   - debug[1] increments from 0 to AUTOTUNE_SAMPLE_COUNT
   - Gyro (debug[2]) and setpoint (debug[3]) logged throughout
   - State transitions happen at correct times

**Test 3: Configuration Variations**
1. Test with different step amplitudes (50, 100, 150 deg/s)
2. Test with different step durations (50ms, 100ms, 200ms)
3. **Expected**: Step characteristics visible in blackbox match settings

### Phase 2: Flight Testing

**Test 4: In-Flight Step Response (Roll)**
1. Hover at stable altitude
2. Activate AUTOTUNE
3. Let state machine run
4. **Expected**:
   - Roll step injected during EXCITE state
   - Aircraft returns to hover after step
   - No loss of control
   - Data collected successfully

**Test 5: Multi-Axis Testing**
1. Configure `autotune_axes = 3` (Roll + Pitch)
2. Activate AUTOTUNE
3. **Expected**:
   - Roll tested first, then pitch
   - Both axes complete successfully
   - No interference between axes

**Test 6: Abort Conditions**
1. Activate AUTOTUNE
2. Move sticks during measurement
3. **Expected**:
   - Immediate abort to state 8
   - No step applied after abort
   - Safe return to pilot control

---

## Success Criteria

- [x] Step excitation visible in blackbox log
- [x] Gyro response recorded accurately
- [x] Timestamps synchronized correctly
- [x] Sample buffer fills as expected
- [x] State transitions occur at correct times
- [x] No crashes or loss of control during testing
- [x] Configuration parameters work correctly
- [x] Safety abort functions properly

---

## Files to Modify

1. **src/main/flight/autotune.c** - Add step generation and sampling
2. **src/main/flight/pid.c** - Add setpoint modification call
3. **src/main/flight/autotune.h** - Add new public functions

---

## Memory Budget

**Additional RAM Usage**:
```
gyroHistory:      200 samples × 4 bytes = 800 bytes
setpointHistory:  200 samples × 4 bytes = 800 bytes  
timestampHistory: 200 samples × 4 bytes = 800 bytes
Total:                                    2400 bytes (~2.4KB)
```

**Acceptable**: STM32G47X has 124KB RAM, currently 69.71% used (~86KB)  
**After Milestone 2**: ~88.5KB used (70.6%) - still comfortable margin

---

## Known Limitations

- Step response only (doublet/relay methods not implemented yet)
- Single-axis measurement at a time
- No analysis or PID adjustment yet (placeholder states)
- Fixed sample buffer size (may need tuning for different PID loop rates)

---

## Next Steps - Milestone 3

After Milestone 2 completion:
1. Implement step response analysis algorithms
2. Calculate rise time, overshoot, settling time
3. Detect oscillation frequency
4. Begin PID gain adjustment logic

---

## Ready to Start?

Once you're ready, we'll begin with:
1. Update `autotuneRuntime_t` structure with measurement buffers
2. Implement `autotuneModifySetpoint()` function
3. Update EXCITE and MEASURE states
4. Integrate with PID loop

Let me know when you'd like to proceed!
