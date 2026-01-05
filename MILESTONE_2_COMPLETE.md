# Milestone 2 Complete: Step Response Measurement

**Date:** January 4, 2026  
**Status:** ✅ COMPLETE - Flight validated

## Overview
Implemented bidirectional doublet excitation and sample collection system to measure aircraft step response for PID tuning analysis.

## What Was Implemented

### 1. Dynamic Sample Rate Calculation
- **Problem:** Hardcoded sample intervals assumed 8kHz PID loop
- **Solution:** Calculate `sampleIntervalLoops` dynamically based on `pidGetPidFrequency()`
- **Target:** 200Hz sample rate regardless of PID frequency
- **Code:** Added `AUTOTUNE_TARGET_SAMPLE_RATE_HZ` constant and calculation in `autotuneInit()`

### 2. Bidirectional Doublet Excitation
- **Evolution:** Started with simple doublet (+A, -A), evolved to zero-displacement design
- **Final Pattern:** +A, -2A, +2A, -2A, +2A, -A (6 phases)
- **Duration:** 6× `step_duration_ms` (600ms with defaults)
- **Benefits:**
  - Zero net angular displacement (returns to starting attitude naturally)
  - Tests both directions (detects asymmetries)
  - More data for analysis
  - Strong, visible response

### 3. Sample Collection System
- **Buffer Size:** 200 samples (gyroHistory[], setpointHistory[])
- **Collection:** Subsample every `sampleIntervalLoops` PID cycles
- **States:**
  - **EXCITE:** Apply doublet + collect samples (~120 samples)
  - **MEASURE:** Observe settling + continue collecting (~60 samples)
  - **Total:** ~181 samples over 900ms

### 4. State Machine Updates
- **EXCITE State:**
  - Initialize buffers and timing on entry
  - Apply bidirectional doublet via `autotuneGetExcitation()`
  - Collect gyro + setpoint pairs
  - Transition after 6× step duration
  
- **MEASURE State:**
  - Continue sample collection (excitation = 0)
  - Observe natural settling behavior
  - Duration: 3× step duration (300ms)
  - Exit when time expires OR buffer full

### 5. PID Loop Integration
- **Integration Point:** After `accelerationLimit()` in `pidController()`
- **Method:** Additive control via `autotuneModifySetpoint()`
- **Behavior:** 
  - Only modifies setpoint during EXCITE state
  - Only on axis being tested
  - Pilot maintains full control authority

## Files Modified

### Created/Modified in Milestone 2:
- `src/main/flight/autotune.c` - Core implementation
  - Dynamic sample rate calculation
  - Bidirectional doublet generator
  - Sample collection logic
  - State machine EXCITE/MEASURE states
  - `autotuneModifySetpoint()` function

- `src/main/flight/autotune.h` - API updates
  - Added `autotuneModifySetpoint()` declaration

- `src/main/flight/pid.c` - Integration
  - Added autotune setpoint modification after acceleration limiting

## Flight Test Validation

### Test Configuration:
- **Target:** BETAFPVG473 (STM32G47X)
- **Firmware:** betaflight 2026.6.0-alpha
- **Settings:**
  - `autotune_step_amplitude = 100` (deg/s)
  - `autotune_step_duration_ms = 100` (ms)
  - `debug_mode = AUTOTUNE`

### Validated Results:
- ✅ State transitions: 1→2→3→4→5 (no skipping)
- ✅ Doublet visible: +100/-200/+200/-200/+200/-100 deg/s pattern
- ✅ Aircraft response: Clearly felt and visible in flight
- ✅ Sample collection: 181 samples (120 EXCITE + 61 MEASURE)
- ✅ Timing: 600ms excitation + 300ms settle = 900ms total
- ✅ Natural return: Aircraft returns to starting attitude
- ✅ Safety: Instant abort on stick movement

### Blackbox Debug Channels:
- **debug[0]:** State progression (validated 1→2→3→4→5)
- **debug[1]:** Sample count (validated 0→120→181)
- **debug[2]:** Gyro response (shows clear step response)
- **debug[3]:** Excitation signal (shows +100/-200/+200/-200/+200/-100 pattern)
- **debug[7]:** Time in doublet (validated 0→600ms in EXCITE)

## Key Bugs Fixed

### 1. State Initialization Bug
- **Problem:** EXCITE state skipped entirely (jumped 2→4)
- **Root Cause:** `if (runtime.stateEnteredAt == currentTimeUs)` unreliable (exact time match)
- **Solution:** Added `stateJustEntered` flag set in `changeState()`, cleared on first execution
- **Impact:** State now properly initializes `excitationStartTime` and buffers

### 2. Type Mismatch in Duration Comparisons
- **Problem:** Compiler error comparing `timeDelta_t` (signed) with `uint32_t` (unsigned)
- **Root Cause:** Duration variables declared as `uint32_t` but `cmpTimeUs()` returns signed
- **Solution:** Changed duration variables to `timeDelta_t`
- **Files:** EXCITE and MEASURE state duration checks

## Design Evolution

### Doublet Design Progression:
1. **Simple doublet:** +A, -A (2 phases)
   - Problem: Doesn't guarantee return to starting attitude
   
2. **3-2-1-1 doublet:** +A, -2A, +A (3 phases)
   - Better: Zero net displacement
   - Issue: Only tests one direction strongly
   
3. **Bidirectional:** +A, -2A, +A, -A, +2A, -A (6 phases)
   - Issue: Weak middle transition
   
4. **Final bidirectional:** +A, -2A, +2A, -2A, +2A, -A (6 phases) ✅
   - Zero net displacement
   - Strong bidirectional excitation
   - Symmetric response testing
   - Natural return to level

## Lessons Learned

### 1. **Avoid Magic Numbers**
- Always calculate based on actual system parameters
- PID frequency varies (1kHz-16kHz) across configurations
- Use target specifications (200Hz sample rate) instead of hardcoded intervals

### 2. **Time-Based State Initialization**
- Never use exact time equality checks (`==`) for microsecond timestamps
- Use flags or other mechanisms for one-time initialization
- Critical for real-time systems with variable execution timing

### 3. **Type Safety in Time Comparisons**
- Match signed/unsigned types when comparing time deltas
- `cmpTimeUs()` returns signed `timeDelta_t` for good reason (handles wraparound)
- Duration calculations should use same type

### 4. **Excitation Design Matters**
- Zero-displacement patterns essential for safe flight
- Bidirectional testing improves identification quality
- Multiple iterations improved response quality significantly

### 5. **Debug Output is Critical**
- Comprehensive debug channels saved hours of troubleshooting
- State number, sample count, timing all visible in blackbox
- Made remote debugging possible without bench access

### 6. **Additive Control Model**
- Preserving pilot authority critical for safety
- Autotune as perturbation rather than override works well
- Allows instant manual abort if needed

## Memory Impact
- **RAM:** ~1.6KB additional (200 samples × 2 arrays × 4 bytes)
- **Flash:** ~2KB for doublet logic and sample collection
- **Total:** Acceptable for target hardware (124KB RAM, 1MB flash)

## Performance
- **Execution:** Negligible impact on PID loop (simple array writes when sampling)
- **Sample Rate:** Confirmed 200Hz regardless of PID frequency (tested at 8kHz)
- **Latency:** No noticeable delay in flight control

## Next Steps (Milestone 3)
1. Implement step response analysis:
   - Rise time calculation
   - Overshoot detection
   - Settling time measurement
   - Oscillation frequency extraction

2. PID gain calculation:
   - Map response metrics to optimal gains
   - Implement gain adjustment logic
   - Apply calculated gains to PID controller

3. Iteration logic:
   - Test new gains
   - Repeat until target performance achieved
   - Limit iterations for safety

## Conclusion
Milestone 2 successfully implements a robust excitation and measurement system for in-flight PID autotuning. The bidirectional doublet provides high-quality step response data while maintaining aircraft safety and pilot control. Flight validation confirms correct operation with 181 samples collected over 900ms, ready for analysis in Milestone 3.

**Total Development Time:** ~3 hours (including design iterations and bug fixes)
**Flight Test Time:** ~15 minutes (multiple tests to validate)
**Status:** Ready for Milestone 3 implementation
