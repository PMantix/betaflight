# Milestone 3 Complete: Procedural Diagnostic Hover Tune

**Date:** January 11, 2026  
**Status:** ✅ COMPLETE - Flight validated

## Overview
Replaced heuristic noise attribution with a controlled A/B testing approach for hover tune. The new procedural diagnostic system directly measures the effect of each parameter change, enabling accurate identification of the dominant noise source and applying targeted fixes.

## Problem Statement

The previous noise attribution approach used ratio-based heuristics (e.g., `gyroRms/dTermRms > 0.8`) to guess which component was causing motor noise. Flight testing revealed:

1. **Inaccurate attribution** - The ratios often didn't correlate with actual noise sources
2. **Overcorrection** - Lowering D-term filter past optimal point made noise worse
3. **No feedback loop** - No mechanism to detect when changes made things worse
4. **Heuristic thresholds** - Arbitrary thresholds tuned for one aircraft type

## Solution: Procedural Diagnostic Testing

Instead of guessing based on ratios, we now **directly measure** the effect of each parameter change through controlled A/B tests.

### Diagnostic Sequence (per iteration)
```
[0.0s - 0.5s]  BASELINE         → Record baseline motor RMS
[0.5s - 1.0s]  ROLL_TEST        → Halve Roll gains, measure, restore
[1.0s - 1.5s]  PITCH_TEST       → Halve Pitch gains, measure, restore
[1.5s - 2.0s]  GYRO_LPF1_TEST   → Lower Gyro LPF1 by 50Hz, measure, restore
[2.0s - 2.5s]  DTERM_LPF1_TEST  → Lower Dterm LPF1 by 50Hz, measure, restore
[2.5s]         ANALYZING        → Calculate improvement %, apply fix to dominant
```

**Total time per iteration:** ~2.5 seconds

### Decision Logic

1. Calculate improvement percentage for each test:
   ```
   improvement% = (baseline - testRms) / baseline * 100
   ```

2. Find the test with the **highest improvement** (positive = better)

3. Apply permanent fix **only if**:
   - Improvement > 10% threshold
   - It's the single dominant contributor

4. Re-run diagnostic cycle to verify fix effectiveness

5. Exit when:
   - Motor RMS ≤ target (15)
   - No test shows >10% improvement
   - Max iterations (5) reached

### Key Advantages

| Old Heuristic Approach | New Diagnostic Approach |
|------------------------|-------------------------|
| Guesses based on ratios | Direct A/B measurement |
| Can overcorrect past optimal | Verifies each change helped |
| Single-shot decisions | Iterative refinement |
| Arbitrary thresholds | Data-driven decisions |
| Hard to debug | Clear phase progression |

## Implementation Details

### New Diagnostic Phases (autotune_types.h)
```c
typedef enum {
    HOVER_DIAG_IDLE = 0,
    HOVER_DIAG_BASELINE,        // Measuring with current settings
    HOVER_DIAG_ROLL_TEST,       // Roll gains * 0.5
    HOVER_DIAG_PITCH_TEST,      // Pitch gains * 0.5
    HOVER_DIAG_GYRO_LPF1_TEST,  // Gyro LPF1 - 50Hz
    HOVER_DIAG_DTERM_LPF1_TEST, // Dterm LPF1 - 50Hz
    HOVER_DIAG_ANALYZING,       // Comparing results
    HOVER_DIAG_COMPLETE,
} hoverDiagPhase_e;
```

### New Reason Codes (1500-1599)
- `1500` - REASON_DIAG_BASELINE - Measuring baseline
- `1510` - REASON_DIAG_ROLL_TEST - Testing with Roll * 0.5
- `1520` - REASON_DIAG_PITCH_TEST - Testing with Pitch * 0.5
- `1530` - REASON_DIAG_GYRO_LPF1_TEST - Testing Gyro LPF1 - 50Hz
- `1540` - REASON_DIAG_DTERM_LPF1_TEST - Testing Dterm LPF1 - 50Hz
- `1550` - REASON_DIAG_ANALYZING - Analyzing results
- `1560-1563` - REASON_DIAG_FIX_* - Identified and fixing dominant contributor
- `1570` - REASON_DIAG_NO_IMPROVEMENT - No test showed significant improvement
- `1580` - REASON_DIAG_TARGET_REACHED - Motor RMS within acceptable limits
- `1590` - REASON_DIAG_MAX_ITERATIONS - Hit maximum iteration limit

### Runtime State Extensions
```c
// Added to autotuneRuntime_t:
hoverDiagPhase_e diagPhase;           // Current diagnostic phase
uint8_t diagIteration;                 // Iteration counter (max 5)
float diagRms[HOVER_DIAG_PHASE_COUNT]; // RMS per phase
float diagImprovement[HOVER_DIAG_PHASE_COUNT]; // Improvement %

struct {
    uint8_t rollP, rollI, rollD;
    uint16_t rollF;
    uint8_t pitchP, pitchI, pitchD;
    uint16_t pitchF;
    uint16_t gyroLpf1Hz, dtermLpf1Hz;
    bool gyroLpf1IsDynamic, dtermLpf1IsDynamic;
} savedSettings;
```

### Debug Output (debug_mode = AUTOTUNE)
```
[0] = state
[1] = diagPhase * 10 + diagIteration (e.g., 32 = phase 3, iteration 2)
[2] = baseline motor RMS * 10
[3] = current/last motor RMS * 10
[4] = Roll improvement %
[5] = Pitch improvement %
[6] = Filter improvement (positive = gyro, negative = dterm)
[7] = reason code
```

## Configuration Constants

```c
#define DIAG_FILTER_STEP_HZ         50    // Larger step for clear differentiation
#define DIAG_IMPROVEMENT_THRESHOLD  10.0f // 10% improvement to be significant
#define DIAG_MAX_ITERATIONS         5     // Max diagnostic cycles
#define MOTOR_RMS_TARGET            15.0f // Target motor RMS
```

## Files Modified

### autotune_types.h
- Added `hoverDiagPhase_e` enum for diagnostic phases
- Added reason codes 1500-1590 for diagnostic states
- Added `DIAG_*` configuration constants
- Extended `autotuneRuntime_t` with diagnostic state and saved settings

### autotune.c
- Added `saveDiagnosticSettings()` - Save current PID/filter values
- Added `restoreDiagnosticSettings()` - Restore after each test
- Added `applyDiagnosticTest()` - Apply test modification for each phase
- Added `applyDiagnosticFix()` - Apply permanent fix to dominant contributor
- Replaced `updateHoverTune()` with `updateHoverDiagnostic()` state machine
- Updated debug output to show diagnostic progress and improvement percentages
- Updated initialization to set up diagnostic state

## Flight Test Results

The procedural diagnostic approach showed clear improvements:

1. **Accurate identification** - Correctly identified dominant noise sources
2. **Iterative refinement** - Each fix verified before moving to next
3. **No overcorrection** - Stops when no improvement detected
4. **Faster convergence** - ~2.5s per iteration vs arbitrary filter stepping
5. **Clear visibility** - Debug shows improvement % for each parameter

## Next Steps

Potential enhancements:
- Add LPF2 filter tests for deeper diagnosis
- Add D-gain test (separate from full axis halving)
- Variable test magnitudes (0.5x, 0.75x) for finer control
- Extend to in-flight PID response tuning

## Conclusion

The procedural diagnostic hover tune provides a scientific, data-driven approach to filter/gain tuning that replaces guesswork with direct measurement. By applying experimental methodology (change one variable, measure, compare), we can reliably identify and fix the actual noise sources rather than relying on heuristic thresholds.
