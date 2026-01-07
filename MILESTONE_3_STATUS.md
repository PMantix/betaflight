# Milestone 3: Autotune Implementation Status

**Last Updated:** January 6, 2026

## Current Status: Phase 0 Sweep Data Collection Complete

### What's Working

✅ **Phase 0 Sweep Implementation**
- Starts at D = 30% of P, increases by 15% per iteration
- Sweeps from D/P ratio 0.30 to 2.31 (15 iterations)
- Successfully collects comprehensive metrics for each iteration
- Stops at iteration 15 with TUNE_PHASE_COMPLETE

✅ **Metrics Collection**
- Rise time (10-90%)
- Rise time to setpoint (first entry to ±5% band) - NEW
- Tracking error (avg error after reaching setpoint) - NEW
- Overshoot percentage
- D-term oscillation amplitude (RMS)
- D-term oscillation frequency
- Gyro oscillation frequency
- Gyro noise RMS

✅ **Debug Output**
- `DEBUG_AUTOTUNE_METRICS` mode outputs all metrics
- Channel 0: State (4 = ANALYZE)
- Channel 1: Iteration number
- Channel 2: Current P gain
- Channel 3: Current D gain
- Channel 4: Overshoot (% × 10)
- Channel 5: D-term oscillation amplitude (× 10)
- Channel 6: Rise time to setpoint (ms × 10)
- Channel 7: Tracking error (deg/s × 10)

✅ **Analysis Tooling**
- Python script `analyze_phase0_metrics.py` extracts and plots all metrics
- 6-panel visualization showing progression of all key metrics
- Composite plot showing rise time vs tracking error colored by D-term oscillation

### Test Results (btfl_0023)

**Optimal Range Identified:** Iterations 8-10 (D/P ratio 0.75-0.91)
- Fast rise time: 49-51ms
- Low tracking error: 90-98 deg/s
- Moderate D-term oscillation: 91-97
- Appropriate overshoot: 5-7%

**Performance Degradation:** Iterations 12+ (D/P > 1.06)
- Rise time increases: 65-137ms (slower response)
- Tracking error increases: 132-169 deg/s (worse stability)
- D-term oscillation increases: 94-381 (excessive D)

**Unflyable Region:** Iterations 24-28 (D/P > 2.0)
- D-term oscillation: 208-381 (pilot-reported unflyable)
- Overshoot erratic: 7-20%

### Code Cleanup Completed

✅ Removed old broken classification system:
- Deleted `dampingState_e` enum
- Deleted `analyzeDamping()` function (~90 lines)
- Deleted `classifyDamping()` function (~45 lines)
- Deleted `calculateSettlingTime()` function (too strict, always 999ms)
- Removed struct fields: `zeroCrossingsAfterPeak`, `oscillationDecayRatio`, `dampingState`

✅ Simplified Phase 0 logic:
- Removed both classification-based early exits
- Sweep now runs for full 15 iterations for data collection
- Status code shows iteration number (1-15) instead of unreliable classification

## Next Implementation: Automated Scoring and Early Stopping

### Objective
Replace manual assessment with automated scoring function that identifies optimal P/D ratio and stops sweep before entering unflyable region.

### Implementation Plan

#### 1. Scoring Function (`calculateResponseScore()`)

**Location:** `src/main/flight/autotune.c` (after `calculateTrackingError()`)

**Algorithm:**
```c
score = 0.0f (start perfect)
penalties:
  - Tracking error × 0.4  (40% weight - stability most important)
  - D-term osc × 0.3      (30% weight - overdamping indicator)
  - Rise time × 0.2       (20% weight - responsiveness)
  - Overshoot deviation × 0.1  (10% weight - target 5-15%)
Lower score = better performance
```

**Expected scores:**
- Optimal (iter 8-10): ~100-150 points
- Degraded (iter 12-20): ~150-200 points
- Unflyable (iter 22+): >200 points

#### 2. History Storage

**Changes:**
- Add `float score` to `iterationHistory_t` struct
- Calculate score in `phase1_establishRatio()` after each analysis
- Store in `runtime.history[iteration].score`
- Output to debug channel 7 for visualization

#### 3. Early Stopping Detection

**Trigger Conditions:**
- Minimum 6 iterations completed (ensure we see full progression)
- Score has increased for 2 consecutive iterations
- Current score > (best score × 1.1)  // 10% worse

**Action:**
- Stop sweep immediately
- Find best iteration in history
- Apply best gains
- Transition to Phase 2

**Status codes:**
- 103: Optimal found, early stop
- 104: Score degrading trend detected

#### 4. Best Iteration Selection

**Function:** `findBestSweepIteration()`
- Search `runtime.history[]` for lowest score
- Return iteration number of best score
- Apply those P/D gains from history

**Debug output:**
- debug[1]: Best iteration number
- debug[6]: Best score value

#### 5. Phase Transition

**On sweep completion:**
- Apply best gains (not current gains!)
- Set `progress->establishedRatio` from best iteration
- Set `progress->phase = TUNE_PHASE_SCALE_UP`
- Status 50: Phase 0 complete
- Beep success tone

### Implementation Steps

1. Add scoring function (~50 lines)
2. Add `score` field to struct
3. Update `phase1_establishRatio()` to calculate and store scores
4. Implement `findBestSweepIteration()` helper
5. Add early stopping logic with score monitoring
6. Update phase completion to apply best gains
7. Test with new blackbox log

### Calibration Considerations

**Scoring weights** (40/30/20/10):
- Based on btfl_0023 manual assessment
- May need adjustment if doesn't identify iter 8-10 as optimal
- Could make CLI-configurable: `set autotune_score_weights = 40,30,20,10`

**Early stopping threshold:**
- Current: 2 consecutive worse + 10% degradation
- Alternative: 3-iteration rolling average
- Alternative: Fixed score threshold (>200 = stop)
- May need tuning based on real-world variability

**Minimum iterations:**
- Current: 6 iterations minimum
- Ensures seeing underdamped → optimal progression
- Could reduce to 4 if scoring converges quickly

## Files Changed

### Core Implementation
- `src/main/flight/autotune.c` - Main autotune logic
- `src/main/flight/autotune.h` - Public API
- `src/main/pg/autotune.c` - Configuration
- `src/main/pg/autotune.h` - Configuration parameters

### Debug Support
- `src/main/build/debug.c` - Added DEBUG_AUTOTUNE_METRICS mode
- `src/main/build/debug.h` - Debug mode enum

### Analysis Tools
- `bb logs/analyze_phase0_metrics.py` - Metric extraction and visualization

## Git Status

**Branch:** master  
**Last Commit:** (to be updated after commit)

## Known Issues / TODO

- [ ] Implement automated scoring function
- [ ] Implement early stopping logic
- [ ] Test scoring weights with btfl_0023 data
- [ ] Validate early stopping prevents unflyable region
- [ ] Phase 2 (Scale Up) not yet implemented
- [ ] Phase 3 (Fine Tune) not yet implemented
- [ ] I-term tuning not implemented
- [ ] F-term tuning (requires stick movement) not implemented

## Success Criteria

Phase 0 scoring will be considered successful when:
1. ✅ Automated scoring identifies iteration 8-10 as optimal in btfl_0023 data
2. ✅ Early stopping triggers before entering unflyable region (iter 22+)
3. ✅ Applied gains match manually-identified optimal gains
4. ✅ Multiple test flights confirm consistent optimal detection
5. ✅ No false positives (stopping too early in underdamped region)
