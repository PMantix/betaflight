# Milestone 4 Roadmap: Path to Critical Damping

## Objective
Transform autotune from "good enough noise reduction" to **optimal critical damping** with snappy, controlled response.

---

## Current State Assessment

| Category | Status | Quality |
|----------|--------|---------|
| Noise Reduction | ✅ Working | Good - hover DOE reduces motor RMS |
| F-term Tracking | ⚠️ Over-active | F-biased - starving P/D tuning |
| P/I/D Adjustment | ❌ Broken | F attribution blocks P/D changes |
| Filter Tuning | ⚠️ Partial | LPF only, separate mode |
| Critical Damping | ❌ Missing | Uses overshoot proxy only |
| Resonance Handling | ❌ Missing | Detected but not fixed |
| Multi-variable DOE | ✅ Partial | Works for hover, not PID |

---

## Feature Tasks

### P0: Fix Attribution Order (CRITICAL - NEW)
**Priority:** CRITICAL - BLOCKING  
**Effort:** Low (30 min)  
**Impact:** Critical - unblocks P/D tuning  
**Files:** `src/main/flight/autotune_analysis.c`

**Problem:** Attribution logic checks F-term issues BEFORE P/D response classification. Since almost every stick move triggers `velocityWeightedLag > 5.0`, F is attributed exclusively and P/D never get adjusted.

**Evidence from flight test:**
- F changed 21 times (bouncing)
- P changed 1 time
- D changed 0 times

**Solution:** Reorder attribution phases so P/D response classification happens FIRST.

**Implementation Steps:**
1. In `autotuneAttributeResponse()` (~line 480-680):
2. Move Phase 3 (P/D response classification) BEFORE Phase 2 (F-term issues)
3. Only check F-term if response class is EXCELLENT or CRITICAL
4. Add early return after P/D attribution for UNDERDAMPED/OVERDAMPED

**Code Change:**
```c
// BEFORE (broken order):
// Phase 1: I-term bounceback/oscillation
// Phase 2: F-term lag/lead detection  <-- triggers too often!
// Phase 3: P/D based on response class

// AFTER (fixed order):
// Phase 1: I-term bounceback/oscillation (unchanged)
// Phase 2: P/D based on response class (moved UP)
//   - If UNDERDAMPED → attribute to P/D, return
//   - If OVERDAMPED → attribute to P/D, return
// Phase 3: F-term lag/lead (only for EXCELLENT/CRITICAL response)
```

**Acceptance Criteria:**
- [ ] P/D attributed for UNDERDAMPED/OVERDAMPED responses
- [ ] F attributed only when P/D response is already good
- [ ] Build passes
- [ ] Flight test shows balanced P/D/F adjustments

---

### P1: Multi-Variable DOE for Hover Tune
**Priority:** CRITICAL  
**Effort:** Medium  
**Impact:** High - faster convergence  
**Files:** `src/main/flight/autotune.c`
**Status:** ✅ PARTIAL (hover mode works, PID mode needs extension)

**Problem:** Current approach applies ONE fix per cycle, wastes information from 4 tests.

**Solution:** Implement linear combination model that uses ALL sensitivities simultaneously.

**What's Done:**
- `applyMultiVariableFix()` implemented and working for hover mode
- Sensitivities calculated from DOE tests
- 90% damping applied

**What's Missing:**
- Types not added to autotune_types.h (using local declarations)
- Not used in PID tuning mode

---

### P2: Bidirectional Gain Exploration
**Priority:** HIGH  
**Effort:** Low  
**Impact:** High - finds optimal, not minimum  
**Files:** `src/main/flight/autotune.c`

**Problem:** Hover tune only tests REDUCING gains/filters, never RAISING them.

**Solution:** Add "relaxation" phase that tests raising parameters when noise is excellent.

**Implementation Steps:**
1. After standard DOE, if baseline RMS < `MOTOR_RMS_EXCELLENT` (8.0):
2. Run relaxation tests:
   - Test 1.5× Roll gains
   - Test 1.5× Pitch gains  
   - Test +50Hz on Gyro LPF1
   - Test +25Hz on D-term LPF1
3. If noise stays acceptable (< 15 RMS), apply the raises
4. This pushes toward optimal, not just "good enough"

**New Phases:** `HOVER_DIAG_RELAX_ROLL`, `HOVER_DIAG_RELAX_PITCH`, `HOVER_DIAG_RELAX_GYRO_LPF1`, `HOVER_DIAG_RELAX_DTERM_LPF1`

**Acceptance Criteria:**
- [ ] Relaxation tests run when RMS < 8.0
- [ ] Parameters raised when headroom exists
- [ ] Reason codes 114x output for relaxation actions
- [ ] D gain can increase during hover tune

---

### P3: Critical Damping Detection
**Priority:** MEDIUM  
**Effort:** Low  
**Impact:** Medium - better P/D decisions  
**Files:** `src/main/flight/autotune_analysis.c`, `src/main/flight/autotune_types.h`

**Problem:** System uses overshoot % as proxy for damping, no actual damping ratio calculation.

**Solution:** Calculate damping ratio (ζ) from step response characteristics.

**Implementation Steps:**
1. Add damping ratio calculation from overshoot:
   ```c
   // ζ = -ln(OS/100) / sqrt(π² + ln²(OS/100))
   float calculateDampingFromOvershoot(float overshootPercent);
   ```
2. Add damping ratio from oscillation decay:
   ```c
   // ζ = ln(A1/A2) / sqrt(4π² + ln²(A1/A2))
   float calculateDampingFromDecay(float peak1, float peak2);
   ```
3. Target ζ = 0.7-0.8 for optimal response (slight underdamping for snappy feel)
4. Add `dampingRatio` to `autotuneMetrics_t`
5. Use ζ to make P/D ratio decisions, not just overshoot

**Acceptance Criteria:**
- [ ] `calculateDampingRatio()` function implemented
- [ ] Damping ratio stored in metrics structure
- [ ] Response classification uses ζ thresholds
- [ ] P/D adjustment considers damping ratio

---

### P4: P/D Push-Up Logic
**Priority:** MEDIUM  
**Effort:** Medium  
**Impact:** High - snappier response  
**Files:** `src/main/flight/autotune_gains.c`

**Problem:** System rarely increases P or D even when there's headroom.

**Solution:** Add explicit "push gains up until noise limit" phase.

**Implementation Steps:**
1. After initial tune converges, check headroom:
   - Motor RMS < 20 (noise headroom exists)
   - Overshoot < 12% (can tolerate more)
   - No oscillation detected
2. Try +15% P, measure response
3. If acceptable, keep. If oscillation, try +15% D to compensate
4. Repeat until hitting noise limit or overshoot target (10-15%)
5. Target: Response that's crisp (near-zero rise time) with 8-12% overshoot

**Key Function:** `autotunePushGainsUp()`

**Acceptance Criteria:**
- [ ] Push-up phase triggers after convergence
- [ ] P increased when headroom exists
- [ ] D compensation applied if P causes oscillation
- [ ] Stops at noise limit or target overshoot

---

### P5: Integrated Filter+PID Tuning
**Priority:** LOW  
**Effort:** Medium  
**Impact:** Medium - less latency  
**Files:** `src/main/flight/autotune_gains.c`

**Problem:** Filter tune and PID tune are separate modes - no coordinated optimization.

**Solution:** During PID tune, if noise exceeds threshold, lower filters inline.

**Implementation Steps:**
1. During PID analysis, check noise RMS
2. If noise > 25 (high), reduce D-term LPF by 10Hz before adjusting PID
3. If noise < 10 (excellent), raise D-term LPF by 10Hz to reduce latency
4. Log filter changes in debug channel
5. Ensure filter limits respected

**Acceptance Criteria:**
- [ ] Filter adjustment during PID mode
- [ ] Reason codes 2xxx output for inline filter changes
- [ ] Latency reduced when noise permits

---

### P6: Notch Filter Configuration
**Priority:** LOW  
**Effort:** Medium  
**Impact:** Medium - resonance handling  
**Files:** `src/main/flight/autotune_gains.c`

**Problem:** Resonances are detected (`suggestedNotchHz`) but never fixed.

**Solution:** Automatically configure D-term or gyro notch when resonance detected.

**Implementation Steps:**
1. If `resonanceDetected && peakAmplitude > threshold`:
2. Check if resonance frequency is above LPF cutoff (can't filter with LPF alone)
3. Configure `dterm_notch_hz = suggestedNotchHz`
4. Configure `dterm_notch_cutoff = suggestedNotchHz * 0.7`
5. Re-run noise measurement to verify improvement
6. If no improvement, revert notch

**Key Function:** `autotuneApplyNotchFix()`

**Acceptance Criteria:**
- [ ] Notch applied when resonance detected above LPF
- [ ] Notch Q/cutoff configured appropriately
- [ ] Verification measurement after notch
- [ ] Revert if no improvement

---

## Implementation Order

| Phase | Task | Dependencies | Est. Effort |
|-------|------|--------------|-------------|
| 4a | P1: Multi-Variable DOE | None | 2-3 hours |
| 4b | P2: Bidirectional Exploration | None | 1-2 hours |
| 4c | P3: Critical Damping Detection | None | 1 hour |
| 4d | P4: P/D Push-Up Logic | P3 | 2 hours |
| 4e | P5: Integrated Filter+PID | P1 | 2 hours |
| 4f | P6: Notch Configuration | None | 2 hours |

**Total Estimated Effort:** 10-12 hours

---

## Success Metrics

1. **Convergence Speed:** Hover tune completes in ≤3 iterations (vs 5+ today)
2. **Final Damping Ratio:** ζ = 0.7-0.8 achieved consistently
3. **Overshoot Range:** 8-12% on step response
4. **Motor Noise:** RMS < 15 maintained while maximizing gains
5. **D Gain Utilization:** D increased when noise headroom exists
