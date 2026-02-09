# FF Autotune v3 Implementation Progress

> **Last Updated:** 2026-02-08
> **Branch:** `ff-autotune-v1`
> **Commits:** `dc39e2eea` (PRD), `78ae93515` (Phase 1 Implementation)

---

## Summary

| Category | Planned | Implemented | Status |
|----------|---------|-------------|--------|
| Phase 1 Functional Requirements | 8 | 8 | Complete |
| Phase 1 CLI Parameters | 11 | 11 | Complete |
| Phase 1 Debug Channels | 8 | 8 | Complete |
| Phase 2 Functional Requirements | 6 | 6 | Complete |
| Phase 2 CLI Parameters | 13 | 13 | Complete |
| Phase 2 Debug Channels | 8 | 8 | Complete |
| New Files | 4 | 4 | Complete |
| Modified Files | 9 | 9 | Complete |
| Bug Fixes | 2 | 2 | Complete |

**Overall Status: PHASE 2 IMPLEMENTATION COMPLETE + BUG FIXES APPLIED - READY FOR FLIGHT TEST**

---

## Phase 1: F-Term Tuning (Previously Complete)

### FR1: Error-Based D-Term - COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Switch D-term from gyro-based to error-based when active | Done | pid.c L1410-L1423 |
| Store previous setpoint for error derivative | Done | `static float previousErrorRate[XYZ_AXIS_COUNT]` |
| Only affects roll/pitch when tuning active | Done | `if (ffAutotuneIsActive() && axis <= FD_PITCH)` |

### FR2: Setpoint Tracking Monitor - COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Detect tracking window | Done | ff_autotune.c `updateAxisTracking()` |
| Trigger on acceleration threshold | Done | `min_accel * 100.0f` |
| Accumulate tracking error during window | Done | `state->errorAccumulator += trackingError` |
| State machine: IDLE->RISING->ADJUSTING->WAITING | Done | `ffWindowState_e` enum |
| Minimum 50 samples for valid maneuver | Done | `FF_AUTOTUNE_MIN_SAMPLES = 50` |

### FR3: Gain Adjustment Logic - COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Lag -> increase FF | Done | `calculateNextGain()` |
| Lead -> decrease FF | Done | `calculateNextGain()` |
| Binary search when bracketed | Done | `(lowerGain + upperGain) / 2` |
| Stop adjustment when converged | Done | `FF_BRACKET_CONVERGED` |

### FR4-FR8: Per-Axis Learning, Mode Control, EEPROM, Debug, History - ALL COMPLETE

---

## Phase 2: P/D Ratio Tuning (NEW)

### FR9: Ringing Analysis Window - COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Open analysis window at end of RISING state | Done | ff_autotune.c `ringWindowOpen()` at RISING->ADJUSTING transition |
| Configurable window duration (default 150ms) | Done | `ring_window_ms` CLI parameter |
| Window closes on duration expiry or state transition | Done | Timer check in `updateAxisTracking()` + close in WAITING->IDLE |
| Signed error tracking (gyro - setpoint) | Done | `signedError = gyroRate - setpoint` |

### FR10: Ringing Detection - COMPLETE (Schmitt Trigger Fix Applied)

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Zero-crossing count with deadband | Done | `ringWindowAccumulate()` with **Schmitt trigger hysteresis** (see Bug Fix #1 below) |
| Skip first overshoot peak | Done | `ringFirstPeakPassed` flag, first crossing marks transition |
| Peak-to-peak amplitude after first peak | Done | `ringPeakPos - ringPeakNeg` tracked after first peak |
| Combined ringing score (crossings x avg amplitude) | Done | `ringWindowClose()` computes score |
| Three-level assessment (WELL_DAMPED / MILD / RINGING) | Done | `assessRinging()` with threshold and threshold/2 |

### FR11: Phase 2a P-Term Adjustment - COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Decrease P when ringing detected | Done | `pAdjustment -= p_step` in `processPhase2Ringing()` |
| P bracket via ringing/well-damped boundary | Done | `ringLowerP` / `ringUpperP` tracking |
| Binary search within P bracket | Done | `(ringLowerP + ringUpperP) / 2` |
| Convergence when bracket width <= step | Done | `bracketWidth <= pStep` check |
| P reduction capped by `p_adjust_max` | Done | `pAdjMax = -(int16_t)p_adjust_max` limit |

### FR12: Phase 2b D-Term Adjustment (Fallback) - COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Enter Phase 2b when P hits reduction limit | Done | `state->adjustingD = true` when `newP < pAdjMax` |
| Increase D by `d_step` when ringing persists | Done | `dAdjustment += dStep` |
| D increase capped by `d_adjust_max` | Done | `newD <= dAdjMax` check |
| Converge when well-damped or D limit reached | Done | Sets `FF_BRACKET_CONVERGED` |

### FR13: Phase 3 F-Term Spot Check - COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Activate after Phase 2 convergence | Done | `state->phase = FF_AUTOTUNE_PHASE3_RECHECK` |
| Collect 3 tracking error maneuvers | Done | `FF_PHASE3_RECHECK_COUNT = 3` |
| Compare against Phase 1 converged error | Done | `errorDrift = recheckAvg - recheckAvgError` |
| If within deadband: mark COMPLETE | Done | `state->phase = FF_AUTOTUNE_COMPLETE` |
| If drifted: re-open Phase 1 bracket | Done | `state->phase = FF_AUTOTUNE_PHASE1_FF` with narrow bracket |

### FR14: P/D Override in PID Controller - COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| P-term adjustment applied as delta | Done | pid.c L1345-L1349: `pidData[axis].P += PTERM_SCALE * (pAdj * 0.01f) * errorRate * tpa` |
| D-term adjustment applied as delta | Done | pid.c L1430-L1434: `preTpaD += DTERM_SCALE * (dAdj * 0.01f) * delta` |
| Only affects roll/pitch | Done | `axis <= FD_PITCH` guard |
| Adjustments stored as deltas from base PID | Done | Config stores `p_adj_roll/pitch`, `d_adj_roll/pitch` |

---

## Phase 2 CLI Parameters

### Phase 2a: P/D Balance (Ringing)

| Parameter | Default | Range | Status |
|-----------|---------|-------|--------|
| `ff_autotune_pd_enabled` | ON | OFF/ON | Done |
| `ff_autotune_ring_window_ms` | 150 | 50-250 | Done |
| `ff_autotune_ring_threshold` | 20 | 5-100 | Done |
| `ff_autotune_ring_deadband` | 5 | 2-20 | Done |
| `ff_autotune_p_step` | 2 | 1-5 | Done |
| `ff_autotune_d_step` | 1 | 1-3 | Done |
| `ff_autotune_p_adjust_max` | 10 | 2-20 | Done |
| `ff_autotune_d_adjust_max` | 5 | 1-10 | Done |
| `ff_autotune_p_adj_roll` | 0 | -20..0 | Done |
| `ff_autotune_p_adj_pitch` | 0 | -20..0 | Done |
| `ff_autotune_d_adj_roll` | 0 | 0..10 | Done |
| `ff_autotune_d_adj_pitch` | 0 | 0..10 | Done |

### Phase 2b: P/D Scale-Down (Noise)

| Parameter | Default | Range | Status |
|-----------|---------|-------|--------|
| `ff_autotune_noise_threshold` | 10 | 5-30 | Done |
| `ff_autotune_scale_step` | 1 | 1-3 | Done |
| `ff_autotune_scale_max` | 8 | 2-15 | Done |
| `ff_autotune_scale_adj_roll` | 0 | -15..0 | Done |
| `ff_autotune_scale_adj_pitch` | 0 | -15..0 | Done |

> **Note:** `ff_autotune_noise_window_ms` was removed in v3. Noise is now measured during the RISING phase (|D-term| accumulation) rather than a separate post-maneuver window.

---

## Debug Channels

Debug mode: `DEBUG_FF_AUTOTUNE` (single consolidated mode for all phases)

### Universal Channels (always valid)

| Channel | Name | Description |
|---------|------|-------------|
| `debug[0]` | Gain | Current FF gain (0-200) |
| `debug[1]` | Tracking Error | Instantaneous `fabsf(gyroRate) - fabsf(setpoint)` |
| `debug[2]` | Window State | 0=IDLE, 1=RISING, 2=ADJUSTING, 3=WAITING |
| `debug[3]` | Phase | 0=Phase1_FF, 1=Phase2_PD, 2=Phase2b_Scale, 3=Phase3_Recheck, 4=Complete |

### Phase-Multiplexed Channels (meaning depends on debug[3])

| debug[3] | debug[4] | debug[5] | debug[6] | debug[7] |
|----------|----------|----------|----------|----------|
| 0 (FF) | lastAvgError | bracketState | lastAssessment | historyCount |
| 1 (PD) | ringingScore | pAdjustment | dAdjustment | ringAssessment |
| 2 (Scale) | noiseScore | scaleAdjustment | noiseBaseline | noiseAssessment |
| 3 (Recheck) | recheckStatus | errorDrift | 0 | 0 |
| 4 (Complete) | 0 | 0 | 0 | 0 |

See `DEBUG_PHASE2.md` for full details.

---

## Files Modified for Phase 2

| File | Modification | Status |
|------|--------------|--------|
| `src/main/flight/ff_autotune.h` | Phase 2 enums (incl. Phase2b, noise assessment), ringing assessment, new API functions | Done |
| `src/main/flight/ff_autotune.c` | Ringing analysis (Schmitt trigger), P/D adjustment, Phase 2b noise scale-down, Phase 3 recheck (~1219 lines) | Done |
| `src/main/pg/ff_autotune.h` | Phase 2a + 2b config fields (ringing, noise, scale) | Done |
| `src/main/pg/ff_autotune.c` | Phase 2 defaults, PG version 3 | Done |
| `src/main/cli/settings.c` | Phase 2a + 2b CLI parameter entries | Done |
| `src/main/flight/pid.c` | P-term override, D-term override, scale adjustment additive | Done |

---

## Build Status

| Target | Status | Notes |
|--------|--------|-------|
| BETAFPVG473 | **PASS** | Clean build, 73.5% FLASH1 usage. Verified 2026-02-08 |

---

## Bug Fixes (2026-02-08)

### Bug Fix #1: Schmitt Trigger Zero-Crossing Detection

**Problem:** The original zero-crossing detection in `ringWindowAccumulate()` compared consecutive sample values against the deadband: `prevPositive = (prevError > deadband)` and `currNegative = (error < -deadband)`. A continuous signal transitions gradually through the deadband zone (e.g., +6 → +3 → 0 → -3 → -6), so by the time the current sample reaches `-deadband`, the previous sample is already in the dead zone — not above `+deadband`. Every single zero-crossing was invisible.

**Evidence:** V4 flight log showed Phase 2a ran 11 maneuvers with **zero** ringing scores despite gyro visibly oscillating with up to 25 raw zero-crossings per maneuver.

**Fix:** Replaced with Schmitt trigger (hysteresis latch). A new `int8_t ringLastSide` field remembers which side of the deadband the signal was last on. A crossing is only counted when the signal reaches the opposite threshold. Inside the deadband, `ringLastSide` retains its value.

**Result:** Same v4 log data now correctly shows 8 of 11 maneuvers as RINGING (scores 30-590), matching what's visible in the blackbox viewer.

### Bug Fix #2: Phase 2b Noise Measurement During RISING

**Problem:** Original Phase 2b measured |D-term| in a separate post-maneuver window (`noise_window_ms`). This measured D-term doing its job (damping after the rise) rather than the noise we want to reduce.

**Fix:** Removed `noise_window_ms` parameter. Phase 2b now accumulates `fabsf(pidData[axis].D)` during the RISING phase alongside the tracking error. Processed at the RISING→ADJUSTING transition.

---

## Known Deviations from Phase 2 PRD

1. **Ringing score scaling** - PRD specified "x10" but implementation uses raw `crossings * avgAmplitude` without explicit x10 since the amplitude is already in deg/s. Threshold values may need adjustment during flight test.

2. **First overshoot skip** - PRD suggested skipping the "first peak" by value. Implementation uses Schmitt trigger zero-crossing detection: the first crossing after window open transitions from "first peak" to "ringing" tracking. This is more robust against varying overshoot shapes.

3. **Noise guard for D increase** (PRD section 6.5) - Not yet implemented. Deferred to flight test validation. If D increase causes audible motor noise, manual rollback via CLI is possible.

4. **P safety floor** (PRD section 6.1) - The `p_adjust_max` parameter cap provides this implicitly. The effective P is `base_P + adjustment`, where adjustment is clamped to `[-p_adjust_max, 0]`.

5. **Phase 2b noise measurement** - PRD specified a separate post-maneuver window. Implementation measures during RISING instead, which better captures the D-term noise during dynamic conditions rather than the D-term's damping action.

---

## Flight Test Results

### V4 Test Flight 1 (2026-02-08, pre-Schmitt-fix firmware)

- Phase 1: 33.2s, 16 maneuvers, FF gain converged 0→38→43→40
- Phase 2a: 18.8s, 11 maneuvers, **zero** P/D adjustments (zero-crossing bug)
- Did not reach Phase 2b or Phase 3
- Total flight: 51.9s, 27 maneuvers

> Firmware with Schmitt trigger fix has been built but not yet flight-tested.

---

## Next Steps

1. **Flight test with Schmitt trigger fix** - Phase 2a should now detect ringing and begin P adjustment
2. **Validate Phase 2b noise reduction** - Confirm |D-term| during RISING correlates with perceived noise
3. **Tune default thresholds** - `ring_threshold=20` may need adjustment based on real ringing scores
4. **Concurrent assessment redesign** - Future: measure all aspects (FF, ringing, noise) on every maneuver and adjust the worst aspect first, instead of sequential phases

### Debug Reference

See `DEBUG_PHASE2.md` for Blackbox analysis guide.

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2026-02-05 | 1.0 | Initial progress document - Phase 1 implementation complete |
| 2026-02-07 | 2.0 | Phase 2 P/D ratio tuning implementation complete |
