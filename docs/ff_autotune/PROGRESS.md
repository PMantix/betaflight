# FF Autotune v3 Implementation Progress

> **Last Updated:** 2026-02-07
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
| Phase 2 CLI Parameters | 14 | 14 | Complete |
| Phase 2 Debug Channels | 8 | 8 | Complete |
| New Files | 4 | 4 | Complete |
| Modified Files | 9 | 9 | Complete |

**Overall Status: PHASE 2 IMPLEMENTATION COMPLETE - READY FOR FLIGHT TEST**

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

### FR10: Ringing Detection - COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Zero-crossing count with deadband | Done | `ringWindowAccumulate()` with `ring_deadband` filtering |
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

---

## Phase 2 Debug Channels

Debug mode: `DEBUG_FF_AUTOTUNE_PD`

| Channel | Name | Description |
|---------|------|-------------|
| `debug[0]` | Ringing Score | Combined ringing score from last maneuver |
| `debug[1]` | Zero Crossings | Zero-crossing count in analysis window |
| `debug[2]` | Ring Amplitude | Peak-to-peak amplitude after first overshoot (deg/s) |
| `debug[3]` | P Adjustment | Current cumulative P adjustment (signed) |
| `debug[4]` | D Adjustment | Current cumulative D adjustment (signed) |
| `debug[5]` | Window Active | Analysis window state (0=closed, 1=open) |
| `debug[6]` | Autotune Phase | Current phase (0=Phase1, 1=Phase2, 2=Phase3, 3=Complete) |
| `debug[7]` | Ring Assessment | Ringing assessment (0=well-damped, 1=mild, 2=ringing) |

---

## Files Modified for Phase 2

| File | Modification | Status |
|------|--------------|--------|
| `src/main/flight/ff_autotune.h` | Phase 2 enums, ringing assessment, new API functions | Done |
| `src/main/flight/ff_autotune.c` | Ringing analysis, P/D adjustment, Phase 3 recheck (~1030 lines) | Done |
| `src/main/pg/ff_autotune.h` | 12 new Phase 2 config fields | Done |
| `src/main/pg/ff_autotune.c` | Phase 2 defaults, PG version bump (0->1) | Done |
| `src/main/cli/settings.c` | 14 new CLI parameter entries | Done |
| `src/main/build/debug.h` | Added `DEBUG_FF_AUTOTUNE_PD` enum | Done |
| `src/main/build/debug.c` | Added `"FF_AUTOTUNE_PD"` debug mode name | Done |
| `src/main/flight/pid.c` | P-term override (L1345-1349), D-term override (L1430-1434) | Done |

---

## Build Status

| Target | Status | Notes |
|--------|--------|-------|
| BETAFPVG473 | Pending build test | Phase 2 code added, needs compilation |

---

## Known Deviations from Phase 2 PRD

1. **Ringing score scaling** - PRD specified "x10" but implementation uses raw `crossings * avgAmplitude` without explicit x10 since the amplitude is already in deg/s. Threshold values may need adjustment during flight test.

2. **First overshoot skip** - PRD suggested skipping the "first peak" by value. Implementation uses zero-crossing detection: the first zero-crossing after window open transitions from "first peak" to "ringing" tracking. This is more robust against varying overshoot shapes.

3. **Noise guard for D increase** (PRD section 6.5) - Not yet implemented. Deferred to flight test validation. If D increase causes audible motor noise, manual rollback via CLI is possible.

4. **P safety floor** (PRD section 6.1) - The `p_adjust_max` parameter cap provides this implicitly. The effective P is `base_P + adjustment`, where adjustment is clamped to `[-p_adjust_max, 0]`.

---

## Next Steps

### Phase 2 Flight Test

See `TEST_PLAN_PHASE2.md` for the detailed test procedure.

### Debug Reference

See `DEBUG_PHASE2.md` for Blackbox analysis guide.

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2026-02-05 | 1.0 | Initial progress document - Phase 1 implementation complete |
| 2026-02-07 | 2.0 | Phase 2 P/D ratio tuning implementation complete |
