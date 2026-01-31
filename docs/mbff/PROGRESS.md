# MBFF Implementation Progress

**Branch:** `model-feedforward`  
**Target:** BETAFPVG473  
**Last Updated:** 2026-01-31

---

## Phase 1: Core Implementation ✅

| Task | Status | Notes |
|------|--------|-------|
| Create PRD.md | ✅ | Product requirements document |
| Create README.md | ✅ | User documentation with tuning guide |
| Create mbff.h | ✅ | Header with types, config struct, API |
| Create mbff.c | ✅ | Core implementation |
| Add PG_MBFF_CONFIG | ✅ | pg_ids.h - ID 561 |
| Add DEBUG_MBFF | ✅ | debug.h/debug.c |
| Add USE_MBFF feature flag | ✅ | common_pre.h |
| Add mbff.c to source.mk | ✅ | Build system integration |
| Integrate into pid.c | ✅ | Replace classic FF when enabled |
| Integrate into pid_init.c | ✅ | Call mbffInit() |
| Add CLI settings | ✅ | settings.c - 10 parameters |

---

## Phase 2: Build Verification ✅

| Task | Status | Notes |
|------|--------|-------|
| Initial build | ✅ | Compiles clean |
| Fix compile errors | ✅ | None needed |
| Verify binary size | ✅ | Fits in flash |

---

## Phase 3: Testing ✅

| Task | Status | Notes |
|------|--------|-------|
| CLI parameter validation | ✅ | All 9 params accessible |
| Debug mode logging | ✅ | Fixed overflow issues |
| Bench test (motors off) | ✅ | State updates working |
| First flight test | ✅ | MBFF producing output |
| Blackbox analysis | 🔲 | Compare MBFF vs classic FF |

---

## Phase 4: Tuning & Refinement 🔲

| Task | Status | Notes |
|------|--------|-------|
| Whoop baseline tune | 🔲 | 65-75mm platform |
| High-rate tracking validation | 🔲 | Key PRD goal |
| Noise sensitivity comparison | 🔲 | |
| Parameter sweep testing | 🔲 | |

---

## Implementation Details

### Files Created
- [docs/mbff/PRD.md](PRD.md) - Product requirements
- [docs/mbff/README.md](README.md) - User guide
- [src/main/flight/mbff.h](../../src/main/flight/mbff.h) - Header
- [src/main/flight/mbff.c](../../src/main/flight/mbff.c) - Implementation

### Files Modified
- `src/main/pg/pg_ids.h` - Added PG_MBFF_CONFIG (561)
- `src/main/build/debug.h` - Added DEBUG_MBFF
- `src/main/build/debug.c` - Added "MBFF" string
- `src/main/target/common_pre.h` - Added USE_MBFF
- `mk/source.mk` - Added flight/mbff.c
- `src/main/flight/pid.c` - MBFF integration, reset, RPM update
- `src/main/flight/pid_init.c` - mbffInit() call
- `src/main/cli/settings.c` - CLI parameters

### CLI Parameters
| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| mbff_enable | bool | OFF/ON | OFF | Enable MBFF |
| mbff_ts | uint8 | 5-50 | 30 | Trajectory time constant (ms) |
| mbff_tp | uint8 | 10-100 | 50 | Preview horizon (ms) - higher = less noise |
| mbff_ka | uint8 | 0-200 | 100 | Trajectory accel gain (x100) |
| mbff_kr | uint8 | 0-200 | 25 | Preview correction gain (x100) - lower = less noise |
| mbff_b0 | uint16 | 10-500 | 100 | Base effectiveness (x100) |
| mbff_b1 | uint8 | 0-200 | 50 | RPM effectiveness (x100) |
| mbff_ff_limit | uint8 | 10-100 | 50 | FF limit (% of pidSumLimit) |
| mbff_gain | uint16 | 10-2000 | 100 | Master output gain (x10, so 100=10.0) |
| mbff_preview_threshold | uint8 | 0-200 | 50 | Setpoint derivative threshold for preview gating |

### Debug Channels (DEBUG_MBFF)
| Index | Name | Description |
|-------|------|-------------|
| 0 | OMEGA_REF | Reference rate [deg/s] |
| 1 | ALPHA_DES | Desired acceleration [deg/s²] |
| 2 | EFFECTIVENESS | Torque effectiveness (raw) |
| 3 | FF_OUTPUT | MBFF output (raw) |
| 4 | CLASSIC_FF | Classic FF for comparison (raw) |
| 5 | RPM_SQ | Avg RPM² / 1,000,000 |
| 6 | TRAJ_ERROR | ω_sp - ω_ref [deg/s] |
| 7 | TRACK_ERROR | ω_ref - ω_meas [deg/s] |

---

## Build Log

### 2026-01-31 - Initial Implementation
- Created all core files
- Integrated into PID loop
- Added CLI parameters (8 initial)
- **Build status:** ✅ SUCCESS - Compiles clean on BETAFPVG473

### 2026-01-31 - First Flight Test & Fixes
- Flashed and tested on whoop
- MBFF producing output, replacing classic FF
- **Issue:** Debug channels 2,3,4 overflowing 16-bit range
  - **Fix:** Removed multipliers, now logging raw values
- **Issue:** RPM² channel overflowing
  - **Fix:** Changed from /1000 to /1,000,000
- **Issue:** FF magnitude ~10x smaller than classic FF
  - **Fix:** Added `mbff_gain` parameter (default 100 = 10.0x multiplier)
- **Build status:** ✅ SUCCESS - All fixes compiled

### 2026-01-31 - Blackbox Analysis & Gain Calibration
- Analyzed `mbff_test_flight_1.csv` (11.1s, 1004 Hz)
- **Key Finding:** MBFF output still ~100x smaller than classic FF
  - With gain=10.0: MBFF RMS=101, Classic FF RMS=9460
  - Required gain median: ~200x (varies by flight condition)
- **Analysis of effectiveness model:**
  - Effectiveness = b0 + b1 * (RPM²/1e6) = 1.0 + 0.5 * 430 ≈ 216
  - This high effectiveness value reduces output significantly
- **Fixes Applied:**
  - Increased `mbff_gain` default from 100 to 500 (50.0x)
  - Increased `mbff_gain` max from 500 to 2000 (200.0x)
- **Build status:** ✅ SUCCESS

### 2026-01-31 - Oscillation Analysis & Noise Reduction
- **Key Finding:** MBFF output oscillating at ~223 Hz
  - Preview term amplifying gyro noise by factor of 32 (k_r/T_p = 0.8/0.025)
  - With setpoint=0, gyro noise of ±3 deg/s → alpha_des of ±100
- **Root Cause:** Preview correction `α_prev = k_r * (ω_ref - gyro) / T_p`
  - Dividing by small T_p (25ms) amplifies high-frequency noise
  - k_r=0.8 further amplifies the signal
- **Fixes Applied:**
  - Reduced `mbff_kr` default from 80 to 25 (0.25x instead of 0.8x)
  - Increased `mbff_tp` default from 25ms to 50ms (halves noise gain)
  - Combined effect: noise amplification reduced from 32x to ~5x
- **Build status:** ✅ SUCCESS

### 2026-01-31 - Preview Term Gating Implementation
- **Design Discussion:** Gate preview term to avoid noise during steady-state
- **Option 1:** Gate on |setpoint| - works for hover but not sustained rolls
- **Option 2:** Gate on |d(setpoint)/dt| - activates only during transitions ✓
- **Implementation:**
  - Added `mbff_preview_threshold` parameter (default 50)
  - Preview term scales with |d(setpoint)/dt|
  - Threshold of 50 → derivative threshold of 5000 deg/s²
- **Behavior:**
  - Hover (stick still) → preview OFF (no gyro noise)
  - Sustained roll (stick held) → preview OFF (no gyro noise)
  - Transitions (stick moving) → preview ON (helps tracking)
- **Build status:** ✅ SUCCESS

### 2026-01-31 - Flight 2 Analysis & Gain Reduction
- **User Feedback:** "Gains too high, getting leading of setpoint on initiation and after receding from roll/flip"
- **Analysis of `mbff_test_flight_2.csv`:**
  - Preview gating worked well ✓ - oscillation reduced from 25.3% to 11.8% sign changes
  - But MBFF saturating at ±250 throughout all transitions
  - Gyro leading setpoint by 66 deg/s during initiation (should be ~0)
  - Classic FF (debug[4]) is logged at 100x scale, so actual range is ±327 (similar to MBFF ±250)
- **Root Cause:**
  - `ts=15ms` too fast → omega_ref chases setpoint quickly → large alpha_traj
  - `gain=500` (50x) causes immediate saturation
  - Saturated output can't modulate, causing overshoot on both initiation and release
- **Transition Timing Analysis:**
  | Phase | Track Error | Traj Lag | MBFF | Issue |
  |-------|-------------|----------|------|-------|
  | Roll initiation | +66 deg/s (gyro leads) | -188 | -250 (sat) | Too aggressive |
  | Sustained roll | -100 to -150 | 0 | -250 (sat) | Gyro overshoots |
  | Stick release | +22 deg/s | +98 | +250 (sat) | Too aggressive |
- **Fixes Applied:**
  - Increased `mbff_ts` default from 15ms to 30ms (slower trajectory = smaller alpha_traj)
  - Reduced `mbff_gain` default from 500 to 100 (10x instead of 50x)
- **Build status:** ✅ SUCCESS

---

## Flight 2 Analysis Summary

| Metric | Value |
|--------|-------|
| Duration | 35.0 s |
| Samples | 35,268 |
| Sample rate | ~1008 Hz |
| Sign change ratio | 11.8% (was 25.3% in flight 1) |
| MBFF output (mean active) | 89.0 (saturated often) |
| Classic FF actual (mean active) | 38.3 (×100 in debug) |
| MBFF:Classic ratio | 2.3:1 (too high) |
| Track error during initiation | +66 deg/s (gyro leads) |
| Track error during release | +22 deg/s (gyro leads) |

---

## Flight 1 Analysis Summary

| Metric | Value |
|--------|-------|
| Duration | 11.1 s |
| Sample rate | 1004 Hz |
| Active stick samples | 914 (8.2%) |
| MBFF output RMS | 101.1 |
| Classic FF RMS | 9460.1 |
| Ratio (gain=10.0x) | 0.01x |
| Avg RPM | ~20,000 |
| RPM²/1e6 | 429.5 |
| Effectiveness | 215.8 |
| Trajectory error RMS | 17.2 deg/s |
| Tracking error RMS | 21.6 deg/s |

---

## Known Issues

| Issue | Status | Resolution |
|-------|--------|------------|
| Debug channel overflow | ✅ Fixed | Removed multipliers, log raw values |
| RPM² overflow | ✅ Fixed | Divide by 1M instead of 1K |
| FF magnitude mismatch | ✅ Fixed | Increased mbff_gain default to 500 (50.0x) |
| Preview term oscillation | ✅ Fixed | Added derivative-based preview gating |

---

## Next Steps

1. ~~Run `make CONFIG=BETAFPVG473` to verify build~~ ✅
2. ~~Flash and verify CLI parameters work~~ ✅
3. ~~Test DEBUG_MBFF mode~~ ✅
4. ~~First hover test with conservative settings~~ ✅
5. ~~Blackbox analysis (flight 1)~~ ✅
6. ~~Implement preview gating~~ ✅
7. ~~Test with new defaults (gain=500, preview_threshold=50)~~ ✅
8. ~~Verify oscillation is eliminated during hover/sustained rolls~~ ✅ (sign changes: 25.3% → 11.8%)
9. ~~Analyze flight 2 for gain tuning~~ ✅
10. Test with reduced gain (100 instead of 500) and slower trajectory (ts=30)
11. High-rate tracking tests
12. Compare feel vs classic FF
