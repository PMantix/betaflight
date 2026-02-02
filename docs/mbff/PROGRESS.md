# MBFF Implementation Progress

**Branch:** `model-feedforward`  
**Target:** BETAFPVG473  
**Last Updated:** 2026-02-01

---

## 🎯 Current Status: Time-Scaling Fix Applied

**Critical fix:** The RLS learner now multiplies gyro delta by `pidFrequency` to get angular acceleration (deg/s²). This ensures learned E values match the Python-validated physics model (~0.006 for roll, ~0.003 for pitch).

### Key Discovery: Time-Scaling Mismatch

The original Python analysis used time-scaled derivatives:
```python
gyro_deriv = d(gyro)/dt   # deg/s²
slope = regress(eRPM²_diff, gyro_deriv)  → E ≈ 0.006
```

But firmware was using per-PID-loop deltas:
```c
gyroDelta = gyro[t] - gyro[t-1]   // deg/s per loop, NOT deg/s²
```

At 4kHz PID frequency, per-loop delta is 4000x smaller than angular acceleration!

**Fix:** In RLS update, multiply gyroDelta by pidFrequency:
```c
const float y = gyroDelta * mbffRuntime.pidFrequency;  // Now in deg/s²
```

### Gating Conditions (Updated)

| Condition | Threshold | Notes |
|-----------|-----------|-------|
| Active maneuvering | `|setpoint| > 400 deg/s` | Excludes hover noise |
| Not crashed | `|gyro_product| < 1e6` | Excludes tumble |
| Accelerating | `|gyroDelta| > 4` per PID loop | Per-loop, NOT time-scaled |

**Note:** Gating still uses per-PID-loop delta (threshold=4), but RLS converts to deg/s² for proper E learning.

### Validated E Values

| Axis | E_rpm² (slope) | Units |
|------|----------------|-------|
| Roll | ~0.006 | deg/s² per eRPM²_diff |
| Pitch | ~0.003 | deg/s² per eRPM²_diff |

### Next Steps

1. Implement eRPM²_diff calculation in firmware
2. Basic feedforward: `u_ff = E_rpm² × eRPM²_diff`
3. Add gating logic for online learning
4. Flight test with learned parameters

See [PRD.md Section 13](PRD.md#13-implementation-guide) for detailed implementation guide.

---

## v2 Architecture (Current)

The v2 approach uses a **physics-based eRPM² model** validated by blackbox log analysis.

### Validated Plant Model

Two-stage causal chain:
```
Relationship 1: delta(gyro) ~ eRPM²_diff        (R² = 0.89-0.99)
Relationship 2: eRPM_diff ~ delta(motor_diff)  (R² = 0.62-0.90)
```

**Physics basis:**
- Thrust ∝ RPM² (propeller aerodynamics)
- Motor has inertia (command controls d(RPM)/dt, not RPM)

### eRPM² Differential Formulas

Motor mapping (Betaflight Quad X):
```
M0 = rear-right (CW)    M1 = front-right (CCW)
M2 = rear-left (CCW)    M3 = front-left (CW)
```

```
Roll:  (eRPM[0]² + eRPM[1]²) - (eRPM[2]² + eRPM[3]²)
Pitch: (eRPM[0]² + eRPM[2]²) - (eRPM[1]² + eRPM[3]²)
Yaw:   (eRPM[0]² + eRPM[3]²) - (eRPM[1]² + eRPM[2]²)
```

### Feedforward Computation
```
eRPM_sq_diff = computeERPMSquaredDiff(axis);
u_ff = E_rpm2 * eRPM_sq_diff * ff_scale;
```

### Online Learning
- Learn `E_rpm²` per axis using zero-intercept RLS
- Gating: |setpoint| > 400, |gyro_product| < 1e6, |gyroDelta| > 4 per PID loop
- **Critical:** RLS multiplies gyroDelta by pidFrequency to get deg/s² for proper E values

### Key Insight: Gyro Filter Delay Compensation
The primary benefit of model-based FF is **compensating for gyro filter delay** (typically 4-8ms). By using eRPM telemetry, we know actual motor speed without waiting for gyro response.

---

## Simulation Validation ✅

Simulation confirms the v2 physics-based approach significantly reduces tracking error.

### Results Summary

| Condition | Baseline RMS Error | Learned FF RMS Error | Improvement |
|-----------|-------------------|---------------------|-------------|
| Good PIDs | 27.0 deg/s | 14.7 deg/s | **45%** |
| Bad PIDs | 71.3 deg/s | 25.5 deg/s | **64%** |

### Learning Accuracy
- Effectiveness `E` identified with **<0.2% error**
- Rapid convergence within 2-3 seconds of flight data

### Filter Chain Modeled
- **Gyro filters:** O1@250Hz → O1@150Hz → O2@500Hz → O1@100Hz
- **D-term filters:** O1@75Hz + O2@250Hz
- Combined gyro delay: ~4-8ms depending on frequency content

### Simulation Code
See [docs/mbff/sim_mfbb.py](sim_mfbb.py) for the full simulation.

---

## ⚠️ v1 Approach (DEPRECATED)

The v1 trajectory-based approach with preview correction is **deprecated** due to fundamental issues.

### Why v1 Failed
1. **Noise amplification from preview term**: `k_r/T_p = 0.8/0.025 = 32x` gain on gyro noise
2. **Gain calibration difficulties**: Required 50-200x multiplier, highly flight-condition dependent
3. **223 Hz oscillation**: Gyro noise directly coupled into FF output
4. **Complex parameter tuning**: Too many interdependent parameters (ts, tp, ka, kr)

### v1 vs v2 Comparison
| Aspect | v1 (Trajectory) | v2 (Physics) |
|--------|-----------------|---------------|
| Model | Reference trajectory + preview | Plant inversion |
| Parameters | 6+ tuning params | 2 learned (E, b) |
| Noise handling | Gating (workaround) | Inherently low-noise |
| Calibration | Manual, difficult | Automatic via learning |

---

## v1 Implementation Phases (Historical)

### Phase 1: Core Implementation ✅

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
| Add CLI settings | ✅ | settings.c - 16 parameters (10 basic + 6 learning) |

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
| mbff_learn_enable | bool | OFF/ON | OFF | Enable online learning of b0/b1 |
| mbff_learn_rate_hz | uint8 | 50-250 | 100 | Learner update rate (Hz) |
| mbff_learn_gyro_min | uint8 | 10-200 | 30 | Minimum |gyro_rate| for learning (deg/s) |
| mbff_learn_beta | uint8 | 10-100 | 20 | Smoothing factor (x1000, so 20=0.02) |
| mbff_learn_excitation | uint8 | 0-200 | 50 | Excitation threshold (deg/s²), 0=disabled |

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

### Debug Channels (DEBUG_MBFF_LEARN)
| Index | Name | Description |
|-------|------|-------------|
| 0 | B0 | Learned b0 [x100] |
| 1 | B1 | Learned b1 [x100] |
| 2 | X | Current x = RPM²/1e6 [x10] |
| 3 | G_EST | Current g estimate [x100] |
| 4 | ALPHA_MEAS | Measured angular acceleration [deg/s²] |
| 5 | GATE_REASON | Gate reason flags (bitmask) |
| 6 | SAMPLES | Total samples in bins |
| 7 | ACTIVE_BINS | Number of bins with enough samples |

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

### 2026-01-31 - Online Parameter Learning
- Implemented bin-averaging method for learning b0/b1 effectiveness parameters
- **Features:**
  - RPM²-bin averaging with weighted least squares fitting
  - Comprehensive gating logic (saturation, command threshold, excitation, RPM validity)
  - Slow smoothing update (beta=0.02 default)
  - DEBUG_MBFF_LEARN mode for monitoring learner behavior
- **New CLI parameters:**
  - `mbff_learn_enable` - Enable/disable learning (OFF by default)
  - `mbff_learn_rate_hz` - Learner update rate (100 Hz default)
  - `mbff_learn_gyro_min` - Minimum gyro rate for learning (30 deg/s)
  - `mbff_learn_beta` - Smoothing factor (20 = 0.02)
  - `mbff_learn_excitation` - Excitation threshold (50 deg/s²)
- **Safety features:**
  - Learning gated when saturated (never underestimate effectiveness)
  - Learning gated when command too low (noisy estimates)
  - Learning gated when no RPM telemetry
  - Learned values clamped to safe CLI ranges
  - Slow update prevents rapid parameter jumps
- **Build status:** ✅ Pending verification

---

## Flight 2 Analysis Summary (v1 Testing)

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

## Flight 1 Analysis Summary (v1 Testing)

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

## Known Issues (v1)

| Issue | Status | Resolution |
|-------|--------|------------|
| Debug channel overflow | ✅ Fixed | Removed multipliers, log raw values |
| RPM² overflow | ✅ Fixed | Divide by 1M instead of 1K |
| FF magnitude mismatch | ✅ Fixed | Increased mbff_gain default to 500 (50.0x) |
| Preview term oscillation | ✅ Fixed | Added derivative-based preview gating |

---

## Next Steps (v2 Roadmap)

### Immediate
1. **Implement v2 physics-based FF in firmware**
   - Simplify mbff.c to use `u_ff = (α_cmd + b·ω) / E` model
   - Remove deprecated trajectory/preview parameters
   - Keep only: `mbff_enable`, `mbff_e` (effectiveness), `mbff_b` (drag)

2. **Port learning algorithm from simulation**
   - Implement least-squares bin-averaging from sim_mfbb.py
   - Real-time `E` and `b` estimation
   - Validated by simulation to achieve <0.2% error

### Testing
3. **Flight test with learned parameters**
   - Use DEBUG_MBFF_LEARN to observe convergence
   - Verify tracking error reduction matches simulation (45-64%)

4. **Compare v2 vs classic FF in real flights**
   - Back-to-back comparison on same tune
   - Measure RMS tracking error from blackbox
   - Evaluate subjective feel

### Future
5. **Per-axis effectiveness learning**
   - Different E values for roll/pitch/yaw
   - Account for asymmetric quads

6. **Adaptive FF based on battery voltage**
   - Scale E with motor constant changes

---

## v1 Next Steps (Deprecated)

<details>
<summary>Historical v1 tasks (click to expand)</summary>

1. ~~Run `make CONFIG=BETAFPVG473` to verify build~~ ✅
2. ~~Flash and verify CLI parameters work~~ ✅
3. ~~Test DEBUG_MBFF mode~~ ✅
4. ~~First hover test with conservative settings~~ ✅
5. ~~Blackbox analysis (flight 1)~~ ✅
6. ~~Implement preview gating~~ ✅
7. ~~Test with new defaults (gain=500, preview_threshold=50)~~ ✅
8. ~~Verify oscillation is eliminated during hover/sustained rolls~~ ✅ (sign changes: 25.3% → 11.8%)
9. ~~Analyze flight 2 for gain tuning~~ ✅
10. ~~Test with reduced gain (100 instead of 500) and slower trajectory (ts=30)~~ ❌ Deprecated
11. ~~High-rate tracking tests~~ ❌ Deprecated
12. ~~Compare feel vs classic FF~~ → Moved to v2 roadmap

</details>
