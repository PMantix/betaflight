# Autotune V2 Session Handoff - January 25, 2026 (Updated)

**Purpose:** Prime the next agent session to continue flight testing and validation

---

## 1. Critical Context

### 1.1 Current State
- **Branch:** `autotune-v2-clean` (pushed to GitHub)
- **Target:** BETAFPVG473
- **Build Status:** ✅ BUILT - Ready to flash

### 1.2 Bugs Fixed This Session

| ID | Issue | Root Cause | Fix Applied |
|----|-------|-----------|-------------|
| FLT-001 | Overshoot 122-194% (nonsense) | Used final setpoint (near 0) instead of peak | Find peak setpoint in buffer, use for calculation |
| FLT-002 | Lag calculation wrong | Same bug as overshoot | Use peak setpoint for 50% threshold |
| FLT-003 | Drone drifts during wiggles | Half-sine (positive only) waveform | Full symmetric sine: 0→+peak→0→-peak→0 |
| FLT-004 | debug[7] F gain always 0 | Only logged in F_TUNE state | Log at PD_RATIO_SEEK and PD_SCALE_UP entry |
| FLT-005 | PD_RATIO_SEEK too slow | Excessive local optimization | Advance after 3 consecutive good events within ±5% |
| OBS-007 | Reason codes persist | No auto-clear mechanism | 200ms pulse timeout, auto-clears to 0 |

### 1.3 Files Modified This Session
1. `src/main/flight/autotune_v2/autotune_metrics.c` - Overshoot and lag calculation fixes
2. `src/main/flight/autotune_v2/autotune_feedback.c` - Symmetric wiggle waveforms
3. `src/main/flight/autotune_v2/autotune_core.c` - F gain logging, faster exit logic, reason code pulsing
4. `src/main/flight/autotune_v2/autotune_types.h` - Added reasonCodeSetTimeUs field

---

## 2. Flight Test Results (Previous Flights)

### 2.1 Phase 2 Repeat #2 (After Overshoot Fix)
- **File:** `bb logs/phase_2_repeat_2.csv`
- **Result:** ✅ State machine progresses correctly
- **States Observed:** 0→1→2→3→4→5 (reached PD_SCALE_UP)
- **Overshoot Values:** 2.9-9.6% (mean 6.8%) - CORRECT
- **Issues Found:** PD_SCALE_UP spent 21s with no gain changes (lag calculation bug - now fixed)

---

## 3. Technical Reference

### 3.1 Debug Channel Mapping (AUTOTUNE_V2 mode)
```
debug[0] = State (0-8)
debug[1] = Axis (0=Roll, 1=Pitch, 2=Yaw)
debug[2] = Reason code (pulses for 200ms then clears to 0)
debug[3] = Decision/Progress
debug[4] = Overshoot × 10
debug[5] = P gain
debug[6] = D gain
debug[7] = F gain (now logged in PD_RATIO_SEEK and PD_SCALE_UP too)
```

### 3.2 State Machine Values
```
IDLE              = 0
HOVER_LOCK        = 1
THROTTLE_SWEEP    = 2
NOISE_CONFIRM     = 3
PD_RATIO_SEEK     = 4
PD_SCALE_UP       = 5
F_TUNE            = 6
PD_RETUNE         = 7
COMPLETE          = 8
IDLE_WAIT_ARM     = 9
IDLE_WAIT_DISARM  = 10
```

### 3.3 Sweep Detection Requirements
After the fix, THROTTLE_SWEEP exits when:
1. `sweepCount >= 2` (two high→low→hover cycles)
2. Stable hover for 2 seconds after sweeps

**Throttle Thresholds (in autotune_filter.c):**
```c
#define SWEEP_HIGH_THROTTLE     0.50f   // 50% = "high"
#define SWEEP_LOW_THROTTLE      0.30f   // 30% = "low"
#define SWEEP_HOVER_LOW         0.30f   // Hover band lower
#define SWEEP_HOVER_HIGH        0.60f   // Hover band upper
#define MIN_SWEEPS_REQUIRED     2
#define SWEEP_STABLE_HOVER_US   2000000 // 2 seconds
```

---

## 4. Blackbox CSV Analysis

### 4.1 Important: Header Row Skip
```python
import pandas as pd
df = pd.read_csv('file.csv', skiprows=147)  # Header is 147 rows!
```

### 4.2 Key Column Names
- `loopIteration` - Frame counter
- `time (us)` - Timestamp
- `rcCommand[3]` - Throttle (1000-2000)
- `gyroADC[0]`, `gyroADC[1]`, `gyroADC[2]` - Roll/Pitch/Yaw rates
- `debug[0]` through `debug[7]` - Autotune state/metrics

---

## 5. Immediate Next Steps

### 5.1 User Action Required
1. **Flash** new firmware to quad (build complete)
2. **Fly** to verify:
   - PD_SCALE_UP now adjusts gains (with corrected lag calculation)
   - Faster state progression (consecutive good events logic)
   - F gain now appears in debug[7]
   - Wiggles no longer cause drift

### 5.2 Verification Goals
1. Export blackbox to CSV
2. Check PD_SCALE_UP actually changes P/D gains
3. Verify reason codes pulse briefly (200ms) then clear
4. Monitor for full tuning cycle through Roll, then Pitch, then (optionally) Yaw

---

## 6. Known Issues / Observations

| Issue | Status | Notes |
|-------|--------|-------|
| Overshoot calculation wrong | ✅ FIXED | Uses peak setpoint now |
| Lag calculation wrong | ✅ FIXED | Uses peak setpoint now |
| Wiggle asymmetric | ✅ FIXED | Full symmetric sine |
| debug[7] F gain missing | ✅ FIXED | Logged at state entry |
| PD_RATIO_SEEK too slow | ✅ FIXED | 3 consecutive good events advances |
| Reason codes persist | ✅ FIXED | 200ms pulse timeout |

---

## 7. Files to Review

| File | Purpose |
|------|---------|
| `docs/autotune_v2/ISSUE_TRACKER.md` | All bug fixes documented |
| `docs/autotune_v2/PROGRESS.md` | Overall progress tracking |
| `src/main/flight/autotune_v2/autotune_core.c` | Main state machine |
| `src/main/flight/autotune_v2/autotune_metrics.c` | Overshoot/lag fixes |
| `src/main/flight/autotune_v2/autotune_feedback.c` | Wiggle fixes |

---

## 8. Agent Instructions

When continuing this session:

1. **First:** Confirm user has flashed the new firmware
2. **Second:** Analyze new flight logs to verify PD_SCALE_UP adjusts gains
3. **Third:** Monitor for completion of full tuning cycle (Roll→Pitch→Yaw)
4. **Fourth:** Document findings in ISSUE_TRACKER.md validation checklist
```bash
cd betaflight
make CONFIG=BETAFPVG473
```

**Output Location:**
```
obj/betaflight_2026.6.0-alpha_STM32G47X_BETAFPVG473.hex
```
