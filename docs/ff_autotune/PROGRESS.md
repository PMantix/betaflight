# FF Autotune v3 Implementation Progress

> **Last Updated:** 2026-02-05  
> **Branch:** `ff-autotune-v1`  
> **Commits:** `dc39e2eea` (PRD), `78ae93515` (Implementation)

---

## Summary

| Category | Planned | Implemented | Status |
|----------|---------|-------------|--------|
| Functional Requirements | 8 | 8 | ✅ Complete |
| CLI Parameters | 11 | 11 | ✅ Complete |
| Debug Channels | 8 | 8 | ✅ Complete |
| New Files | 4 | 4 | ✅ Complete |
| Modified Files | 7 | 9 | ✅ Exceeded |

**Overall Status: ✅ IMPLEMENTATION COMPLETE - READY FOR FLIGHT TEST**

---

## Functional Requirements

### FR1: Error-Based D-Term ✅ COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Switch D-term from gyro-based to error-based when active | ✅ | [pid.c#L1410-L1423](../../../src/main/flight/pid.c#L1410-L1423) |
| Store previous setpoint for error derivative | ✅ | `static float previousErrorRate[XYZ_AXIS_COUNT]` |
| Only affects roll/pitch when tuning active | ✅ | `if (ffAutotuneIsActive() && axis <= FD_PITCH)` |

**Implementation Details:**
```c
// Error-based D: derivative of (setpoint - gyro) for better tracking
const float currentError = currentPidSetpoint - gyroRateDterm[axis];
delta = (currentError - previousErrorRate[axis]) * pidRuntime.pidFrequency;
```

---

### FR2: Setpoint Tracking Monitor ✅ COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Detect tracking window (|setpoint| 100-600 deg/s) | ✅ | [ff_autotune.c#L459-L466](../../../src/main/flight/ff_autotune.c#L459-L466) |
| Trigger on |setpoint_accel| > 10000 deg/s² | ✅ | `min_accel * 100.0f` |
| Accumulate tracking error during window | ✅ | `state->errorAccumulator += trackingError` |
| State machine: IDLE→RISING→PEAK→FALLING→SETTLING→ADJUSTING | ✅ | `ffWindowState_e` enum |
| Minimum 50 samples for valid maneuver | ✅ | `FF_AUTOTUNE_MIN_SAMPLES = 50` |
| 50ms settle time after maneuver | ✅ | `FF_AUTOTUNE_SETTLE_TIME_US = 50000` |

---

### FR3: Gain Adjustment Logic ✅ COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Negative error (lag) → increase FF | ✅ | [ff_autotune.c#L358-L360](../../../src/main/flight/ff_autotune.c#L358-L360) |
| Positive error (lead) → decrease FF | ✅ | [ff_autotune.c#L361-L363](../../../src/main/flight/ff_autotune.c#L361-L363) |
| Error within deadband → no change | ✅ | Handled by bracket logic |
| Binary search when bracketed | ✅ | `newGain = (lowerGain + upperGain) / 2` |
| Stop adjustment when converged | ✅ | `FF_BRACKET_CONVERGED` state check |

**Three-Phase Convergence:**
1. **SEARCHING** - Linear step in error direction
2. **BRACKETED** - Binary search within bracket
3. **CONVERGED** - Hold at midpoint of final bracket

---

### FR4: Per-Axis Learning ✅ COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Separate state for Roll | ✅ | `runtime.axis[FD_ROLL]` |
| Separate state for Pitch | ✅ | `runtime.axis[FD_PITCH]` |
| Independent gain tracking | ✅ | `ffAxisState_t` struct per axis |
| Independent history buffers | ✅ | 8-entry buffer each |
| Independent bracket state | ✅ | Separate `lowerGain`, `upperGain` |

---

### FR5: Flight Mode Control ✅ COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| New box mode `BOXFFAUTOTUNE` | ✅ | [rc_modes.h](../../../src/main/fc/rc_modes.h) |
| Permanent ID 56 | ✅ | [msp_box.c](../../../src/main/msp/msp_box.c) |
| Only active when armed | ✅ | `ARMING_FLAG(ARMED)` check |
| Beeper on activation | ✅ | `beeper(BEEPER_RX_SET)` |
| Beeper on save | ✅ | `beeper(BEEPER_READY_BEEP)` |

---

### FR6: EEPROM Save Policy ✅ COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Save only on mode deactivation | ✅ | [ff_autotune.c#L568-L571](../../../src/main/flight/ff_autotune.c#L568-L571) |
| Track `gainsModified` flag | ✅ | `runtime.gainsModified` |
| Save both roll and pitch gains | ✅ | `ffAutotuneSaveGains()` |
| Clear modified flag after save | ✅ | `runtime.gainsModified = false` |

**Implementation:**
```c
} else if (!modeActive && runtime.wasActive) {
    // Just deactivated - save if modified
    runtime.active = false;
    if (runtime.gainsModified) {
        ffAutotuneSaveGains();
    }
}
```

---

### FR7: Debug Output ✅ COMPLETE

| PRD Channel | Content | Implemented | Location |
|-------------|---------|-------------|----------|
| 0 | Current Gain | ✅ | `state->gain` |
| 1 | Tracking Error | ✅ | `lrintf(trackingError)` |
| 2 | Window State | ✅ | `state->windowState` |
| 3 | Last Assessment | ✅ | `state->lastAssessment` |
| 4 | Last Avg Error | ✅ | `state->lastAvgError` |
| 5 | Sample Count | ✅ | `state->sampleCount` (capped 255) |
| 6 | History Count | ✅ | `state->historyCount` |
| 7 | Bracket State | ✅ | `state->bracketState` |

**Debug Mode:** `DEBUG_FF_AUTOTUNE` added to [debug.h](../../../src/main/build/debug.h) and [debug.c](../../../src/main/build/debug.c)

---

### FR8: Performance History & Bracketing ✅ COMPLETE

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| 8-entry ring buffer per axis | ✅ | `FF_AUTOTUNE_HISTORY_SIZE = 8` |
| Store (gain, avgError, sampleCount) tuples | ✅ | `ffHistoryEntry_t` struct |
| Intelligent eviction (preserve diversity) | ✅ | `findEvictionIndex()` |
| Never evict lower bracket bound | ✅ | `isLowerBound[]` check |
| Never evict upper bracket bound | ✅ | `isUpperBound[]` check |
| Evict same-type samples first | ✅ | `newType` matching logic |
| Update bracket bounds on new samples | ✅ | [ff_autotune.c#L327-L340](../../../src/main/flight/ff_autotune.c#L327-L340) |
| Check for bracket establishment | ✅ | [ff_autotune.c#L343-L352](../../../src/main/flight/ff_autotune.c#L343-L352) |

---

## CLI Parameters

| Parameter | PRD Default | Implemented Default | Status |
|-----------|-------------|---------------------|--------|
| `ff_autotune_enabled` | OFF | OFF | ✅ |
| `ff_autotune_setpoint_low` | 100 | 100 | ✅ |
| `ff_autotune_setpoint_high` | 600 | 600 | ✅ |
| `ff_autotune_min_accel` | 100 | 100 | ✅ |
| `ff_autotune_gain_step` | 5 | 5 | ✅ |
| `ff_autotune_gain_max` | 200 | 200 | ✅ |
| `ff_autotune_gain_min` | 0 | 0 | ✅ |
| `ff_autotune_error_deadband` | 10 | 10 | ✅ |
| `ff_autotune_converge_threshold` | 3 | 3 | ✅ |
| `ff_autotune_gain_roll` | 0 | 0 | ✅ |
| `ff_autotune_gain_pitch` | 0 | 0 | ✅ |

---

## Files Created

| File | Lines | Status |
|------|-------|--------|
| `src/main/flight/ff_autotune.h` | ~60 | ✅ Created |
| `src/main/flight/ff_autotune.c` | ~588 | ✅ Created |
| `src/main/pg/ff_autotune.h` | ~40 | ✅ Created |
| `src/main/pg/ff_autotune.c` | ~45 | ✅ Created |

---

## Files Modified

| File | Modification | Status |
|------|--------------|--------|
| `src/main/target/common_pre.h` | Added `USE_FF_AUTOTUNE` | ✅ |
| `src/main/fc/rc_modes.h` | Added `BOXFFAUTOTUNE` | ✅ |
| `src/main/msp/msp_box.c` | Registered box mode (ID 56) | ✅ |
| `src/main/pg/pg_ids.h` | Added `PG_FF_AUTOTUNE_CONFIG = 561` | ✅ |
| `src/main/build/debug.h` | Added `DEBUG_FF_AUTOTUNE` | ✅ |
| `src/main/build/debug.c` | Added debug mode name | ✅ |
| `src/main/cli/settings.c` | Added 11 CLI parameters | ✅ |
| `src/main/flight/pid.c` | Error-based D-term + update call | ✅ |
| `mk/source.mk` | Added ff_autotune.c to build | ✅ |

---

## Build Status

| Target | Status | Size |
|--------|--------|------|
| BETAFPVG473 | ✅ Builds successfully | 1,037,854 bytes |

---

## Known Deviations from PRD

### Minor Implementation Differences

1. **PRD suggested gain ranges** in table but implementation uses same values - ✅ Consistent

2. **Debug channel naming** - PRD used descriptive names, implementation uses indices 0-7 - ✅ Standard BF convention

3. **Window state machine** - Implementation has slightly different state names but same logic:
   - PRD: IDLE → RISING → PEAK → FALLING → SETTLING → ADJUSTING
   - Impl: Same sequence with minor flow differences

---

## Next Steps

### Ready for Flight Test

1. **Flash firmware** to BETAFPVG473
2. **Configure in CLI:**
   ```
   set ff_autotune_enabled = ON
   set feedforward_roll = 0
   set feedforward_pitch = 0
   save
   ```
3. **Assign AUX switch** to "FF AUTOTUNE" mode
4. **Test flight procedure:**
   - Hover, arm autotune
   - Perform aggressive maneuvers (30-60 sec)
   - Disarm autotune (gains save automatically)
   - Check learned values: `get ff_autotune_gain_roll`, `get ff_autotune_gain_pitch`

### Blackbox Analysis

Set `debug_mode = FF_AUTOTUNE` to see:
- Real-time gain changes
- Window detection
- Bracket progression
- Error measurements

---

## Test Checklist

- [ ] Mode activates/deactivates with AUX switch
- [ ] Beeper sounds on activation
- [ ] Beeper sounds on deactivation (when gains saved)
- [ ] Gains save to EEPROM only on deactivation
- [ ] Window detection works during maneuvers
- [ ] Bracketing progresses toward convergence
- [ ] Debug output visible in Blackbox
- [ ] Error-based D-term active during tuning
- [ ] Gains persist across power cycles

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2026-02-05 | 1.0 | Initial progress document - implementation complete |
