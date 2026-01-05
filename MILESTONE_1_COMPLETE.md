# Milestone 1: Infrastructure & State Machine - COMPLETE ✅

**Date**: January 4, 2026  
**Target Hardware**: BETAFPVG473 (STM32G47X) - BETAFPV Air G4 5in1  
**Firmware**: betaflight 2026.6.0-alpha  

## Summary

Successfully implemented and tested the autotune infrastructure with a 9-state finite state machine, parameter configuration system, box mode activation, CLI commands, and blackbox logging support.

## Implementation Details

### Files Created

1. **src/main/pg/autotune.h** - Parameter group header
   - Configuration structure with 9 parameters
   - Mode enum (STEP, DOUBLET, RELAY)
   
2. **src/main/pg/autotune.c** - Parameter group implementation
   - Default values
   - PG registration

3. **src/main/flight/autotune.h** - Public API
   - State machine enum (9 states)
   - Function prototypes

4. **src/main/flight/autotune.c** - State machine implementation
   - Runtime state tracking
   - Safety checks (attitude ±30°, gyro <500°/s, throttle 0.2-0.8, stick inputs <0.1)
   - State machine with activation/deactivation logic
   - Beeper feedback

### Files Modified

1. **src/main/pg/pg_ids.h** - Added PG_AUTOTUNE_CONFIG = 561
2. **src/main/target/common_pre.h** - Added USE_AUTOTUNE flag
3. **src/config/configs/BETAFPVG473/config.h** - Added USE_AUTOTUNE + optimizations
4. **src/main/fc/rc_modes.h** - Added BOXAUTOTUNE enum
5. **src/main/msp/msp_box.c** - Added AUTOTUNE box definition + activeBoxIds enablement
6. **src/main/io/beeper.h** - Added BEEPER_AUTOTUNE_START, _DONE, _FAIL
7. **src/main/build/debug.h** - Added DEBUG_AUTOTUNE
8. **src/main/build/debug.c** - Added "AUTOTUNE" debug name
9. **src/main/flight/pid.c** - Added autotune.h include, rc_modes.h include, autotuneUpdate() call
10. **src/main/flight/pid_init.c** - Added autotuneInit() call
11. **src/main/flight/autotune.c** - Added rc.h, rc_modes.h includes
12. **src/main/pg/autotune.c** - Added pg.h, pg_ids.h includes
13. **src/main/cli/settings.c** - Added autotune.h include + 9 CLI settings entries
14. **mk/source.mk** - Added autotune.c and pg/autotune.c to build

### Target Optimizations

Disabled unnecessary features in BETAFPVG473 config for improved performance:
- ❌ Barometer (not needed for rate-based tuning)
- ❌ Magnetometer
- ❌ GPS
- ❌ LED strip  
- ❌ Servos
- ❌ PINIO features
- ❌ HD OSD (kept analog MAX7456 OSD)

## State Machine

```
IDLE → SETUP → WAIT_STABLE → EXCITE → MEASURE → ANALYZE → ADJUST → COMPLETE
                                ↓                                      ↓
                              ABORTED ←──────────────────────────────┘
```

### States
- **IDLE**: Waiting for activation
- **SETUP**: Initialize measurement for current axis
- **WAIT_STABLE**: Wait for aircraft to stabilize
- **EXCITE**: Apply step input
- **MEASURE**: Collect response data
- **ANALYZE**: Calculate gains (placeholder)
- **ADJUST**: Update PID values (placeholder)
- **COMPLETE**: All axes tuned
- **ABORTED**: Safety limits exceeded

### Safety Checks
- Attitude limits: ±30° roll/pitch
- Gyro rate limits: <500°/s
- Throttle range: 0.2-0.8 (20-80%)
- Stick deflection: <0.2 (20%)

## Configuration Parameters

All accessible via CLI with `get/set autotune_*`:

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `autotune_enabled` | bool | OFF/ON | ON | Master enable |
| `autotune_mode` | uint8 | 0-2 | 0 (STEP) | Excitation type |
| `autotune_axes` | uint8 | 0-7 | 3 (Roll+Pitch) | Bitmask: bit0=roll, bit1=pitch, bit2=yaw |
| `autotune_step_amplitude` | uint8 | 10-200 | 100 | Step size in deg/sec |
| `autotune_step_duration_ms` | uint16 | 50-500 | 100 | Step duration |
| `autotune_target_overshoot` | uint8 | 0-50 | 10 | Target overshoot % |
| `autotune_max_iterations` | uint8 | 1-20 | 10 | Max tuning iterations |
| `autotune_safety_margin` | uint8 | 10-50 | 30 | Safety margin % |
| `autotune_save_on_complete` | bool | OFF/ON | OFF | Auto-save PIDs |

## Activation

### Box Mode
- Navigate to **Modes** tab in Betaflight Configurator
- Find **AUTOTUNE** mode
- Assign to an AUX switch
- Mode only activates if accelerometer is present

### CLI Verification
```
# get autotune
autotune_enabled = ON
autotune_mode = 0
autotune_axes = 3
autotune_step_amplitude = 100
...
```

## Blackbox Logging

Autotune state is logged via DEBUG mode:
```
set debug_mode = AUTOTUNE
```

Debug fields:
- `debug[0]`: Current autotune state (0-8)
- `debug[1]`: Activation events (1000 on activate, 7777 on complete, 8888 on abort)
- `debug[2]`: Status markers (same as debug[1])
- `debug[3]`: Time in current state (milliseconds)
- `debug[4]`: Safety check failure reason (0=pass, 1=attitude, 2=gyro, 3=throttle, 4=sticks)
- `debug[5]`: Throttle percentage (0-100)
- `debug[6]`: Maximum stick deflection percentage (0-100)
- `debug[7]`: Reserved for future use

## Testing Status

✅ **Firmware built successfully**  
✅ **Firmware flashed successfully**  
✅ **AUTOTUNE mode visible in Modes tab**  
✅ **Mode assigned to switch successfully**  
✅ **CLI commands working** (`get autotune`)  
✅ **DEBUG_AUTOTUNE available for blackbox**  
✅ **Box mode activation confirmed** (state machine activates on switch flip)  
✅ **Flight tested** (state progression 1→7 verified, safety checks working)  
✅ **Throttle calculation fixed** (calculateThrottlePercent() properly returns 0-100%)  
✅ **All safety checks validated** (attitude, gyro, throttle, stick deflection)  
✅ **State transitions confirmed** (IDLE→SETUP→WAIT_STABLE→EXCITE→...→COMPLETE)  
✅ **Beeper feedback working**

## Build Commands

```bash
# Clean build
make clean

# Build for BETAFPVG473
make CONFIG=BETAFPVG473

# Output: obj/betaflight_STM32G47X_BETAFPVG473.hex
```

## Memory Usage

```
Memory region         Used Size  Region Size  %age Used
           FLASH:         480 B        10 KB      4.69%
    FLASH_CONFIG:           0 B         8 KB      0.00%
          FLASH1:      368348 B       496 KB     72.52%
   SYSTEM_MEMORY:           0 B        64 KB      0.00%
             RAM:       88512 B       124 KB     69.71%
             CCM:        2520 B         4 KB     61.52%
```

## Next Steps - Milestone 2: Step Response Measurement

1. Implement excitation signal generation (step input)
2. Integrate with existing SDFT for frequency domain analysis
3. Capture gyro response data
4. Implement peak detection and settling time measurement
5. Calculate system transfer function parameters
6. Test on bench with motor disabled

## Known Limitations

- State machine transitions are placeholders (immediate advancement after delays)
- No actual excitation signal generation yet (EXCITE state is placeholder)
- No gyro response measurement yet (MEASURE state is placeholder)
- No gain calculation algorithms yet (ANALYZE state is placeholder)
- No PID adjustment yet (ADJUST state is placeholder)
- Requires accelerometer to enable BOXAUTOTUNE mode

## Lessons Learned

### Throttle Calculation Bug
**Problem**: Initial implementation used `rcCommand[THROTTLE] / 1000.0f` which produced incorrect values (136% when actual throttle was ~35%).

**Root Cause**: `rcCommand[THROTTLE]` is in PWM range (typically 1000-2000), not a direct 0-1000 scale. Betaflight uses `mincheck` offset and other transformations.

**Solution**: Use Betaflight's built-in `calculateThrottlePercent()` function from `fc/core.h`, which properly handles:
- PWM range normalization
- 3D mode
- Returns correct 0-100% integer values

**Learning**: Always search for existing Betaflight API functions before implementing calculations from scratch. The codebase has well-tested utilities for common operations.

### Debug Output Strategy
**Effective Approach**: Added diagnostic debug channels (4-6) during development:
- `debug[4]`: Which safety check failed (1=attitude, 2=gyro, 3=throttle, 4=sticks)
- `debug[5]`: Throttle percentage value
- `debug[6]`: Stick deflection value

This allowed rapid identification of the throttle calculation bug through blackbox analysis without needing print debugging or repeated flashing.

**Learning**: Implement comprehensive debug output early, including both status codes AND actual values being checked. This pays off during flight testing when iteration cycles are expensive.

## Files Changed Summary

**Created**: 4 files  
**Modified**: 14 files  
**Build System**: 1 file  
**Total**: 19 files

## Git Status

Ready to commit. Files staged for commit: 19 files modified/created.

### Commit Command

```bash
cd /c/Users/pmant/source/repos/betaflight/betaflight
git add -A
git commit -m "feat: Add autotune infrastructure and state machine (Milestone 1)

- Add parameter group for autotune configuration with 9 parameters
- Implement 9-state FSM with safety checks (attitude, gyro, throttle, sticks)
- Add BOXAUTOTUNE mode for switch activation  
- Add CLI commands for autotune configuration
- Add DEBUG_AUTOTUNE mode (8 channels) for blackbox logging
- Integrate with PID loop and initialization
- Optimize BETAFPVG473 target (disable unused features)
- Add beeper events for user feedback
- Fix throttle calculation using calculateThrottlePercent()

Safety checks: attitude ±30°, gyro <500°/s, throttle 20-80%, sticks <20%
State machine: IDLE → SETUP → WAIT_STABLE → EXCITE → MEASURE → ANALYZE 
               → ADJUST → COMPLETE/ABORTED

Tested: CLI, box mode, firmware build/flash, flight validation
Flight validated: State transitions 1→7, safety checks functional
Next: Implement step response measurement (Milestone 2)
"
```

### Push Command

```bash
git push origin HEAD
```
