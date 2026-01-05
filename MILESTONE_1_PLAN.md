# Milestone 1: Infrastructure & State Machine

## Overview

**Goal**: Create the basic autotune framework that compiles and can be activated, but doesn't tune yet.

**Duration**: Week 1-2

**Success Criteria**:
- Autotune mode appears in Betaflight Configurator
- Switch activates/deactivates autotune mode
- State machine transitions are visible in debug output
- Safety abort works correctly
- No tuning happens yet - just infrastructure

---

## Prerequisites

✅ Milestone 0 completed:
- Development environment set up
- Firmware builds successfully
- Baseline flight tested

---

## File Structure to Create

```
src/main/flight/
├── autotune.c          # Main autotune logic (NEW)
└── autotune.h          # Public interface (NEW)

src/main/pg/
├── autotune.c          # Parameter group definition (NEW)
└── autotune.h          # Configuration structure (NEW)
```

---

## Step-by-Step Implementation

### Step 1.1: Create Parameter Group Files

**File: `src/main/pg/autotune.h`**

Define the configuration structure and settings that users can adjust:

```c
/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#pragma once

#include <stdint.h>
#include "pg/pg.h"

typedef enum {
    AUTOTUNE_MODE_STEP = 0,
    AUTOTUNE_MODE_DOUBLET,
    AUTOTUNE_MODE_RELAY,
} autotuneMode_e;

typedef struct autotuneConfig_s {
    uint8_t  autotune_enabled;        // Master enable flag
    uint8_t  autotune_mode;           // autotuneMode_e
    uint8_t  autotune_axes;           // Bitmask: bit0=roll, bit1=pitch, bit2=yaw
    uint8_t  step_amplitude;          // Step size in degrees/sec (10-200)
    uint16_t step_duration_ms;        // Duration of step (50-500ms)
    uint8_t  target_overshoot;        // Desired overshoot % (0-30)
    uint8_t  max_iterations;          // Max tune iterations per axis (1-20)
    uint8_t  safety_margin;           // Attitude limit during tune (degrees)
    uint8_t  save_on_complete;        // 1=auto-save, 0=require manual save
} autotuneConfig_t;

PG_DECLARE(autotuneConfig_t, autotuneConfig);
```

**File: `src/main/pg/autotune.c`**

Provide default values:

```c
/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "platform.h"

#ifdef USE_AUTOTUNE

#include "pg/autotune.h"

PG_REGISTER_WITH_RESET_TEMPLATE(autotuneConfig_t, autotuneConfig, PG_AUTOTUNE_CONFIG, 0);

PG_RESET_TEMPLATE(autotuneConfig_t, autotuneConfig,
    .autotune_enabled = 1,
    .autotune_mode = AUTOTUNE_MODE_STEP,
    .autotune_axes = 0x03,              // Roll + Pitch (bits 0 and 1)
    .step_amplitude = 100,              // 100 deg/sec
    .step_duration_ms = 100,            // 100ms
    .target_overshoot = 10,             // 10%
    .max_iterations = 10,
    .safety_margin = 30,                // 30 degrees
    .save_on_complete = 0,              // Don't auto-save
);

#endif // USE_AUTOTUNE
```

---

### Step 1.2: Create Main Autotune Files

**File: `src/main/flight/autotune.h`**

Public interface:

```c
/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "common/time.h"

// State machine states
typedef enum {
    AUTOTUNE_STATE_IDLE = 0,
    AUTOTUNE_STATE_SETUP,
    AUTOTUNE_STATE_WAIT_STABLE,
    AUTOTUNE_STATE_EXCITE,
    AUTOTUNE_STATE_MEASURE,
    AUTOTUNE_STATE_ANALYZE,
    AUTOTUNE_STATE_ADJUST,
    AUTOTUNE_STATE_COMPLETE,
    AUTOTUNE_STATE_ABORTED,
} autotuneState_e;

// Public functions
void autotuneInit(void);
void autotuneUpdate(timeUs_t currentTimeUs);
bool autotuneIsActive(void);
autotuneState_e autotuneGetState(void);
const char* autotuneGetStateName(void);
```

**File: `src/main/flight/autotune.c`**

State machine implementation:

```c
/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_AUTOTUNE

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/imu.h"
#include "flight/pid.h"

#include "io/beeper.h"

#include "pg/autotune.h"

#include "sensors/gyro.h"

#include "autotune.h"

// Runtime state
typedef struct autotuneRuntime_s {
    autotuneState_e state;
    autotuneState_e prevState;
    timeUs_t stateEnteredAt;
    uint8_t currentAxis;
    bool active;
} autotuneRuntime_t;

static autotuneRuntime_t runtime;

// State names for debugging
static const char* const stateNames[] = {
    "IDLE",
    "SETUP",
    "WAIT_STABLE",
    "EXCITE",
    "MEASURE",
    "ANALYZE",
    "ADJUST",
    "COMPLETE",
    "ABORTED"
};

// Initialize autotune system
void autotuneInit(void)
{
    memset(&runtime, 0, sizeof(runtime));
    runtime.state = AUTOTUNE_STATE_IDLE;
    runtime.prevState = AUTOTUNE_STATE_IDLE;
    runtime.active = false;
}

// Check if it's safe to run autotune
static bool autotuneSafetyCheck(void)
{
    // Check attitude limits (in decidegrees, so 300 = 30 degrees)
    const int safetyMarginDeci = autotuneConfig()->safety_margin * 10;
    
    if (ABS(attitude.raw[FD_ROLL]) > safetyMarginDeci ||
        ABS(attitude.raw[FD_PITCH]) > safetyMarginDeci) {
        return false;
    }
    
    // Check gyro rates not excessive (deg/sec)
    for (int axis = 0; axis < 3; axis++) {
        if (fabsf(gyro.gyroADCf[axis]) > 500.0f) {
            return false;
        }
    }
    
    // Check throttle is in reasonable range (hover)
    const float throttle = rcCommand[THROTTLE] / 1000.0f;
    if (throttle < 0.2f || throttle > 0.8f) {
        return false;
    }
    
    // Check no stick input (pilot not commanding)
    if (fabsf(getRcDeflection(FD_ROLL)) > 0.1f ||
        fabsf(getRcDeflection(FD_PITCH)) > 0.1f ||
        fabsf(getRcDeflection(FD_YAW)) > 0.1f) {
        return false;
    }
    
    return true;
}

// Change state with logging
static void changeState(autotuneState_e newState, timeUs_t currentTimeUs)
{
    if (runtime.state != newState) {
        runtime.prevState = runtime.state;
        runtime.state = newState;
        runtime.stateEnteredAt = currentTimeUs;
        
        DEBUG_SET(DEBUG_AUTOTUNE, 0, newState);
    }
}

// Update autotune state machine
void autotuneUpdate(timeUs_t currentTimeUs)
{
    // Check if autotune mode is active
    const bool shouldBeActive = IS_RC_MODE_ACTIVE(BOXAUTOTUNE) && 
                                autotuneConfig()->autotune_enabled;
    
    // Activation/deactivation logic
    if (shouldBeActive && !runtime.active) {
        // Just activated
        runtime.active = true;
        changeState(AUTOTUNE_STATE_SETUP, currentTimeUs);
        beeper(BEEPER_AUTOTUNE_START);
    } else if (!shouldBeActive && runtime.active) {
        // Just deactivated
        runtime.active = false;
        changeState(AUTOTUNE_STATE_IDLE, currentTimeUs);
        beeper(BEEPER_AUTOTUNE_DONE);
    }
    
    if (!runtime.active) {
        return;
    }
    
    // State machine
    switch (runtime.state) {
        case AUTOTUNE_STATE_IDLE:
            // Should not be here if active
            changeState(AUTOTUNE_STATE_SETUP, currentTimeUs);
            break;
            
        case AUTOTUNE_STATE_SETUP:
            // Wait 2 seconds before starting
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 2000000) {
                changeState(AUTOTUNE_STATE_WAIT_STABLE, currentTimeUs);
            }
            break;
            
        case AUTOTUNE_STATE_WAIT_STABLE:
            // Check if conditions are safe
            if (autotuneSafetyCheck()) {
                // Stable for 1 second
                if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 1000000) {
                    changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
                    beeper(BEEPER_AUTOTUNE_DONE);
                }
            } else {
                // Reset timer if not stable
                runtime.stateEnteredAt = currentTimeUs;
            }
            break;
            
        case AUTOTUNE_STATE_EXCITE:
            // TODO: Implement in Milestone 2
            changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
            break;
            
        case AUTOTUNE_STATE_MEASURE:
            // TODO: Implement in Milestone 2
            changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
            break;
            
        case AUTOTUNE_STATE_ANALYZE:
            // TODO: Implement in Milestone 3
            changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
            break;
            
        case AUTOTUNE_STATE_ADJUST:
            // TODO: Implement in Milestone 3
            changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
            break;
            
        case AUTOTUNE_STATE_COMPLETE:
            // Stay in complete state until switch turned off
            break;
            
        case AUTOTUNE_STATE_ABORTED:
            // Stay in aborted state until switch turned off
            break;
    }
    
    // Safety check - abort if unsafe
    if (runtime.state != AUTOTUNE_STATE_IDLE && 
        runtime.state != AUTOTUNE_STATE_COMPLETE &&
        runtime.state != AUTOTUNE_STATE_ABORTED) {
        
        if (!autotuneSafetyCheck()) {
            changeState(AUTOTUNE_STATE_ABORTED, currentTimeUs);
            beeper(BEEPER_AUTOTUNE_FAIL);
        }
    }
}

// Query functions
bool autotuneIsActive(void)
{
    return runtime.active;
}

autotuneState_e autotuneGetState(void)
{
    return runtime.state;
}

const char* autotuneGetStateName(void)
{
    return stateNames[runtime.state];
}

#endif // USE_AUTOTUNE
```

---

### Step 1.3: Add Build System Integration

**Modify: `src/main/pg/pg_ids.h`**

Add autotune parameter group ID (find the list and add):

```c
    PG_AUTOTUNE_CONFIG,
```

**Modify: `src/main/target/common_pre.h`**

Add feature flag (in the features section):

```c
#define USE_AUTOTUNE
```

---

### Step 1.4: Add Box Mode Integration

**Modify: `src/main/fc/rc_modes.h`**

Add autotune box (find the boxId_e enum and add before the last entry):

```c
    BOXAUTOTUNE,
```

**Modify: `src/main/msp/msp_box.c`**

Add box definition (find the box definitions array):

```c
    { BOXID_AUTOTUNE, "AUTOTUNE", PERMANENT_ID_AUTOTUNE },
```

**Modify: `src/main/fc/rc_modes.c`**

Add to mode activation conditions array.

---

### Step 1.5: Integrate with PID Loop

**Modify: `src/main/flight/pid.c`**

Add autotune header:

```c
#ifdef USE_AUTOTUNE
#include "flight/autotune.h"
#endif
```

In `pidInit()` function, add:

```c
#ifdef USE_AUTOTUNE
    autotuneInit();
#endif
```

In main PID loop function (find `pidController()`), add before or after PID calculations:

```c
#ifdef USE_AUTOTUNE
    if (FLIGHT_MODE(AUTOTUNE_MODE)) {
        autotuneUpdate(currentTimeUs);
    }
#endif
```

---

## Testing Plan

### Test 1: Compilation

```bash
cd C:\Users\pmant\source\repos\betaflight\betaflight
& "C:\Program Files\Git\bin\bash.exe" -c "make CONFIG=BETAFPVG473"
```

**Expected**: Clean build with no errors

### Test 2: Flash and Connect

1. Flash firmware to drone
2. Connect in Configurator
3. Go to Modes tab

**Expected**: "AUTOTUNE" mode appears in the list

### Test 3: Activate Mode

1. Assign AUTOTUNE to a switch
2. Save and reboot
3. Arm drone (props off!)
4. Flip autotune switch ON

**Expected**: 
- Hear startup beep
- Mode activates

### Test 4: Debug Output

1. Set debug mode to AUTOTUNE
2. Activate autotune
3. Watch debug values in Configurator

**Expected**:
- Debug[0] shows state transitions (0→1→2→7)
- State progresses: IDLE → SETUP → WAIT_STABLE → COMPLETE

### Test 5: Safety Abort

1. Activate autotune
2. Move sticks significantly

**Expected**:
- Autotune aborts (state 8)
- Hear failure beep

---

## Troubleshooting

### Build Errors

**"PG_AUTOTUNE_CONFIG undeclared"**
- Make sure you added the ID to `pg_ids.h`

**"BOXAUTOTUNE undeclared"**
- Check `rc_modes.h` has the box enum

**"undefined reference to autotuneInit"**
- Make sure autotune.c is being compiled
- Check `USE_AUTOTUNE` is defined

### Runtime Issues

**"AUTOTUNE not in modes list"**
- Check box definition in `msp_box.c`
- Reflash firmware

**"Mode activates but nothing happens"**
- Check debug output
- Verify `autotuneUpdate()` is being called from PID loop

---

## Success Criteria Checklist

- [ ] All files created without errors
- [ ] Firmware compiles successfully
- [ ] AUTOTUNE mode appears in Configurator
- [ ] Mode can be assigned to switch
- [ ] Switch activation works
- [ ] State machine transitions (IDLE → SETUP → WAIT_STABLE → COMPLETE)
- [ ] Debug output shows state changes
- [ ] Safety abort works (stick input triggers abort)
- [ ] Beeper feedback works (start/complete/fail beeps)

---

## Current Progress

### ✅ Completed
- None yet - ready to start!

### 🎯 Next Task
- Create `src/main/pg/autotune.h`

---

## Ready?

Once this milestone is complete, you'll have:
- A working autotune mode that can be switched on/off
- State machine that transitions properly
- Safety checks that abort if conditions aren't met
- Framework ready for actual tuning implementation in Milestone 2

**Let's start with Step 1.1: Create the parameter group files!**
