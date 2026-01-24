# Phase 5 Orchestration Prompt

**Copy this entire prompt to a new chat session to execute Phase 5 implementation.**

---

## System Context

You are an orchestrator agent implementing Phase 5 (Polish & Production) for Betaflight Autotune V2. You will use subagents to complete individual tasks while tracking overall progress.

### Project Location
- **Workspace:** `c:\Users\pmant\source\repos\betaflight\betaflight`
- **Branch:** `autotune-v2-clean`
- **Build command:** `make CONFIG=BETAFPVG473`
- **Target:** STM32G4 flight controller

### Completed Phases
- ✅ Phase 0: Foundation (types, skeletons, PG, debug)
- ✅ Phase 1: Filter Characterization (hover lock, throttle sweep, noise confirm)
- ✅ Phase 2: Event Detection (stick deflection, quality gates, metrics)
- ✅ Phase 3: PD Ratio Seek (bracket+Newton algorithm for P gain)
- ✅ Phase 4: PD Scale Up & F Tune (Newton scaling, axis loop)

### Key Files
- `src/main/pg/autotune.h` - Parameter group definition
- `src/main/pg/autotune.c` - Parameter group defaults
- `src/main/cli/settings.c` - CLI settings table
- `src/main/cli/cli.c` - CLI command handlers
- `src/main/flight/autotune_v2/autotune_core.c` - Main state machine
- `src/main/flight/autotune_v2/autotune_core.h` - Public API
- `docs/autotune_v2/phases/PHASE_5_POLISH.md` - Detailed spec

---

## Phase 5 Overview

Phase 5 prepares Autotune V2 for production:
1. **CLI Settings** - Add all autotune settings to CLI
2. **Status Command** - Add `autotune status` command
3. **Timeout Handling** - Add graceful timeout exits
4. **Edge Cases** - Handle repeated failures, gain limits
5. **tuneFeedforward Config** - Add missing config field
6. **Documentation** - User quick start guide

---

## Task Breakdown

Execute these tasks in order using subagents:

### Task 5.1: Add tuneFeedforward to Config
**File:** `src/main/pg/autotune.h`

The code references `autotuneConfig()->tuneFeedforward` but it's missing from the struct. Add it:

```c
typedef struct autotuneConfig_s {
    uint8_t enabled;            // Autotune enabled (0 = off, 1 = on)
    uint8_t axes;               // Axis bitmask: 1=Roll, 2=Pitch, 4=Yaw (7=all)
    uint8_t aggressiveness;     // Aggressiveness 0-100 (50 = balanced)
    uint8_t pStepPercent;       // Base P step size percentage (default 10)
    uint8_t dStepPercent;       // Base D step size percentage (default 10)
    uint8_t fStepPercent;       // Base F step size percentage (default 5)
    uint8_t tuneFeedforward;    // Enable F tuning (0 = skip, 1 = tune)  // ADD THIS
    uint8_t minEvents;          // Minimum events before deciding (default 1)
    uint8_t maxEventsPerAxis;   // Maximum events per axis before abort (default 10)
    uint8_t stickThreshold;     // Minimum stick deflection degrees (default 15)
    uint8_t crossAxisThreshold; // Maximum cross-axis deflection degrees (default 10)
    uint8_t overshootTargetLow; // Lower bound of target overshoot % (default 5)
    uint8_t overshootTargetHigh;// Upper bound of target overshoot % (default 10)
} autotuneConfig_t;
```

**File:** `src/main/pg/autotune.c`

Add default value in PG_RESET_TEMPLATE:
```c
    .tuneFeedforward = 1,           // Enable F tuning by default
```

---

### Task 5.2: Add CLI Settings
**File:** `src/main/cli/settings.c`

Find an appropriate location (search for other PG entries) and add autotune settings. Look for how other parameter groups are added (use `PG_AUTOTUNE_CONFIG`).

Add these settings in the settings table (around the end, before the closing):

```c
#ifdef USE_AUTOTUNE_V2
// PG_AUTOTUNE_CONFIG
    { "autotune_enabled",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1 },   PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, enabled) },
    { "autotune_axes",              VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 7 },   PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, axes) },
    { "autotune_aggressiveness",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, aggressiveness) },
    { "autotune_p_step",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 30 },  PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, pStepPercent) },
    { "autotune_d_step",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 30 },  PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, dStepPercent) },
    { "autotune_f_step",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 50 },  PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, fStepPercent) },
    { "autotune_tune_ff",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1 },   PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, tuneFeedforward) },
    { "autotune_stick_threshold",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 50 },  PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, stickThreshold) },
    { "autotune_cross_axis_max",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 30 },  PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, crossAxisThreshold) },
    { "autotune_max_events",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 3, 20 },  PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, maxEventsPerAxis) },
    { "autotune_overshoot_low",     VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 20 },  PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, overshootTargetLow) },
    { "autotune_overshoot_high",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 30 },  PG_AUTOTUNE_CONFIG, offsetof(autotuneConfig_t, overshootTargetHigh) },
#endif
```

**Also add include** at top of settings.c:
```c
#ifdef USE_AUTOTUNE_V2
#include "pg/autotune.h"
#endif
```

---

### Task 5.3: Add Timeout Constants
**File:** `src/main/flight/autotune_v2/autotune_debug.h`

Add timeout constants if not already present:

```c
// ============================================================================
// Timeout Constants
// ============================================================================

#define AUTOTUNE_TOTAL_TIMEOUT_US       (180 * 1000000)  // 3 minutes total
#define AUTOTUNE_AXIS_TIMEOUT_US        (60 * 1000000)   // 60 seconds per axis
#define AUTOTUNE_EVENT_TIMEOUT_US       (30 * 1000000)   // 30 seconds for event
```

---

### Task 5.4: Add Timeout Checking Function
**File:** `src/main/flight/autotune_v2/autotune_core.c`

Add a timeout check function and call it from `autotuneUpdate()`:

```c
static void checkTimeouts(timeUs_t currentTimeUs)
{
    // Check for total timeout
    if (runtime.startTimeUs > 0) {
        uint32_t totalElapsed = cmpTimeUs(currentTimeUs, runtime.startTimeUs);
        if (totalElapsed > AUTOTUNE_TOTAL_TIMEOUT_US) {
            runtime.reasonCode = AUTOTUNE_REASON_ABORT_TIMEOUT;
            transitionToState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
            return;
        }
    }
    
    // Check for per-state timeout
    uint32_t stateElapsed = cmpTimeUs(currentTimeUs, runtime.stateEntryTimeUs);
    
    switch (runtime.masterState) {
        case AUTOTUNE_STATE_HOVER_LOCK:
            if (stateElapsed > AUTOTUNE_HOVER_LOCK_TIMEOUT_US) {
                runtime.reasonCode = AUTOTUNE_REASON_ABORT_TIMEOUT;
                transitionToState(AUTOTUNE_STATE_IDLE, currentTimeUs);
            }
            break;
            
        case AUTOTUNE_STATE_PD_RATIO_SEEK:
        case AUTOTUNE_STATE_PD_SCALE_UP:
        case AUTOTUNE_STATE_F_TUNE:
            if (stateElapsed > AUTOTUNE_AXIS_TIMEOUT_US) {
                runtime.reasonCode = AUTOTUNE_REASON_ABORT_TIMEOUT;
                advanceToNextAxisOrComplete(currentTimeUs);
            }
            break;
            
        default:
            break;
    }
}
```

Add `startTimeUs` to the runtime struct if missing, and set it when autotune starts.

Call `checkTimeouts(currentTimeUs);` at the beginning of `autotuneUpdate()` when active.

---

### Task 5.5: Add Status Command
**File:** `src/main/cli/cli.c`

Add a CLI command to show autotune status. Find the command table and add:

```c
#ifdef USE_AUTOTUNE_V2
static void cliAutotune(const char *cmdName, char *cmdline);
#endif
```

Add to command table:
```c
#ifdef USE_AUTOTUNE_V2
    CLI_COMMAND_DEF("autotune", "autotune status", NULL, cliAutotune),
#endif
```

Implement the command:
```c
#ifdef USE_AUTOTUNE_V2
#include "flight/autotune_v2/autotune_core.h"

static const char * const autotuneStateNames[] = {
    "IDLE", "HOVER_LOCK", "THROTTLE_SWEEP", "NOISE_CONFIRM",
    "PD_RATIO_SEEK", "PD_SCALE_UP", "F_TUNE", "PD_RETUNE_AFTER_F", "COMPLETE"
};

static const char * const axisNames[] = { "ROLL", "PITCH", "YAW" };

static void cliAutotune(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);
    
    cliPrintLinef("Autotune V2 Status");
    cliPrintLinef("==================");
    
    autotuneState_e state = autotuneGetState();
    cliPrintLinef("State: %s", state < AUTOTUNE_STATE_COUNT ? autotuneStateNames[state] : "UNKNOWN");
    cliPrintLinef("Active: %s", autotuneIsActive() ? "Yes" : "No");
    cliPrintLinef("Current Axis: %s", axisNames[autotuneGetCurrentAxis()]);
    cliPrintLinef("Progress: %d%%", autotuneGetProgress());
    cliPrintLinef("Last Reason: %d", autotuneGetReasonCode());
}
#endif
```

---

### Task 5.6: Add Edge Case Handling
**File:** `src/main/flight/autotune_v2/autotune_types.h`

Add `consecutiveBadEvents` field to `axisTuneState_t`:
```c
    uint8_t consecutiveBadEvents;       // Count of consecutive bad events (for repeated failure detection)
```

**File:** `src/main/flight/autotune_v2/autotune_core.c`

In decision functions, add repeated failure detection:
- If `consecutiveBadEvents >= 3`, force ADVANCE instead of continuing
- Reset `consecutiveBadEvents = 0` on GOOD event
- Increment on BAD event

---

### Task 5.7: Create User Documentation
**File:** `docs/autotune_v2/README_USER.md`

Create quick start guide:

```markdown
# Autotune V2 User Guide

## Quick Start

### Setup (CLI)
```
set autotune_enabled = 1
set autotune_axes = 7
set autotune_aggressiveness = 50
save
```

### Configure Mode Switch
In Betaflight Configurator, assign AUTOTUNE to an AUX channel.

### Flying

1. **Take off and hover** at mid-throttle in calm conditions
2. **Flip autotune switch ON**
3. **Wait for hover lock** (hold steady for 1 second)
4. **Perform stick movements:**
   - Roll left, hold briefly, return to center
   - Roll right, hold briefly, return to center
   - Repeat for pitch and yaw

5. **Feedback patterns:**
   - 1 wiggle = axis complete
   - 4 wiggles = all done!

6. **Flip switch OFF** when complete (or to abort)

## Settings Reference

| Setting | Default | Range | Description |
|---------|---------|-------|-------------|
| autotune_enabled | 0 | 0-1 | Enable autotune feature |
| autotune_axes | 7 | 0-7 | Bitmask: 1=Roll, 2=Pitch, 4=Yaw |
| autotune_aggressiveness | 50 | 0-100 | Higher = faster response |
| autotune_tune_ff | 1 | 0-1 | Enable feedforward tuning |
| autotune_stick_threshold | 15 | 5-50 | Min stick deflection (degrees) |

## Troubleshooting

**Nothing happens when I flip the switch**
- Check autotune_enabled is ON
- Verify AUX channel is correctly assigned
- Must be armed and flying

**Tuning aborts immediately**
- Hover more steadily (stick movement detected)
- Reduce autotune_stick_threshold if too sensitive

**Oscillation during tuning**
- This is normal - autotune will detect and rollback
- If persistent, reduce autotune_aggressiveness

## Tips
- Fly in calm conditions (no wind)
- Use fresh battery
- Make clean, deliberate stick movements
- Wait for each wiggle before next movement
```

---

### Task 5.8: Build and Verify
Run `make CONFIG=BETAFPVG473` and fix any compile errors.

---

### Task 5.9: Update PROGRESS.md
Mark Phase 5 complete and update status table.

---

## Existing Infrastructure Reference

### Public API (autotune_core.h)
```c
bool autotuneIsActive(void);
autotuneState_e autotuneGetState(void);
uint8_t autotuneGetCurrentAxis(void);
uint8_t autotuneGetProgress(void);
uint16_t autotuneGetReasonCode(void);
autotuneDecision_e autotuneGetLastDecision(void);
void autotuneAbort(uint16_t reasonCode);
```

### Parameter Group
```c
autotuneConfig()->enabled           // uint8_t 0-1
autotuneConfig()->axes              // uint8_t bitmask 0-7
autotuneConfig()->aggressiveness    // uint8_t 0-100
autotuneConfig()->tuneFeedforward   // uint8_t 0-1 (to be added)
```

### CLI Pattern
Settings use this format:
```c
{ "setting_name", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { min, max }, PG_ID, offsetof(struct_t, field) },
```

---

## Execution Instructions

1. **Start with Task 5.1** - Add missing config field first
2. **Build after each task** to catch errors early
3. **Use subagents** for each task with specific file/function context
4. **Update todo list** after completing each task
5. **Final build** must pass with no errors
6. **Update PROGRESS.md** as final step

---

## Success Criteria

- [ ] `tuneFeedforward` field added to config
- [ ] All autotune settings accessible via CLI
- [ ] `autotune status` command works
- [ ] Timeout handling prevents infinite loops
- [ ] Edge case handling for repeated failures
- [ ] User documentation created
- [ ] Build passes cleanly for BETAFPVG473
- [ ] PROGRESS.md shows Phase 5 complete

---

*End of Phase 5 Orchestration Prompt*
