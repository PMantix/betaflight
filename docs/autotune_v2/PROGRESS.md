# Autotune V2 Progress Tracker

**Last Updated:** January 20, 2026  
**Current Phase:** Complete  
**Current Status:** ✅ All phases complete, ready for flight testing

---

## Purpose of This Document

This is the **single source of truth** for project progress. Every development session should:
1. **START** by reading this document
2. **END** by updating this document

This prevents context loss between sessions and ensures AI agents can quickly understand where we are.

---

## Quick Status

| Phase | Name | Status | Completion |
|-------|------|--------|------------|
| 0 | Foundation | ✅ Complete | 100% |
| 1 | Filter Characterization | ✅ Complete | 100% |
| 2 | Event Detection | ✅ Complete | 100% |
| 3 | PD Ratio Seek | ✅ Complete | 100% |
| 4 | PD Scale Up & F Tune | ✅ Complete | 100% |
| 5 | Polish | ✅ Complete | 100% |

**Overall Progress:** 100% (6/6 phases)

---

## Current Focus

### Active Work
<!-- Update this section when starting work -->

**Currently working on:** Flight testing and validation

**Blocked by:** Nothing

**Next step:** Flash firmware, test in flight, gather logs

---

## Completed Milestones

<!-- Add entries as milestones are completed -->

### Documentation (Pre-Implementation)
- [x] PRD.md - Product requirements defined
- [x] ARCHITECTURE.md - Technical design with bracket+Newton algorithms
- [x] Phase documents created (PHASE_0 through PHASE_5)
- [x] LOG_ANALYSIS.md - Blackbox log analysis guide
- [x] SESSION_BOOTSTRAP.md - New session onboarding
- [x] AGENT_ORCHESTRATION.md - AI agent usage guide
- [x] Consistent debug channel mapping across all docs
- [x] Blackbox CSV header info documented (147 rows, data at 148)

---

## Phase 0: Foundation

| Task | Status | Notes |
|------|--------|-------|
| 0.1 Create clean branch | ✅ | `autotune-v2-clean` from upstream master |
| 0.2 Create autotune_types.h | ✅ | All enums, structs, fixed-point macros |
| 0.3 Create module skeletons | ✅ | core, event, filter, metrics, rollback, feedback |
| 0.4 Create parameter group | ✅ | autotune_pg.c/h with CLI settings |
| 0.5 Build passes | ✅ | Compiles clean for BETAFPVG473 |
| 0.6 Update Makefile | ✅ | V2 sources integrated |
| 0.7 Create debug constants | ✅ | autotune_debug.h with reason codes |

**Phase 0 Completed:** January 19, 2026

---

## Phase 1: Filter Characterization

| Task | Status | Notes |
|------|--------|-------|
| 1.1 Hover detection | ✅ | stateHoverLockUpdate() with stick/throttle checks |
| 1.2 Throttle sweep detection | ✅ | stateThrottleSweepUpdate() tracks range |
| 1.3 Noise measurement | ✅ | measureCurrentNoise() from gyro RMS |
| 1.4 Filter adjustment logic | ✅ | autotuneFilterComputeRecommendations() |
| 1.5 Noise confirm state | ✅ | stateNoiseConfirmUpdate() validates filters |

**Phase 1 Completed:** January 19, 2026

---

## Phase 2: Event Detection

| Task | Status | Notes |
|------|--------|-------|
| 2.1 Stick deflection detection | ✅ | autotuneEventUpdate() state machine |
| 2.2 Cross-axis rejection | ✅ | crossAxisMax tracked via new parameter |
| 2.3 Event quality gates | ✅ | autotuneEventCheckQuality() with 4 gates |
| 2.4 Sample buffer management | ✅ | eventBuffer with gyro/setpoint capture |
| 2.5 Overshoot measurement | ✅ | autotuneMetricsComputeOvershoot() |
| 2.6 Rebound detection | ✅ | autotuneMetricsDetectRebound() |
| 2.7 Settling time metric | ✅ | autotuneMetricsComputeSettlingTime() |
| 2.8 Lag metric | ✅ | autotuneMetricsComputeLag() |
| 2.9 Integration test | ✅ | statePdRatioSeekUpdate() wired up |

**Phase 2 Completed:** January 19, 2026

---

## Phase 3: PD Ratio Seek

| Task | Status | Notes |
|------|--------|-------|
| 3.1 Bracket state types | ✅ | bracketState_t in autotune_types.h |
| 3.2 Decision function | ✅ | pdRatioSeekDecision() with bracket+Newton |
| 3.3 Gain application | ✅ | applyPGain() updates pidProfile |
| 3.4 State enter function | ✅ | statePdRatioSeekEnter() inits bracket/pState |
| 3.5 State update function | ✅ | statePdRatioSeekUpdate() calls decision, applies gains |
| 3.6 Build verification | ✅ | Compiles clean for BETAFPVG473 |

**Phase 3 Completed:** January 19, 2026

---

## Phase 4: PD Scale Up & F Tune

| Task | Status | Notes |
|------|--------|-------|
| 4.1 Scale history types | ✅ | scaleHistory_t, fHistory_t in autotune_types.h |
| 4.2 statePdScaleUpEnter | ✅ | Captures ratioSeekP/D, inits scale tracking |
| 4.3 pdScaleUpDecision | ✅ | Newton-primary with applyScale() helper |
| 4.4 statePdScaleUpUpdate | ✅ | Full event loop wired up |
| 4.5 stateFTuneEnter | ✅ | Saves pre-F baseline, inits F rollback |
| 4.6 fTuneDecision | ✅ | Cautious Newton with 15% max step |
| 4.7 stateFTuneUpdate | ✅ | Full event loop wired up |
| 4.8 PD_RETUNE_AFTER_F | ✅ | Quick validation with 3 event max |
| 4.9 advanceToNextAxisOrComplete | ✅ | Axis loop with wiggle feedback |
| 4.10 Build verification | ✅ | Compiles clean for BETAFPVG473 |

**Phase 4 Completed:** January 20, 2026

---

## Phase 5: Polish

| Task | Status | Notes |
|------|--------|-------|
| 5.1 Add tuneFeedforward config | ✅ | Field added to autotuneConfig_t with default=1 |
| 5.2 CLI settings | ✅ | 12 settings added to settings.c |
| 5.3 Timeout constants | ✅ | TOTAL=3min, AXIS=60s, EVENT=30s |
| 5.4 Timeout checking | ✅ | checkTimeouts() in autotuneUpdate() |
| 5.5 Status CLI command | ✅ | `autotune` command shows state/axis/progress |
| 5.6 Edge case handling | ✅ | consecutiveBadEvents tracking, force advance at 3 |
| 5.7 User documentation | ✅ | README_USER.md quick start guide |
| 5.8 Build verification | ✅ | BETAFPVG473 builds clean |

**Phase 5 Completed:** January 20, 2026

---

## Design Decisions Log

<!-- Record important decisions made during development -->

| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-01-19 | Bracket-first, Newton-second for PD ratio seek | Overshoot is nonlinear threshold; need bounds first |
| 2026-01-19 | Newton-primary for PD scale up | Lag varies smoothly with scale |
| 2026-01-19 | Cautious Newton (15% max step) for F tune | Cross-coupling makes derivatives unreliable |
| 2026-01-19 | Persistent debug channels | Prevent misinterpretation when channels change meaning by state |
| 2026-01-19 | Pilot maneuvers, not injected doublets | More natural, controlled excitation |
| 2026-01-19 | Single event = single decision | No averaging; use rollback for stability |

---

## Open Issues

<!-- Track problems that span sessions -->

| ID | Issue | Severity | Status | Notes |
|----|-------|----------|--------|-------|
| - | None yet | - | - | Project not started |

---

## Blocked Items

<!-- Track items waiting on external dependencies -->

| Item | Blocked By | Since | Resolution |
|------|------------|-------|------------|
| - | Nothing | - | - |

---

## Session Log

<!-- Add an entry at the end of each development session -->

### 2026-01-19 - Phase 2 Complete (Integration)

**What was done:**
- Implemented `statePdRatioSeekUpdate()` in `autotune_core.c`
  - Wired up event detection with full substate machine
  - WAIT_EVENT: Monitors stick position, triggers event detection
  - ANALYZING: Validates quality gates, computes metrics
  - Captures gyro/setpoint samples during event via buffer
  - Logs reject reasons to debug channel on quality gate failure
  - Outputs overshoot metric (×10) to debug channel on success
- Added required includes:
  - `sensors/gyro.h` for `gyro.gyroADCf[]` access
  - `fc/rc.h` for `getSetpointRate()` access
- Integration complete: event detection → metrics → debug output
- Build passes clean for BETAFPVG473

**Files modified:**
- `src/main/flight/autotune_v2/autotune_core.c` - Integration wiring

**What's next:**
- Begin Phase 3: PD Ratio Seek
- Implement bracket search algorithm for P gain
- Store original gains and implement rollback

**Notes:**
- Phase 2 now complete (100%)
- Event detection flow: IDLE → DEFLECTING → RETURNING → COMPLETE
- Substate flow: WAIT_EVENT → (buffer capture) → ANALYZING → WAIT_EVENT
- Ready for Phase 3 tuning decisions

---

### 2026-01-19 - Phase 2 Event Detection Progress

**What was done:**
- Updated `autotuneEventUpdate()` signature to include `crossAxisPosition` parameter
  - Header: `autotune_event.h` updated
  - Implementation: Added tracking in all event states (IDLE, DEFLECTING, RETURNING)
  - Uses `fmaxf()` to track maximum cross-axis movement during event
- Implemented `autotuneMetricsComputeSettlingTime()` in `autotune_metrics.c`
  - Finds final setpoint (average of last 5 samples)
  - Uses 5% settling band (`AUTOTUNE_SETTLING_BAND`)
  - Searches backwards for last sample outside band
  - Returns settling time in milliseconds
- Implemented `autotuneMetricsComputeLag()` in `autotune_metrics.c`
  - Finds final setpoint (average of last 5 samples)
  - Finds when gyro first reaches 50% of target
  - Handles both positive and negative targets
  - Returns lag time in milliseconds
- Updated `autotuneMetricsAnalyze()` to call timing metrics
  - Uses 4000µs sample period (250Hz sampling rate)
  - Properly fills settlingTimeMs and lagMs in output
- Build passes clean for BETAFPVG473

**Files modified:**
- `src/main/flight/autotune_v2/autotune_event.h` - Function signature
- `src/main/flight/autotune_v2/autotune_event.c` - Cross-axis tracking
- `src/main/flight/autotune_v2/autotune_metrics.c` - Timing metrics

**What's next:**
- Wire up event detection in `statePdRatioSeekUpdate()` (Phase 3)
- Call `autotuneEventUpdate()` with stick positions and cross-axis data
- Connect metrics analysis to make tuning decisions

**Notes:**
- Skeleton code from Phase 0 was already well-structured
- Cross-axis tracking was missing - now fully implemented
- Timing metrics (settling, lag) were TODO stubs - now complete

---

### 2026-01-19 - Phase 1 Complete

**What was done:**
- Implemented HOVER_LOCK state detection in `autotune_core.c`
  - Stick centering check (roll, pitch, yaw < 5%)
  - Throttle stability check (within ±5% for 1 second)
  - Timeout handling (10 seconds)
  - Feedback wiggle on success via `autotuneFeedbackHoverLocked()`
- Implemented THROTTLE_SWEEP state in `autotune_core.c`
  - Calls `autotuneFilterUpdate()` with current throttle
  - Tracks throttle range progress in debug channel
  - Triggers `autotuneFeedbackFiltersSet()` on completion
  - Transitions to NOISE_CONFIRM when 50% throttle range covered
- Implemented NOISE_CONFIRM state in `autotune_core.c`
  - Waits 500ms for filter stabilization
  - Checks if noise acceptable via `autotuneFilterNoiseAcceptable()`
  - Transitions to PD_RATIO_SEEK on success
- Implemented noise measurement in `autotune_filter.c`
  - `measureCurrentNoise()` uses gyro RMS from 32-sample buffer
  - Tracks noise by throttle band (10% increments)
  - Hover noise tracked with exponential moving average
- Implemented filter recommendations in `autotune_filter.c`
  - Based on max noise: >40→150Hz, >20→200Hz, else 250Hz LPF
- Added `#include <math.h>` for `fabsf()` and `sqrtf()` functions
- Fixed type comparison issue (`timeUs_t` vs `timeDelta_t`)
- Build passes clean for BETAFPVG473

**Files modified:**
- `src/main/flight/autotune_v2/autotune_core.c` - State handlers
- `src/main/flight/autotune_v2/autotune_filter.c` - Noise/filter logic

**What's next:**
- Begin Phase 2: Event Detection
- Implement stick deflection detection
- Implement cross-axis rejection quality gates

**Notes:**
- Much of Phase 1 foundation was already in place from Phase 0
- State flow: IDLE → HOVER_LOCK → THROTTLE_SWEEP → NOISE_CONFIRM → PD_RATIO_SEEK

---

### 2026-01-19 - Phase 0 Complete

**What was done:**
- Created `autotune-v2-clean` branch from upstream betaflight master (no v1 code)
- Implemented all type definitions in `autotune_types.h`
- Created module skeletons: core, event, filter, metrics, rollback, feedback
- Created parameter group (`autotune_pg.c/h`) with CLI settings
- Created debug infrastructure (`autotune_debug.h`)
- Build passes clean for BETAFPVG473 target

**Files created:**
- `src/main/flight/autotune_v2/autotune_types.h`
- `src/main/flight/autotune_v2/autotune_core.c/h`
- `src/main/flight/autotune_v2/autotune_debug.h`
- `src/main/flight/autotune_v2/autotune_event.c/h`
- `src/main/flight/autotune_v2/autotune_filter.c/h`
- `src/main/flight/autotune_v2/autotune_metrics.c/h`
- `src/main/flight/autotune_v2/autotune_rollback.c/h`
- `src/main/flight/autotune_v2/autotune_feedback.c/h`
- `src/main/pg/autotune.c/h`

**Files modified:**
- `src/main/pg/pg_ids.h` - Added `PG_AUTOTUNE_CONFIG` (ID 561)
- `src/main/build/debug.h` - Added `DEBUG_AUTOTUNE_V2` enum
- `src/main/build/debug.c` - Added debug mode name
- `src/platform/STM32/include/platform/platform.h` - Added `USE_AUTOTUNE_V2` for STM32G4
- `mk/source.mk` - Added autotune_v2 source files to build

**What's next:**
- Begin Phase 1: Hover lock detection and throttle sweep
- Implement filter characterization

**Notes:**
- Branch `autotune-v2-clean` pushed to origin
- Old v1 code in fork's master was NOT used (fresh from upstream)

---

### 2026-01-19 - Documentation Setup

**What was done:**
- Created all V2 documentation structure
- Defined PRD, Architecture, Phase documents
- Established bracket+Newton optimization strategy
- Fixed debug channel consistency across docs
- Added blackbox CSV header info

**What's next:**
- Begin Phase 0: Create branch, types, skeleton files

**Notes:**
- Branch created: `autotune-v2-docs` (docs only, based on master)
- Old work stashed from `autotune-simplified` branch

---

## How to Update This Document

### At Session Start
1. Read "Current Focus" section
2. Check "Blocked Items" and "Open Issues"
3. Continue from where last session ended

### At Session End
1. Update task status in relevant phase section
2. Update "Current Focus" with what you worked on
3. Add entry to "Session Log"
4. Move any new issues to "Open Issues"
5. Update completion percentages in "Quick Status"

### Status Legend
- ⬜ Not Started
- 🔄 In Progress
- ✅ Complete
- ❌ Blocked
- ⚠️ Needs Review

---

*This document is the project's memory. Keep it current.*
