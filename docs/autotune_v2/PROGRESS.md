# Autotune V2 Progress Tracker

**Last Updated:** January 19, 2026  
**Current Phase:** 0 - Foundation  
**Current Status:** ⬜ NOT STARTED

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
| 0 | Foundation | ⬜ Not Started | 0% |
| 1 | Filter Characterization | ⬜ Not Started | 0% |
| 2 | Event Detection | ⬜ Not Started | 0% |
| 3 | PD Ratio Seek | ⬜ Not Started | 0% |
| 4 | PD Scale Up & F Tune | ⬜ Not Started | 0% |
| 5 | Polish | ⬜ Not Started | 0% |

**Overall Progress:** 0%

---

## Current Focus

### Active Work
<!-- Update this section when starting work -->

**Currently working on:** Nothing yet - project not started

**Blocked by:** Nothing

**Next step:** Begin Phase 0 - create branch and type definitions

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
| 0.1 Create clean branch | ⬜ | Need to branch from upstream main |
| 0.2 Create autotune_types.h | ⬜ | All V2 type definitions |
| 0.3 Create module skeletons | ⬜ | Empty .c/.h files |
| 0.4 Create parameter group | ⬜ | autotune_pg.c/h |
| 0.5 Build passes | ⬜ | With empty modules |
| 0.6 Update Makefile | ⬜ | Add V2 sources |
| 0.7 Create debug constants | ⬜ | Reason codes, debug macros |

---

## Phase 1: Filter Characterization

| Task | Status | Notes |
|------|--------|-------|
| 1.1 Hover detection | ⬜ | |
| 1.2 Throttle sweep detection | ⬜ | |
| 1.3 Noise measurement | ⬜ | |
| 1.4 Filter adjustment logic | ⬜ | |
| 1.5 Integration test | ⬜ | |

---

## Phase 2: Event Detection

| Task | Status | Notes |
|------|--------|-------|
| 2.1 Stick deflection detection | ⬜ | |
| 2.2 Cross-axis rejection | ⬜ | |
| 2.3 Event quality gates | ⬜ | |
| 2.4 Sample buffer management | ⬜ | |
| 2.5 Integration test | ⬜ | |

---

## Phase 3: PD Ratio Seek

| Task | Status | Notes |
|------|--------|-------|
| 3.1 Overshoot calculation | ⬜ | |
| 3.2 Rebound detection | ⬜ | |
| 3.3 Bracket state management | ⬜ | Bracket-first algorithm |
| 3.4 Newton refinement | ⬜ | Newton-second algorithm |
| 3.5 Rollback system | ⬜ | |
| 3.6 Integration test | ⬜ | |

---

## Phase 4: PD Scale Up & F Tune

| Task | Status | Notes |
|------|--------|-------|
| 4.1 Scale history tracking | ⬜ | Newton-primary approach |
| 4.2 Lag measurement | ⬜ | |
| 4.3 PD scale up decision | ⬜ | |
| 4.4 F history tracking | ⬜ | Cautious Newton |
| 4.5 F tune decision | ⬜ | |
| 4.6 Integration test | ⬜ | |

---

## Phase 5: Polish

| Task | Status | Notes |
|------|--------|-------|
| 5.1 CLI commands | ⬜ | |
| 5.2 OSD integration | ⬜ | |
| 5.3 Full flight test | ⬜ | |
| 5.4 Documentation update | ⬜ | |

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
