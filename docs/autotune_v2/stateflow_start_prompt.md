# TASK: State Flow Assessment Run

You are the State Workflow Lead Engineer. Produce a State Flow Assessment document for the auto-tuning PID controller.

## Context
- Codebase: C++ (Betaflight/flight-controller style)
- Transition logic may be distributed across helper functions and nested branches.
- Current state is tracked by explicit state enum/flag.

## Inputs Provided
- PRD markdown: <PASTE OR LINK CONTENT HERE>
- Code context: <PASTE RELEVANT FILES OR SNIPPETS HERE>
- Change context (optional): 
  - PR summary: <...>
  - commit hash/diff snippet: <...>

## What to do
1) Identify the state enum/flag and the state update/tick entrypoints.
2) Enumerate all states and all assignments to the current state.
3) Reconstruct the real transition graph:
   - list edges with guard + code location
4) Detect axes/modes that change the flow.
   - group equivalent modes; avoid state explosion
5) Produce Mermaid diagrams:
   - common path
   - mode/axis divergences as deltas/subdiagrams
6) Write per-state contracts:
   - mission (PRD or inferred)
   - entry/exit conditions
   - exit success/failure/timeout semantics (as implemented)
7) Run correctness checks:
   - unreachable states
   - sticky states
   - cycles without progress
   - conflicting guards or order dependence
   - unsafe fallbacks / missing abort path
8) Compare to PRD:
   - map coverage and mismatches
   - classify mismatches (PRD underspecified vs code bug likely vs implementation detail)
9) Produce an issue list suitable for spinning up fix agents.

## Required Output
Return a single markdown document titled:
“State Flow Assessment — <component/module name> — <date or commit>”

Include these sections in order:
1) Executive Summary
2) State Inventory
3) State Flow Diagram(s) (Mermaid)
4) Per-State Contracts
5) Graph Correctness Checks
6) PRD Traceability + Mismatch Analysis
7) Issue List (handoff-ready)

## Constraints
- Evidence-first: every diagram edge and issue must cite code location (file + function/symbol; line numbers if available).
- Do not assume PRD or code is correct when they disagree—reconstruct intent vs reality and judge the gap.
- If code context is incomplete, list exactly what is missing and proceed with best-effort partial assessment using available evidence.
