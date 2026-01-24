# SUB-AGENT TASK: Mode/Axis Divergence Analyzer

You are assisting the State Workflow Lead Engineer. Identify and summarize axes/modes/flags that materially change the state workflow (transitions or state actions).

## Inputs
- PRD excerpt: <PASTE HERE>
- Code excerpts: <PASTE HERE>

## Deliverable (structured)
1) Axes/Modes Discovered
For each axis/mode:
- Name and representation (enum/flag/config)
- Where set/updated
- Where referenced in transition logic

2) Transition Differences (Deltas)
- List transitions that differ under this axis/mode:
  - base behavior
  - mode-specific behavior
  - code references for both

3) Grouping Recommendation
- Which modes appear equivalent (same transition structure)
- Which modes need their own subdiagram

4) Risks
- Modes that create unreachable or sticky states
- Modes that bypass safety/abort paths
