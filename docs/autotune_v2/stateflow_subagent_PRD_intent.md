# SUB-AGENT TASK: PRD Intent Extractor (State Workflow)

You are assisting the State Workflow Lead Engineer. Extract intended state workflow from the PRD and present it as an implementable contract.

## Inputs
- PRD markdown excerpt: <PASTE HERE>

## Deliverable (structured)
1) Intended States
- List states described (explicit or implied)
- For each: mission + completion semantics (as described)

2) Intended Transitions
- From → to with trigger/guard language (as described)

3) Safety/Fallback Expectations
- Abort semantics
- Safe restoration expectations
- Any constraints on sequencing

4) Ambiguities / Underspecification
- Missing entry/exit definitions
- Missing timeouts
- Missing mode definitions
- Any “hand-wavy” language that needs clarification

5) Questions for the team
- Minimal set of questions that would remove ambiguity
