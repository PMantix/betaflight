# SUB-AGENT TASK: State Deep Dive (Single State)

You are assisting the State Workflow Lead Engineer. Analyze ONE state in the auto-tuning PID controller state machine (C++).

## State to analyze
- State name: <STATE_NAME>

## Inputs Provided
- Relevant code snippets/files: <PASTE HERE>
- PRD excerpt relevant to this state (if available): <PASTE HERE>

## Deliverable (structured response required)
Return the following sections:

1) State Mission
- PRD-stated (quote/paraphrase) OR inferred (mark inferred)

2) Entry Conditions (as implemented)
- Guards/conditions required to enter
- Where enforced (file + function/symbol + line if available)
- Any mode/axis dependencies

3) Actions/Side Effects in State
- What the code does while in this state (high-level)
- Signals/metrics computed or updated
- Any actuator/control-affecting behavior

4) Exit Conditions (as implemented)
- Success exits: guard → next state (with code refs)
- Failure/abort exits: guard → next state (with code refs)
- Timeout exits: if present (with code refs)

5) Risks / Issues
- Sticky risk (no plausible exits)
- Conflicting guards / order dependence
- Missing timeout / missing fallback
- Unclear mission vs behavior mismatch

6) Suggested Fix Patterns (no full code)
- If issues found, propose approach-level fixes + what to validate

## Constraints
- Evidence-first: cite code location for every entry/exit path you claim.
- If you lack needed code, state what’s missing and provide partial findings.
