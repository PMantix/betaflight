# Autotune Milestone 4 Orchestrator Prompt

Use this prompt with the `runSubagent` tool to orchestrate implementation of Milestone 4 features.

---

## Orchestrator Prompt

```
You are a senior engineering manager orchestrating the implementation of Betaflight autotune improvements. Your role is to delegate tasks to specialist subagents who will implement specific features.

## Context

Project: Betaflight autotune system (flight controller firmware)
Language: C (embedded, no dynamic allocation, -Werror strict)
Build: `make CONFIG=BETAFPVG473`
Roadmap: MILESTONE_4_ROADMAP.md
Progress: MILESTONE_4_PROGRESS.md

## Current Task: [TASK_ID]

Read MILESTONE_4_ROADMAP.md to understand the full requirements for task [TASK_ID].

## Your Responsibilities

1. READ the roadmap section for task [TASK_ID] completely
2. READ the relevant source files to understand current implementation
3. IMPLEMENT the feature according to the acceptance criteria
4. VERIFY the build compiles without errors
5. UPDATE MILESTONE_4_PROGRESS.md with your results

## Key Files

- src/main/flight/autotune.c - Main state machine (~2200 lines)
- src/main/flight/autotune_gains.c - Gain adjustment logic (~1100 lines)  
- src/main/flight/autotune_analysis.c - Response analysis (~500 lines)
- src/main/flight/autotune_types.h - Type definitions (~600 lines)

## Code Standards

1. No dynamic memory allocation (malloc/free)
2. Use existing patterns from the codebase
3. Add reason codes (defined in autotune_types.h) for debug output
4. Maximum function length ~100 lines
5. Comments for non-obvious logic
6. Build must pass with `make CONFIG=BETAFPVG473`

## Output Format

When complete, provide:
1. Summary of changes made
2. Files modified
3. New functions added
4. Acceptance criteria status (checked/unchecked)
5. Build status (PASS/FAIL)
6. Any issues or blockers encountered
```

---

## Task-Specific Prompts

### P0: Fix Attribution Order (CRITICAL)

```
TASK_ID: P0
FEATURE: Fix F-Bias in Attribution Logic

The attribution logic currently checks F-term issues BEFORE P/D response classification. This causes F to be adjusted almost exclusively while P/D are starved. Flight test showed: F changed 21 times, P changed 1 time, D changed 0 times.

SPECIFIC REQUIREMENTS:
1. In autotune_analysis.c, find function autotuneAttributeResponse() (around line 450-680)

2. CURRENT ORDER (broken):
   - Phase 1: I-term issues (bounceback, slow oscillation) ~line 485-515
   - Phase 2: F-term issues (velocityWeightedLag checks) ~line 520-575
   - Phase 3: P/D based on response class (switch statement) ~line 580-680

3. CHANGE TO:
   - Phase 1: I-term issues (unchanged)
   - Phase 2: P/D based on response class (MOVE UP from Phase 3)
   - Phase 3: F-term issues (MOVE DOWN, only for EXCELLENT/CRITICAL)

4. Key changes:
   a. Move the switch(responseClass) block BEFORE the F-term checks
   b. For UNDERDAMPED and OVERDAMPED cases, return immediately after attribution
   c. Only check velocityWeightedLag for RESPONSE_EXCELLENT and RESPONSE_CRITICAL cases
   d. The highVelocityManeuver check should only apply when P/D are already good

5. Add guard at start of F-term section:
   ```c
   // Only check F-term issues if P/D response is already acceptable
   if (responseClass != RESPONSE_EXCELLENT && 
       responseClass != RESPONSE_CRITICAL &&
       responseClass != RESPONSE_NOISY) {
       // P/D needs work first, skip F-term attribution
       return;
   }
   ```

VERIFICATION:
- Build must pass with `make CONFIG=BETAFPVG473`
- The switch(responseClass) for UNDERDAMPED should attribute to P or D
- F attribution should only happen when response is already good

REFERENCE: Look at the existing code structure - just reordering, minimal new code.
```

### P1: Multi-Variable DOE

```
TASK_ID: P1
FEATURE: Multi-Variable DOE for Hover Tune

Implement a linear combination solver that uses ALL test sensitivities to compute optimal parameter adjustments in a single step.

SPECIFIC REQUIREMENTS:
1. In autotune.c, after HOVER_DIAG_ANALYZING completes:
   - Calculate sensitivity for each test: sensitivity = improvement / parameter_change
   - Store sensitivities in an array
2. Add new function applyMultiVariableFix():
   - Input: baseline_rms, target_rms, sensitivities[4], current_params[4]
   - Compute: Δparam[i] = (baseline - target) × sensitivity[i] / Σ(sensitivity²)
   - Apply 90% damping: actual_Δ = 0.90 × computed_Δ
   - Clamp to limits (min/max frequencies, gains)
3. Replace single-fix logic with multi-variable fix
4. Update debug output to show which parameters changed

REFERENCE: Look at how applyDiagnosticFix() currently works and enhance it.
```

### P2: Bidirectional Exploration

```
TASK_ID: P2
FEATURE: Bidirectional Gain Exploration

Add relaxation tests that try RAISING parameters when noise is excellent.

SPECIFIC REQUIREMENTS:
1. Add new phases to hoverDiagPhase_e enum:
   - HOVER_DIAG_RELAX_ROLL (test 1.5× roll gains)
   - HOVER_DIAG_RELAX_PITCH (test 1.5× pitch gains)
   - HOVER_DIAG_RELAX_GYRO_LPF1 (test +50Hz)
   - HOVER_DIAG_RELAX_DTERM_LPF1 (test +25Hz)
2. After standard DOE, if baseline RMS < MOTOR_RMS_EXCELLENT (8.0):
   - Run relaxation tests
   - If noise stays < 15, apply the increases
3. Add reason codes 114x for relaxation actions (already defined in autotune_types.h)
4. Update state machine to handle new phases

REFERENCE: Look at existing HOVER_DIAG_ROLL_TEST implementation and mirror it.
```

### P3: Critical Damping Detection

```
TASK_ID: P3
FEATURE: Critical Damping Detection

Calculate actual damping ratio (ζ) from response characteristics.

SPECIFIC REQUIREMENTS:
1. In autotune_analysis.c, add:
   float calculateDampingFromOvershoot(float overshootPercent) {
       if (overshootPercent <= 0) return 1.0f;  // No overshoot = critically damped or overdamped
       float os = overshootPercent / 100.0f;
       float lnOs = logf(os);
       return -lnOs / sqrtf(M_PI * M_PI + lnOs * lnOs);
   }

2. In autotune_types.h, add to autotuneMetrics_t:
   float dampingRatio;  // ζ: <0.7 underdamped, 0.7-1.0 optimal, >1.0 overdamped

3. In autotuneAnalyzeResponse(), calculate and store damping ratio

4. Update autotuneClassifyResponse() to use damping ratio:
   - ζ < 0.5: RESPONSE_UNDERDAMPED (bouncy)
   - ζ 0.5-0.7: RESPONSE_CRITICAL (snappy, slight overshoot)
   - ζ 0.7-1.0: RESPONSE_EXCELLENT (optimal)
   - ζ > 1.0: RESPONSE_OVERDAMPED (sluggish)

REFERENCE: Standard control theory damping ratio formulas.
```

### P4: P/D Push-Up Logic

```
TASK_ID: P4
FEATURE: P/D Push-Up Logic
DEPENDS_ON: P3 (damping ratio calculation)

Add explicit gain increase phase when headroom exists.

SPECIFIC REQUIREMENTS:
1. In autotune_gains.c, add:
   bool autotunePushGainsUp(autotuneRuntime_t *runtime, uint16_t *reasonCode)
   
2. Call this after normal gain adjustment when:
   - runtime->metrics.noiseRms < 20.0f (headroom)
   - runtime->metrics.overshootPercent < 12.0f (can tolerate more)
   - runtime->responseClass != RESPONSE_UNDERDAMPED

3. Push-up logic:
   - Try +15% P, re-analyze
   - If oscillation detected, add +15% D to compensate
   - If noise exceeds 25, revert and stop
   - Target: 8-12% overshoot, ζ = 0.7-0.8

4. Add reason codes for push-up actions (may need new 37xx codes)

REFERENCE: Look at existing autotuneApplyGainAdjustment() structure.
```

### P5: Integrated Filter+PID

```
TASK_ID: P5
FEATURE: Integrated Filter+PID Tuning
DEPENDS_ON: P1 (multi-variable approach)

Adjust filters inline during PID tuning.

SPECIFIC REQUIREMENTS:
1. In autotuneApplyGainAdjustment(), before applying PID changes:
   - Check runtime->metrics.noiseRms
   - If > 25: reduce dterm_lpf1 by 10Hz, set reason code 2210
   - If < 10: raise dterm_lpf1 by 10Hz, set reason code 2310

2. Ensure filter limits are respected (DTERM_LPF1_MIN/MAX_HZ)

3. Log filter changes alongside PID changes

4. This creates coordinated filter+PID optimization

REFERENCE: Look at autotuneApplyFilterAdjustment() for filter change patterns.
```

### P6: Notch Configuration

```
TASK_ID: P6
FEATURE: Automatic Notch Configuration

Apply notch filters when resonance detected above LPF range.

SPECIFIC REQUIREMENTS:
1. In autotune_gains.c, add:
   bool autotuneApplyNotchFix(const autotuneFilterAnalysis_t *analysis, uint16_t *reasonCode)

2. Trigger when:
   - analysis->resonanceDetected == true
   - analysis->peakFrequency > current gyro_lpf2_hz (can't filter with LPF)
   - analysis->peakAmplitude > threshold (significant resonance)

3. Configure notch:
   - dterm_notch_hz = analysis->suggestedNotchHz
   - dterm_notch_cutoff = suggestedNotchHz * 0.7 (Q factor)

4. After applying, verify with 500ms measurement
   - If noise improved, keep
   - If noise worse or unchanged, revert

5. Add reason code 2120 (REASON_FILTER_RESONANCE_NOTCH)

REFERENCE: Look at resonance detection in autotuneAnalyzeNoise().
```

---

## Usage Example

To start implementing P1, use runSubagent with:

```
runSubagent(
  description: "Implement P1 Multi-Variable DOE",
  prompt: "[Full orchestrator prompt above] + [P1 task-specific prompt]"
)
```

After the agent completes, update MILESTONE_4_PROGRESS.md with results, then move to the next task.
