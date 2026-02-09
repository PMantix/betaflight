# Phase 2 Test Plan: P/D Ratio Tuning

> **Date:** 2026-02-08
> **Prerequisite:** Phase 1 F-term tuning verified working
> **Firmware:** ff-autotune-v1 branch with Phase 2 implementation (Schmitt trigger fix applied)
> **PG Version:** 3

---

## Pre-Flight Checklist

### 1. Build Verification

- [ ] Firmware compiles for target without errors
- [ ] Flash size within budget
- [ ] No new warnings in build output

### 2. CLI Configuration

```
# Verify Phase 1 is enabled and has been tested
get ff_autotune_enabled
# Should be ON

# Enable Phase 2
set ff_autotune_pd_enabled = ON

# Verify Phase 2a defaults loaded
get ff_autotune_ring_window_ms
# Should be 150
get ff_autotune_ring_threshold
# Should be 20
get ff_autotune_ring_deadband
# Should be 5
get ff_autotune_p_step
# Should be 2
get ff_autotune_d_step
# Should be 1
get ff_autotune_p_adjust_max
# Should be 10
get ff_autotune_d_adjust_max
# Should be 5

# Verify Phase 2b defaults loaded
get ff_autotune_noise_threshold
# Should be 10
get ff_autotune_scale_step
# Should be 1
get ff_autotune_scale_max
# Should be 8

# Verify learned adjustments start at zero
get ff_autotune_p_adj_roll
get ff_autotune_p_adj_pitch
get ff_autotune_d_adj_roll
get ff_autotune_d_adj_pitch
get ff_autotune_scale_adj_roll
get ff_autotune_scale_adj_pitch
# All should be 0

# Set debug mode for blackbox logging
set debug_mode = FF_AUTOTUNE
save
```

### 3. Blackbox Setup

- [ ] Blackbox logging enabled
- [ ] SD card has free space
- [ ] Debug mode set to `FF_AUTOTUNE`
- [ ] Logging rate sufficient (2kHz recommended for ringing visibility)

### 4. AUX Switch

- [ ] AUX switch assigned to FF AUTOTUNE mode
- [ ] Switch verified in Configurator modes tab

---

## Test 1: Phase Transition (Ground/Bench Test)

**Goal:** Verify Phase 1 -> Phase 2 transition occurs after Phase 1 convergence.

**Procedure:**
1. Arm with autotune switch OFF
2. Activate autotune switch
3. Perform 8-12 aggressive pitch maneuvers to converge Phase 1
4. Watch `debug[3]` (Phase):
   - Should start at 0 (Phase 1)
   - Should transition to 1 (Phase 2a) after F-term converges
5. Deactivate autotune switch
6. Verify beeper sounds (save confirmation)
7. Check CLI: `get ff_autotune_gain_pitch` should have a non-zero learned value

**Pass Criteria:**
- [ ] Phase transitions from 0 to 1 visible in blackbox (`debug[3]`)
- [ ] Phase does not transition to 1 until Phase 1 bracket is converged
- [ ] Beeper sounds on deactivation

---

## Test 2: Ringing Window Detection

**Goal:** Verify the ringing analysis window opens and closes correctly.

**Procedure:**
1. Ensure Phase 2a is active (`debug[3]` = 1, Phase 1 must have converged)
2. With `debug_mode = FF_AUTOTUNE`, perform a maneuver:
   - Quick pitch forward, hold for ~500ms, release
3. In blackbox, check `debug[2]` (Window State):
   - Should cycle through 0(IDLE) -> 1(RISING) -> 2(ADJUSTING) -> 3(WAITING) -> 0(IDLE)
   - Ringing analysis happens during the ADJUSTING and WAITING phases
4. Check `debug[4]` (Ringing Score) - should show a non-zero value after the window closes
5. Check `debug[7]` (Ring Assessment) - should show 0, 1, or 2

**Pass Criteria:**
- [ ] Window state transitions visible in `debug[2]`
- [ ] The ring_window_ms period starts at the RISING -> ADJUSTING transition
- [ ] Window closes before next maneuver starts
- [ ] Ringing score and assessment update after each maneuver

---

## Test 3: Ringing Score and Assessment (Schmitt Trigger Verification)

**Goal:** Verify ringing detection with Schmitt trigger produces reasonable scores.

**Procedure:**
1. With Phase 2a active (`debug[3]` = 1), perform 5-6 aggressive pitch maneuvers
2. After each maneuver, check:
   - `debug[4]` (Ringing Score) - should be non-zero when ringing is visible
   - `debug[7]` (Ring Assessment):
     - 0 = WELL_DAMPED (score below threshold/2)
     - 1 = MILD (score between threshold/2 and threshold)
     - 2 = RINGING (score above threshold)
3. Cross-reference: look at the raw gyro and setpoint traces during the plateau
   - Visible oscillation should correspond to higher ringing scores
   - Smooth settling should correspond to lower scores
4. **Schmitt trigger validation**: Verify `debug[4]` is **not always zero** — the old consecutive-sample comparison bug always produced 0. Any non-zero ringing score confirms the fix is working.

**Pass Criteria:**
- [ ] Ringing score is non-zero for maneuvers with visible oscillation (confirms Schmitt trigger fix)
- [ ] Assessment correlates with visible ringing in gyro trace
- [ ] First overshoot is NOT counted in the score (verify by checking that a single overshoot with clean settling gives a low score)
- [ ] Deadband filtering works (noise-level error oscillations not counted as zero-crossings)
- [ ] Different maneuver intensities produce varying scores (not binary 0/nonzero)

---

## Test 4: P-Term Adjustment (Phase 2a)

**Goal:** Verify P-term decreases in response to ringing.

**Procedure:**
1. Start with known ringing condition (e.g., P=33 on pitch which showed ringing in blackbox analysis)
2. Activate autotune, wait for Phase 2 to become active
3. Perform 5-8 aggressive maneuvers
4. Monitor `debug[5]` (P Adjustment):
   - Should step negative (-2 per maneuver if ringing detected)
   - Should stop decreasing when ringing assessment improves
5. Deactivate autotune
6. Check CLI: `get ff_autotune_p_adj_pitch` should be negative

**Pass Criteria:**
- [ ] P adjustment (`debug[5]`) decreases (becomes more negative) when ringing detected
- [ ] P adjustment stabilizes when well-damped assessment reached
- [ ] Ringing score (`debug[4]`) decreases as P is reduced
- [ ] No adjustment made when assessment is WELL_DAMPED from the start
- [ ] Adjustment magnitude does not exceed `p_adjust_max`

---

## Test 5: P Bracket Convergence

**Goal:** Verify bracketing narrows to find optimal P.

**Procedure:**
1. With Phase 2 active, perform enough maneuvers (8-12) for bracket to form
2. Monitor `debug[5]` (P Adjustment) pattern:
   - Should show initial step-down (searching phase)
   - Should switch to binary search once bracket established
   - Should converge when bracket width <= p_step
3. After convergence, `debug[3]` should transition to 2 (Phase 2b Scale-Down)

**Pass Criteria:**
- [ ] P adjustment shows searching then narrowing pattern
- [ ] Bracket width decreases over successive maneuvers
- [ ] Phase transitions to Phase 2b (`debug[3]` = 2) after P convergence
- [ ] Final P adjustment is between the last ringing and last well-damped values

---

## Test 6: Phase 2b D-Term Fallback (within Phase 2a)

**Goal:** Verify D-term increases when P reduction alone is insufficient.

**Setup:** This test requires a craft where even maximum P reduction doesn't suppress ringing. To simulate:
```
# Set a small P reduction limit
set ff_autotune_p_adjust_max = 2
save
```

**Procedure:**
1. Activate autotune with Phase 2a active (`debug[3]` = 1)
2. Perform maneuvers until P hits the -2 limit
3. Monitor `debug[6]` (D Adjustment):
   - Should start increasing after P limit reached
   - Should step by `d_step` (default 1) per maneuver
4. Continue until ringing is suppressed or D limit reached

**Pass Criteria:**
- [ ] D adjustment (`debug[6]`) increases after P hits limit
- [ ] D adjustment does not exceed `d_adjust_max`
- [ ] Phase transitions to Phase 2b (`debug[3]` = 2) after convergence
- [ ] No audible motor noise increase (listen carefully)

**Cleanup:**
```
set ff_autotune_p_adjust_max = 10
save
```

---

## Test 6b: Phase 2b Noise Scale-Down

**Goal:** Verify Phase 2b measures D-term noise during RISING and applies uniform P+D scale-down.

**Procedure:**
1. Wait for Phase 2a to converge (`debug[3]` transitions from 1 to 2)
2. First maneuver in Phase 2b: check `debug[6]` (Noise Baseline) — should be non-zero
3. After 2nd maneuver: check `debug[5]` (Scale Adjustment) — should be -1
4. Check `debug[4]` (Noise Score) — should be lower than baseline if noise improved
5. Continue until Phase 2b converges or `scale_max` reached
6. `debug[3]` should transition to 3 (Phase 3 Recheck)

**Pass Criteria:**
- [ ] Baseline noise score is measured on first Phase 2b maneuver
- [ ] Scale adjustment steps down (-1, -2, ...) per maneuver
- [ ] Noise score decreases as scale-down is applied
- [ ] Phase converges when improvement drops below `noise_threshold`% or `scale_max` hit
- [ ] If noise *increases* after a step, the last step is reverted
- [ ] `debug[3]` transitions to 3 after convergence
- [ ] CLI shows: `get ff_autotune_scale_adj_pitch` is negative (or zero if noise was already minimal)

---

## Test 7: Phase 3 F-Term Spot Check

**Goal:** Verify F-term revalidation after P/D changes.

**Procedure:**
1. Wait for Phase 2b to converge (`debug[3]` = 3)
2. Perform 3 more maneuvers (the recheck count)
3. Watch `debug[3]`:
   - Should remain at 3 during recheck
   - Should transition to 4 (COMPLETE) if F-term still valid
   - Should transition back to 0 (Phase 1) if F-term drifted

**Pass Criteria:**
- [ ] Phase 3 collects exactly 3 maneuvers before deciding
- [ ] If F-term is still valid: transitions to COMPLETE (`debug[3]` = 4)
- [ ] If F-term drifted: transitions back to Phase 1 (`debug[3]` = 0) with narrow bracket
- [ ] Re-convergence from Phase 1 is fast (1-3 maneuvers)

---

## Test 8: EEPROM Persistence

**Goal:** Verify P/D adjustments persist across power cycles.

**Procedure:**
1. Complete a Phase 2 tuning session (convergence reached)
2. Deactivate autotune (beeper should sound)
3. Record values:
   ```
   get ff_autotune_p_adj_roll
   get ff_autotune_p_adj_pitch
   get ff_autotune_d_adj_roll
   get ff_autotune_d_adj_pitch
   get ff_autotune_scale_adj_roll
   get ff_autotune_scale_adj_pitch
   get ff_autotune_gain_roll
   get ff_autotune_gain_pitch
   ```
4. Power cycle the craft
5. Re-read the same values
6. Verify they match

**Pass Criteria:**
- [ ] All 6 values persist across power cycle
- [ ] P adjustments are negative (or zero if no ringing)
- [ ] D adjustments are zero or positive
- [ ] Scale adjustments are negative (or zero if no noise)
- [ ] F-term gains match pre-power-cycle values

---

## Test 9: P/D Override Effect on Flight

**Goal:** Verify the P/D adjustments actually affect PID output.

**Procedure:**
1. Set known P/D adjustments manually:
   ```
   set ff_autotune_p_adj_pitch = -5
   set ff_autotune_d_adj_pitch = 2
   save
   ```
2. Set `debug_mode = PIDLOOP` (or log PID terms in blackbox)
3. Fly with autotune switch ON
4. Compare P-term and D-term magnitudes against a baseline flight with adjustments at 0

**Pass Criteria:**
- [ ] P-term magnitude visibly reduced on pitch axis
- [ ] D-term magnitude visibly increased on pitch axis
- [ ] Roll axis P/D unchanged (adjustments are 0)
- [ ] Yaw axis unaffected
- [ ] No P/D changes when autotune switch is OFF

---

## Test 10: Full Sequence End-to-End

**Goal:** Complete Phase 1 + Phase 2 + Phase 3 in a single flight.

**Procedure:**
1. Reset all autotune state:
   ```
   set ff_autotune_gain_roll = 0
   set ff_autotune_gain_pitch = 0
   set ff_autotune_p_adj_roll = 0
   set ff_autotune_p_adj_pitch = 0
   set ff_autotune_d_adj_roll = 0
   set ff_autotune_d_adj_pitch = 0
   set ff_autotune_scale_adj_roll = 0
   set ff_autotune_scale_adj_pitch = 0
   save
   ```
2. Set debug mode: `set debug_mode = FF_AUTOTUNE`
3. Arm and activate autotune
4. Perform 15-25 aggressive maneuvers (mix of roll and pitch)
5. Monitor `debug[3]` progression: 0 -> 1 -> 2 -> 3 -> 4
6. Deactivate autotune when COMPLETE (4) shown
7. Review all learned values

**Pass Criteria:**
- [ ] Phase 1 converges (F-term gain found) per axis
- [ ] Phase 2a activates automatically after Phase 1 convergence
- [ ] Ringing analysis runs and P adjustment progresses (non-zero scores confirm Schmitt trigger)
- [ ] Phase 2b activates after Phase 2a convergence, measures and reduces noise
- [ ] Phase 3 validates F-term after P/D + scale changes
- [ ] Final state is COMPLETE (`debug[3]` = 4) on both axes
- [ ] All values saved to EEPROM on deactivation
- [ ] Total maneuver count reasonable (15-30 for full sequence)

---

## Performance Validation

After all tests pass, validate the success criteria from the PRD:

| Criterion | Target | How to Measure |
|-----------|--------|----------------|
| Ringing reduction | 50%+ decrease in zero-crossings | Compare blackbox before/after Phase 2 |
| Response preservation | First overshoot drops no more than 30% | Measure peak overshoot in blackbox |
| Convergence speed | 5-8 maneuvers per axis for Phase 2 | Count maneuvers from Phase 2 start to convergence |
| Noise neutrality | No increase in motor noise or D-term noise floor | Listen + blackbox D-term RMS comparison |
| Repeatability | P/D within +/-1 unit across flights | Run full sequence twice, compare results |

---

## Troubleshooting Quick Reference

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Phase never reaches 1 (Phase 2a) | Phase 1 not converging | Check F-term bracketing: `debug[5]` (bracket state) when `debug[3]`=0 |
| Ringing score (`debug[4]`) always 0 | Schmitt trigger bug (should be fixed) or deadband too high | Verify firmware has Schmitt trigger fix; decrease `ring_deadband` |
| Ringing score always high | Threshold too low or mechanical issue | Increase `ring_threshold` |
| P decreases too much | `p_adjust_max` too high or threshold too sensitive | Decrease `p_adjust_max` or increase `ring_threshold` |
| Response feels sluggish after tuning | P reduced too much + scale-down | Decrease `p_adjust_max` and `scale_max`; reset learned values to 0 |
| Motor noise after tuning | D increased in 2a but not scaled enough in 2b | Increase `scale_max` or decrease `noise_threshold` |
| Phase 3 keeps looping back to Phase 1 | F-term coupling with P/D+scale changes | May need wider `error_deadband` |
| Window never opens | Not reaching Phase 2a or maneuvers too small | Ensure Phase 1 converged; use larger stick deflections |
| Phase 2b finishes instantly | Noise already below threshold | Normal — quad is clean, no scale-down needed |

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2026-02-07 | 1.0 | Initial Phase 2 test plan |
| 2026-02-08 | 2.0 | Updated: `FF_AUTOTUNE` debug mode (removed `FF_AUTOTUNE_PD`), fixed all debug channel references, added Phase 2b noise scale-down test (Test 6b), added Schmitt trigger verification in Test 3, updated phase transition values (0-4 instead of 0-3), added scale adjustment CLI params |
