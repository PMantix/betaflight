# Phase 2 Test Plan: P/D Ratio Tuning

> **Date:** 2026-02-07
> **Prerequisite:** Phase 1 F-term tuning verified working
> **Firmware:** ff-autotune-v1 branch with Phase 2 implementation

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

# Verify Phase 2 defaults loaded
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

# Verify learned adjustments start at zero
get ff_autotune_p_adj_roll
get ff_autotune_p_adj_pitch
get ff_autotune_d_adj_roll
get ff_autotune_d_adj_pitch
# All should be 0

# Set debug mode for Phase 2 blackbox logging
set debug_mode = FF_AUTOTUNE_PD
save
```

### 3. Blackbox Setup

- [ ] Blackbox logging enabled
- [ ] SD card has free space
- [ ] Debug mode set to `FF_AUTOTUNE_PD`
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
4. Watch `debug[6]` (Autotune Phase):
   - Should start at 0 (Phase 1)
   - Should transition to 1 (Phase 2) after F-term converges
5. Deactivate autotune switch
6. Verify beeper sounds (save confirmation)
7. Check CLI: `get ff_autotune_gain_pitch` should have a non-zero learned value

**Pass Criteria:**
- [ ] Phase transitions from 0 to 1 visible in blackbox
- [ ] Phase does not transition to 2 until Phase 1 bracket is converged
- [ ] Beeper sounds on deactivation

---

## Test 2: Ringing Window Detection

**Goal:** Verify the ringing analysis window opens and closes correctly.

**Procedure:**
1. Ensure Phase 2 is active (Phase 1 must have converged, or manually set a converged state)
2. With `debug_mode = FF_AUTOTUNE_PD`, perform a maneuver:
   - Quick pitch forward, hold for ~500ms, release
3. In blackbox, check `debug[5]` (Window Active):
   - Should transition 0 -> 1 when stick reaches peak deflection
   - Should return to 0 after ~150ms (or when stick returns to center)
4. Check `debug[1]` (Zero Crossings) - should show a count when window closes
5. Check `debug[2]` (Ring Amplitude) - should show peak-to-peak value

**Pass Criteria:**
- [ ] `debug[5]` shows window opening at end of RISING state
- [ ] Window duration approximately matches `ring_window_ms` setting
- [ ] Window closes before next maneuver starts
- [ ] Zero-crossing count and amplitude reflect visible ringing in gyro trace

---

## Test 3: Ringing Score and Assessment

**Goal:** Verify ringing detection produces reasonable scores.

**Procedure:**
1. With Phase 2 active, perform 5-6 aggressive pitch maneuvers
2. After each maneuver, check:
   - `debug[0]` (Ringing Score) - should be non-zero after each window
   - `debug[7]` (Ring Assessment):
     - 0 = WELL_DAMPED (score below threshold/2)
     - 1 = MILD (score between threshold/2 and threshold)
     - 2 = RINGING (score above threshold)
3. Cross-reference: look at the raw gyro and setpoint traces during the plateau
   - Visible oscillation should correspond to higher ringing scores
   - Smooth settling should correspond to lower scores

**Pass Criteria:**
- [ ] Ringing score varies meaningfully across maneuvers
- [ ] Assessment correlates with visible ringing in gyro trace
- [ ] First overshoot is NOT counted in the score (verify by checking that a single overshoot with clean settling gives a low score)
- [ ] Deadband filtering works (noise-level error oscillations not counted as zero-crossings)

---

## Test 4: P-Term Adjustment (Phase 2a)

**Goal:** Verify P-term decreases in response to ringing.

**Procedure:**
1. Start with known ringing condition (e.g., P=33 on pitch which showed ringing in blackbox analysis)
2. Activate autotune, wait for Phase 2 to become active
3. Perform 5-8 aggressive maneuvers
4. Monitor `debug[3]` (P Adjustment):
   - Should step negative (-2 per maneuver if ringing detected)
   - Should stop decreasing when ringing assessment improves
5. Deactivate autotune
6. Check CLI: `get ff_autotune_p_adj_pitch` should be negative

**Pass Criteria:**
- [ ] P adjustment decreases (becomes more negative) when ringing detected
- [ ] P adjustment stabilizes when well-damped assessment reached
- [ ] Ringing score decreases as P is reduced
- [ ] No adjustment made when assessment is WELL_DAMPED from the start
- [ ] Adjustment magnitude does not exceed `p_adjust_max`

---

## Test 5: P Bracket Convergence

**Goal:** Verify bracketing narrows to find optimal P.

**Procedure:**
1. With Phase 2 active, perform enough maneuvers (8-12) for bracket to form
2. Monitor `debug[3]` (P Adjustment) pattern:
   - Should show initial step-down (searching phase)
   - Should switch to binary search once bracket established
   - Should converge when bracket width <= p_step
3. After convergence, `debug[6]` should transition to 2 (Phase 3 recheck)

**Pass Criteria:**
- [ ] P adjustment shows searching then narrowing pattern
- [ ] Bracket width decreases over successive maneuvers
- [ ] Phase transitions to Phase 3 (value 2) after P convergence
- [ ] Final P adjustment is between the last ringing and last well-damped values

---

## Test 6: Phase 2b D-Term Fallback

**Goal:** Verify D-term increases when P reduction alone is insufficient.

**Setup:** This test requires a craft where even maximum P reduction doesn't suppress ringing. To simulate:
```
# Set a small P reduction limit
set ff_autotune_p_adjust_max = 2
save
```

**Procedure:**
1. Activate autotune with Phase 2 active
2. Perform maneuvers until P hits the -2 limit
3. Monitor `debug[4]` (D Adjustment):
   - Should start increasing after P limit reached
   - Should step by `d_step` (default 1) per maneuver
4. Continue until ringing is suppressed or D limit reached

**Pass Criteria:**
- [ ] D adjustment increases after P hits limit
- [ ] D adjustment does not exceed `d_adjust_max`
- [ ] Phase transitions to Phase 3 after D convergence
- [ ] No audible motor noise increase (listen carefully)

**Cleanup:**
```
set ff_autotune_p_adjust_max = 10
save
```

---

## Test 7: Phase 3 F-Term Spot Check

**Goal:** Verify F-term revalidation after P/D changes.

**Procedure:**
1. Wait for Phase 2 to converge (debug[6] = 2)
2. Perform 3 more maneuvers (the recheck count)
3. Watch `debug[6]`:
   - Should remain at 2 during recheck
   - Should transition to 3 (COMPLETE) if F-term still valid
   - Should transition back to 0 (Phase 1) if F-term drifted

**Pass Criteria:**
- [ ] Phase 3 collects exactly 3 maneuvers before deciding
- [ ] If F-term is still valid: transitions to COMPLETE (3)
- [ ] If F-term drifted: transitions back to Phase 1 (0) with narrow bracket
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
   save
   ```
2. Set debug mode: `set debug_mode = FF_AUTOTUNE_PD`
3. Arm and activate autotune
4. Perform 15-25 aggressive maneuvers (mix of roll and pitch)
5. Monitor `debug[6]` progression: 0 -> 1 -> 2 -> 3
6. Deactivate autotune when COMPLETE (3) shown
7. Review all learned values

**Pass Criteria:**
- [ ] Phase 1 converges (F-term gain found) per axis
- [ ] Phase 2 activates automatically after Phase 1 convergence
- [ ] Ringing analysis runs and P adjustment progresses
- [ ] Phase 3 validates F-term after P/D changes
- [ ] Final state is COMPLETE (3) on both axes
- [ ] All values saved to EEPROM on deactivation
- [ ] Total maneuver count reasonable (15-25 for full sequence)

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
| Phase never reaches 1 (Phase 2) | Phase 1 not converging | Check F-term bracketing in `FF_AUTOTUNE` debug mode |
| Ringing score always 0 | Window too short or deadband too high | Increase `ring_window_ms`, decrease `ring_deadband` |
| Ringing score always high | Threshold too low | Increase `ring_threshold` |
| P decreases too much | `p_adjust_max` too high or threshold too sensitive | Decrease `p_adjust_max` or increase `ring_threshold` |
| Response feels sluggish after tuning | P reduced too much | Decrease `p_adjust_max` or increase `p_step` for faster convergence |
| Motor noise after tuning | D increase too aggressive | Decrease `d_adjust_max` or `d_step` |
| Phase 3 keeps looping back to Phase 1 | F-term coupling with P/D changes | May need wider `error_deadband` |
| Window never opens | Not reaching Phase 2 or maneuvers too small | Ensure Phase 1 converged; use larger stick deflections |

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2026-02-07 | 1.0 | Initial Phase 2 test plan |
