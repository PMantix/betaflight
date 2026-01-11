# Autotune Reason Codes

This document describes the reason codes output to `debug[7]` when using `set debug_mode = AUTOTUNE`. These codes explain what autotune is currently doing or what action it last took.

## Code Format

Reason codes follow the format `XYZZ` where:
- **X** = Category (1=Hover, 2=Filter, 3=PID, 9=No action)
- **Y** = Subcategory (action type within category)
- **ZZ** = Specific reason

---

## Category 1xxx: Hover Tune (Phase 0)

Hover tune runs automatically after arming when autotune is enabled. It adjusts D-term LPF and D gain to minimize motor command noise while hovering with sticks centered.

| Code | Name | Description |
|------|------|-------------|
| 1000 | `REASON_HOVER_WAITING` | Waiting for stable hover (calibrating hover throttle reference) |
| 1001 | `REASON_HOVER_MEASURING` | Actively measuring motor RMS during 500ms window |
| 1110 | `REASON_HOVER_RMS_HIGH_D_DOWN` | Motor RMS above target (15), reducing D gain by 5 |
| 1120 | `REASON_HOVER_RMS_HIGH_LPF_DOWN` | Motor RMS above target (15), lowering dterm_lpf1 by 10Hz |
| 1210 | `REASON_HOVER_RMS_LOW_LPF_UP` | Motor RMS excellent (<8), raising dterm_lpf1 by 10Hz to improve response |
| 1220 | `REASON_HOVER_RMS_LOW_D_UP` | Motor RMS excellent, raising D gain (currently unused) |
| 1300 | `REASON_HOVER_STABLE` | Motor RMS stable for 3+ iterations or in acceptable range - converged |
| 1999 | `REASON_HOVER_COMPLETE` | Hover tune complete (hit max iterations or converged) |

**Note:** Category 9xxx codes can also appear during hover tune:
- `9300` (AT_LIMIT): All filters at minimum and D gain at minimum, cannot reduce further

### Hover Tune Decision Logic

```
Motor RMS > 15 (target)?
  ├─ Yes → dterm_lpf1 > 50Hz?
  │         ├─ Yes → Lower LPF by 10Hz (1120)
  │         └─ No → D gain > 15?
  │                  ├─ Yes → Lower D by 5 (1110)
  │                  └─ No → At limit (9300)
  └─ No → Motor RMS < 8 (excellent)?
           ├─ Yes → dterm_lpf1 < 150Hz?
           │         ├─ Yes → Raise LPF by 10Hz (1210)
           │         └─ No → At limit (9300)
           └─ No → In acceptable range, done (1300)
```

---

## Category 2xxx: Filter Tune

Filter tune activates when throttle pumps are detected (sticks centered, throttle > hover+15%). It analyzes noise and resonances to adjust LPF frequencies.

| Code | Name | Description |
|------|------|-------------|
| 2000 | `REASON_FILTER_WAITING` | Waiting for throttle pump to start filter tune |
| 2001 | `REASON_FILTER_COLLECTING` | Collecting noise data during throttle punch |
| 2110 | `REASON_FILTER_RESONANCE_LPF` | Resonance peak detected, lowering gyro_lpf2 by 10Hz |
| 2120 | `REASON_FILTER_RESONANCE_NOTCH` | Resonance detected, adding notch filter (future feature) |
| 2210 | `REASON_FILTER_NOISE_HIGH_LPF` | Noise high (no resonance), lowering dterm_lpf1 by 10Hz |
| 2310 | `REASON_FILTER_NOISE_LOW_LPF` | Noise low, raising gyro_lpf2 by 10Hz to reduce delay |
| 2400 | `REASON_FILTER_NOISE_OK` | Noise in acceptable range, no change needed |
| 2500 | `REASON_FILTER_NO_CHANGE` | No change possible (at limits or already optimal) |
| 2999 | `REASON_FILTER_COMPLETE` | Filter tune complete (converged or max iterations) |

**Note:** Category 9xxx codes can also appear during filter tune:
- `9100` (GRACE_PERIOD): In 500ms grace period after throttle blip, checking if this is a roll/flip instead
- `9300` (AT_LIMIT): All filters already at minimum/maximum limits, cannot adjust further

### Filter Tune Decision Logic

```
Resonance detected (SDFT peak)?
  ├─ Yes → Lower gyro_lpf2 by 10Hz (2110)
  └─ No → Check noise ratio (current/target):
           ├─ Ratio > 1.2 → Noise too high, lower dterm_lpf1 (2210)
           ├─ Ratio < 0.8 → Noise low, raise gyro_lpf2 (2310)
           └─ 0.8-1.2 → Noise OK, no change (2400)
```

---

## Category 3xxx: PID Tune

PID tune activates when roll or pitch maneuvers are detected. It analyzes the response quality and adjusts P, I, D, or F gains.

| Code | Name | Description |
|------|------|-------------|
| 3000 | `REASON_PID_WAITING` | Waiting for roll/flip maneuver |
| 3001 | `REASON_PID_COLLECTING` | Collecting response data during maneuver |
| 3110 | `REASON_PID_OVERSHOOT_P_DOWN` | Overshoot too high, reducing P gain |
| 3120 | `REASON_PID_OVERSHOOT_D_UP` | Overshoot too high, increasing D gain |
| 3210 | `REASON_PID_SLUGGISH_P_UP` | Response too slow, increasing P gain |
| 3220 | `REASON_PID_SLUGGISH_D_DOWN` | Response too slow (overdamped), reducing D gain |
| 3310 | `REASON_PID_OSCILLATION_D_UP` | Oscillation detected, increasing D gain |
| 3320 | `REASON_PID_OSCILLATION_P_DOWN` | Oscillation detected, reducing P gain |
| 3410 | `REASON_PID_NOISE_D_DOWN` | Noise too high, reducing D gain |
| 3420 | `REASON_PID_DRIFT_I_UP` | Drift detected (not holding position), increasing I gain |
| 3430 | `REASON_PID_BOUNCEBACK_I_DOWN` | Bounceback detected (I windup), reducing I gain |
| 3440 | `REASON_PID_SLOW_OSC_I_DOWN` | Slow oscillation (<5Hz), reducing I gain |
| 3500 | `REASON_PID_RESPONSE_GOOD` | Response is good, no change needed |
| 3510 | `REASON_PID_LAG_F_UP` | Gyro lagging stick input, increasing F (feedforward) |
| 3520 | `REASON_PID_LEAD_F_DOWN` | Gyro leading stick input, reducing F (feedforward) |
| 3530 | `REASON_PID_PHASE_F_UP` | Large phase lag with low overshoot, increasing F |
| 3999 | `REASON_PID_COMPLETE` | PID tune complete (converged or max iterations) |

**Note:** Category 9xxx codes can also appear during PID tune:
- `9200` (DATA_INVALID): Response data was invalid (e.g., crash detected), skipping adjustment
- `9300` (AT_LIMIT): Gain already at minimum or maximum limit, cannot adjust further

### I-Term Attribution

The autotuner detects I-term issues by analyzing:

| Issue | Detection | Action |
|-------|-----------|--------|
| **Drift** | Rate drift >20 deg/s² after settling | Increase I (3420) |
| **Bounceback** | Bounceback >30% | Decrease I (3430) |
| **Slow Oscillation** | Oscillation 0.5-5Hz | Decrease I (3440) |

### F-Term (Feedforward) Attribution

The autotuner detects F-term issues by analyzing stick-to-gyro relationship, with special sensitivity during fast stick movements:

| Issue | Detection | Action |
|-------|-----------|--------|
| **Velocity-weighted lag** | `peakStickVelocity > 15` AND `velocityWeightedLag > 5` | Increase F (3510) |
| **Velocity-weighted lead** | `peakStickVelocity > 15` AND `velocityWeightedLag < -3` | Decrease F (3520) |
| **Stick Lag** | `stickLeadError > 30` with fast rise time `<40ms` | Increase F (3510) |
| **Gyro Lead** | Gyro leading stick by `>3ms` (negative phase) | Decrease F (3520) |
| **Phase Lag** | Large phase lag `>10ms` with low overshoot `<10%` | Increase F (3530) |

**Key insight**: F-term errors are most visible during fast stick movements. A 10ms lag during a slow stick input is less important than a 5ms lag during a rapid snap roll initiation. The velocity-weighted detection prioritizes errors during the portions of the maneuver where the stick was moving fastest.

### Response Classifications

| Classification | Meaning | Primary Action |
|----------------|---------|----------------|
| `UNDERDAMPED` | Too much overshoot, oscillating | Reduce P or increase D |
| `OVERDAMPED` | Too slow, sluggish response | Increase P or reduce D |
| `CRITICAL` | Good response, slight overshoot | Minor adjustments or none |
| `NOISY` | Noise limiting further gains | Reduce D |
| `EXCELLENT` | Optimal response achieved | No change |

---

## Category 9xxx: No Action / System States

General system states when autotune is not actively making adjustments.

| Code | Name | Description |
|------|------|-------------|
| 9000 | `REASON_IDLE` | Autotune is idle (switch off or not armed) |
| 9100 | `REASON_GRACE_PERIOD` | In 500ms grace period, checking if throttle blip was for roll/flip |
| 9200 | `REASON_DATA_INVALID` | Collected data was invalid (crash detected), skipping adjustment |
| 9300 | `REASON_AT_LIMIT` | Value already at minimum or maximum limit, cannot adjust further |

---

## Debug Output Layout

When `set debug_mode = AUTOTUNE`:

**Note:** For LPF1 filters, the displayed value is `dyn_min_hz` if dynamic mode is active, otherwise `static_hz`.

### During Hover Tune (mode 4)
| Channel | Value |
|---------|-------|
| debug[0] | State (0-9) |
| debug[1] | iteration×10 + 4 (e.g., 24 = iteration 2, hover mode) |
| debug[2] | dterm_lpf1 frequency (Hz) - dynamic min or static |
| debug[3] | D gain (roll) |
| debug[4] | gyro_lpf1 frequency (Hz) - dynamic min or static |
| debug[5] | Best motor RMS × 10 |
| debug[6] | Current motor RMS × 10 |
| debug[7] | **Reason code** |

### During Filter Tune (mode 3)
| Channel | Value |
|---------|-------|
| debug[0] | State (0-9) |
| debug[1] | iteration×10 + 3 (e.g., 13 = iteration 1, filter mode) |
| debug[2] | dterm_lpf1 frequency (Hz) - dynamic min or static |
| debug[3] | dterm_lpf2 frequency (Hz) |
| debug[4] | gyro_lpf1 frequency (Hz) - dynamic min or static |
| debug[5] | gyro_lpf2 frequency (Hz) |
| debug[6] | Noise floor × 10 |
| debug[7] | **Reason code** |

### During PID Tune (mode 1=roll, 2=pitch)
| Channel | Value |
|---------|-------|
| debug[0] | State (0-9) |
| debug[1] | iteration×10 + mode (e.g., 21 = iteration 2, roll mode) |
| debug[2] | P gain (or throttle % before hover calibrated) |
| debug[3] | D gain (or hover throttle reference in ARMED state) |
| debug[4] | I gain |
| debug[5] | F gain |
| debug[6] | Overshoot % × 10 |
| debug[7] | **Reason code** |

---

## State Values (debug[0])

| Value | State | Description |
|-------|-------|-------------|
| 0 | IDLE | Autotune not active |
| 1 | ARMED | Active, waiting for maneuver or hover tuning |
| 2 | DETECTING | Maneuver in progress |
| 3 | COLLECTING | Collecting post-maneuver settling data |
| 4 | SETTLING | Waiting for stable hover |
| 5 | ANALYZING | Running analysis algorithms |
| 6 | ADJUSTING | Applying gain/filter changes |
| 7 | SIGNALING | Wiggling to signal ready for next maneuver |
| 8 | COMPLETE | Tuning complete |
| 9 | ABORTED | Safety abort triggered |

---

## Example Log Interpretation

```
debug[0]=1, debug[1]=4, debug[7]=1120
→ State=ARMED, mode=hover tune, lowering dterm LPF because motor RMS high

debug[0]=6, debug[1]=23, debug[7]=2110  
→ State=ADJUSTING, iteration 2 filter mode, lowering gyro_lpf2 due to resonance

debug[0]=6, debug[1]=31, debug[7]=3110
→ State=ADJUSTING, iteration 3 roll mode, reducing P due to overshoot
```
