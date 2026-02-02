# Autotune V2 Test Flight Checklist

## Pre-Flight Setup

### Firmware
- [ ] Flash `betaflight_2026.6.0-alpha_STM32G47X_BETAFPVG473.hex`
- [ ] Verify autotune CLI settings:
  ```
  get autotune
  ```
- [ ] Set debug mode for autotune:
  ```
  set debug_mode = AUTOTUNE_V2
  ```

### Blackbox Configuration
- [ ] Enable blackbox logging
- [ ] Set appropriate sample rate (recommend 2kHz for detailed analysis)
- [ ] Verify SD card has space
- [ ] Note the log file number before flight

### Debug Channel Reference

**General (all states):**
| Debug Index | Value | Scale | Description |
|-------------|-------|-------|-------------|
| debug[0] | State | 1x | Current autotune state (0-8) |
| debug[1] | Axis | 1x | Current axis (0=Roll, 1=Pitch, 2=Yaw) |
| debug[2] | Reason | 1x | Reason code (0-35, see docs) |
| debug[3] | Decision | varies | Gain change or progress |
| debug[4] | Overshoot | ×10 | Overshoot % (50 = 5.0%) |
| debug[5] | P Gain | 1x | Current P gain value |
| debug[6] | D Gain | 1x | Current D gain value |
| debug[7] | F Gain | 1x | Current F gain value |

**During THROTTLE_SWEEP (state=2):**
| Debug Index | Value | Scale | Description |
|-------------|-------|-------|-------------|
| debug[3] | Sweep Progress | 1x | sweepCount×100 + hover progress |
| debug[4] | **Peak Freq** | Hz | Dynamic notch primary peak frequency |
| debug[5] | **Throttle Range** | % | Throttle range covered (0-100) |

---

## Test Flight #1: Overshoot Validation

### Objective
Validate that the overshoot calculation fix produces accurate values matching visual observation.

### Maneuvers
1. **Hover stabilization** (10 seconds)
   - Establishes baseline
   
2. **Roll flicks** (5-10 repetitions)
   - Sharp stick input to ~80% deflection
   - Hold briefly at max
   - Return to center
   - Pause 1-2 seconds between flicks
   
3. **Pitch flicks** (5-10 repetitions)
   - Same pattern as roll
   
4. **Yaw snaps** (5-10 repetitions)
   - Quick yaw inputs

### Expected Results
| Metric | Expected Range | Previous (Broken) | Notes |
|--------|----------------|-------------------|-------|
| Overshoot % | 5-15% | 24-47% (×10 debug) | Should match visual |
| debug[1] | 50-150 | 240-470 | ×10 scaled |

### Analysis Steps
1. Download blackbox log
2. Open in Blackbox Explorer or analyze with Python script
3. Look at `debug[1]` values during EVENT_DETECTED state
4. Compare to actual gyro vs setpoint overshoot in plots
5. Values should now correlate with visual observation

---

## Test Flight #2: Filter Characterization

### Objective
Verify filter characterization is detecting peak frequencies during throttle sweep.

### Maneuvers
1. **Arm and hover** at mid-throttle
2. **Activate autotune** (aux switch)
3. **Throttle sweep** - slowly move throttle from hover to ~80% and back
4. **Monitor state** - should see state transitions 0→1→2

### Expected Results
| Metric | Expected | Current Issue |
|--------|----------|---------------|
| debug[2] | 100-400 Hz | Shows 0 |
| Peak frequency | Non-zero | Not being logged |

### Debug Focus
- If `debug[2]` remains 0, the `readDynNotchPeaks()` function may not be working
- Check if dynamic notch filter is enabled
- Verify `dynNotchPeakHz` is populated

---

## Test Flight #3: Full State Progression

### Objective
Verify complete state machine progression through all phases.

### State Reference
| State | Name | Trigger |
|-------|------|---------|
| 0 | IDLE | Autotune inactive |
| 1 | THROTTLE_SWEEP | Autotune activated |
| 2 | WAIT_FOR_EVENT | Throttle sweep complete |
| 3 | EVENT_DETECTED | Flick detected |
| 4 | COMPUTE_METRICS | Event captured |
| 5 | PD_RATIO_SEEK | Adjusting P/D ratio |
| 6 | PD_SCALE_UP | Scaling gains |
| 7 | POLISH | Final tuning |
| 8 | COMPLETE | Tuning finished |

### Maneuvers
1. Activate autotune
2. Perform throttle sweep
3. Execute multiple flicks
4. Monitor state progression
5. Note any states that don't transition

### Known Issues to Watch
- [ ] PD_SCALE_UP not changing gains (pending investigation)
- [ ] Filter peak frequency showing 0

---

## Post-Flight Analysis

### Blackbox Log Review
1. **State transitions**: Plot `debug[0]` - verify progression
2. **Overshoot values**: Plot `debug[1]` - verify reasonable range
3. **Filter frequencies**: Plot `debug[2]` - check for non-zero during throttle sweep
4. **Gain changes**: Plot `debug[3]` - verify gains are being adjusted

### Python Analysis Script
```bash
cd "bb logs"
python analyze_quick.py <log_file.csv>
```

### Validation Criteria
- [ ] Overshoot values correlate with visual observation
- [ ] State machine progresses through expected states
- [ ] No unexpected resets or stuck states
- [ ] Filter characterization populates frequency data

---

## Issue Reporting

If issues are found, document:
1. Log file name
2. Timestamp of issue
3. Expected vs actual behavior
4. Relevant debug values
5. Screenshots of Blackbox Explorer plots

Update [ISSUE_TRACKER.md](ISSUE_TRACKER.md) with findings.
