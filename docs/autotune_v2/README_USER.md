# Autotune V2 User Guide

## Quick Start

### Setup (CLI)
```
set autotune_enabled = 1
set autotune_axes = 7
set autotune_aggressiveness = 50
save
```

### Configure Mode Switch
In Betaflight Configurator, assign AUTOTUNE to an AUX channel:
1. Go to Modes tab
2. Add a range for AUTOTUNE mode
3. Assign it to an AUX switch

### Flying

1. **Take off and hover** at mid-throttle in calm conditions
2. **Flip autotune switch ON**
3. **Wait for hover lock** (hold steady for 1 second)
4. **Perform stick movements:**
   - Roll left, hold briefly, return to center
   - Roll right, hold briefly, return to center
   - Repeat for pitch and yaw

5. **Feedback patterns:**
   - 1 wiggle = axis complete
   - 4 wiggles = all done!

6. **Flip switch OFF** when complete (or to abort)

---

## Settings Reference

| Setting | Default | Range | Description |
|---------|---------|-------|-------------|
| `autotune_enabled` | 0 | 0-1 | Enable autotune feature |
| `autotune_axes` | 7 | 0-7 | Bitmask: 1=Roll, 2=Pitch, 4=Yaw |
| `autotune_aggressiveness` | 50 | 0-100 | Higher = faster response, more overshoot |
| `autotune_tune_ff` | 1 | 0-1 | Enable feedforward tuning |
| `autotune_p_step` | 10 | 5-30 | Base P step size (%) |
| `autotune_d_step` | 10 | 5-30 | Base D step size (%) |
| `autotune_f_step` | 5 | 5-50 | Base F step size (%) |
| `autotune_stick_threshold` | 15 | 5-50 | Min stick deflection (degrees) |
| `autotune_cross_axis_max` | 10 | 5-30 | Max cross-axis movement (degrees) |
| `autotune_max_events` | 10 | 3-20 | Max events per axis |
| `autotune_overshoot_low` | 5 | 1-20 | Lower overshoot target (%) |
| `autotune_overshoot_high` | 10 | 5-30 | Upper overshoot target (%) |

### Axis Bitmask
- 1 = Roll only
- 2 = Pitch only
- 4 = Yaw only
- 3 = Roll + Pitch
- 7 = All axes (default)

---

## Troubleshooting

### Nothing happens when I flip the switch
- Check `autotune_enabled` is ON (`set autotune_enabled = 1`)
- Verify AUX channel is correctly assigned in Modes tab
- Must be armed and flying
- Run `autotune` in CLI to check status

### Tuning aborts immediately
- Hover more steadily (stick movement detected)
- Reduce `autotune_stick_threshold` if too sensitive
- Ensure throttle is stable (no rapid changes)

### Oscillation during tuning
- This is normal - autotune will detect and rollback
- If persistent, reduce `autotune_aggressiveness`
- After 3 consecutive bad events, autotune will advance anyway

### Takes too long
- Increase `autotune_aggressiveness` for faster convergence
- Reduce number of axes (`autotune_axes = 3` for Roll+Pitch only)
- Total timeout is 3 minutes, per-axis timeout is 60 seconds

### Poor tune results
- Make clean, deliberate stick movements
- Wait for each wiggle before next movement
- Use fresh battery (consistent power)
- Fly in calm conditions (no wind)

---

## CLI Commands

### Check Status
```
autotune
```
Shows current autotune state, axis, progress, and last reason code.

### View All Settings
```
get autotune
```
Displays all autotune-related settings.

### Reset to Defaults
```
defaults nosave
get autotune
```
Shows default values without saving.

---

## Tips for Best Results

1. **Fly in calm conditions** - wind affects tuning quality
2. **Use fresh battery** - consistent power is important
3. **Make clean, deliberate stick movements** - not too fast, not too slow
4. **Wait for each wiggle** before next movement
5. **Start with default aggressiveness** (50) and adjust if needed
6. **Tune Roll/Pitch first** (`autotune_axes = 3`) then Yaw separately if needed

---

## How It Works

Autotune V2 uses a bracket+Newton algorithm to find optimal PID gains:

1. **Hover Lock** - Waits for stable hover
2. **PD Ratio Seek** - Finds optimal P/D ratio by targeting specific overshoot
3. **PD Scale Up** - Scales P and D together to minimize response lag
4. **F Tune** (optional) - Adjusts feedforward for stick tracking

The algorithm measures overshoot and lag from your stick movements, then uses Newton's method to efficiently converge on optimal values.

---

## Safety Features

- **Automatic rollback** on oscillation detection
- **Gain limits** prevent dangerous values (P: 20-200)
- **Timeout protection** - 3 minute total, 60 seconds per axis
- **Repeated failure handling** - forces advance after 3 bad events
- **Original gain preservation** for abort recovery

---

## Debug Information

For developers and advanced users, enable blackbox logging with DEBUG mode set to AUTOTUNE_V2 to record:
- State transitions
- Overshoot measurements
- Gain changes
- Decision codes

---

*Last updated: January 20, 2026*
