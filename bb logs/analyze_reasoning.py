#!/usr/bin/env python3
"""
Analyze autotune reason codes from blackbox logs.

Debug channels:
[0] = State (0=IDLE, 1=ARMED, 2=DETECTING, 3=COLLECTING, 4=SETTLING, 5=ANALYZING, 6=ADJUSTING, 7=SIGNALING)
[1] = iteration * 10 + mode (mode: 1=ROLL, 2=PITCH, 3=FILTER, 4=HOVER_TUNE)
[2] = varies by mode (dterm_lpf1 for hover/filter, P for PID)
[3] = varies by mode (D for hover, dterm_lpf2/D for filter/PID)
[4] = varies by mode
[5] = varies by mode
[6] = varies by mode (motor RMS for hover, noise floor for filter, overshoot for PID)
[7] = reason code
"""

import pandas as pd
import sys

STATE_NAMES = {
    0: "IDLE",
    1: "ARMED", 
    2: "DETECTING",
    3: "COLLECTING",
    4: "SETTLING",
    5: "ANALYZING",
    6: "ADJUSTING",
    7: "SIGNALING",
    8: "COMPLETE",
    9: "ABORTED"
}

MODE_NAMES = {
    0: "NONE",
    1: "ROLL",
    2: "PITCH", 
    3: "FILTER",
    4: "HOVER_TUNE"
}

# Reason code definitions
REASON_CODES = {
    # Hover tune (1xxx)
    1000: "HOVER_WAITING - Waiting for stable hover",
    1001: "HOVER_MEASURING - Measuring motor RMS",
    1110: "HOVER_RMS_HIGH_D_DOWN - Motor RMS high, reducing D",
    1120: "HOVER_RMS_HIGH_LPF_DOWN - Motor RMS high, lowering dterm LPF",
    1210: "HOVER_RMS_LOW_LPF_UP - Motor RMS low, raising dterm LPF",
    1220: "HOVER_RMS_LOW_D_UP - Motor RMS low, raising D",
    1300: "HOVER_STABLE - Motor RMS stable, converged",
    1999: "HOVER_COMPLETE - Hover tune complete",
    
    # Filter tune (2xxx)
    2000: "FILTER_WAITING - Waiting for throttle pump",
    2001: "FILTER_COLLECTING - Collecting noise data",
    2110: "FILTER_RESONANCE_LPF - Resonance detected, lowering LPF",
    2120: "FILTER_RESONANCE_NOTCH - Resonance detected, adding notch",
    2210: "FILTER_NOISE_HIGH_LPF - Noise high, lowering LPF",
    2310: "FILTER_NOISE_LOW_LPF - Noise low, raising LPF",
    2400: "FILTER_NOISE_OK - Noise in acceptable range",
    2500: "FILTER_NO_CHANGE - No change needed",
    2999: "FILTER_COMPLETE - Filter tune complete",
    
    # PID tune (3xxx)
    3000: "PID_WAITING - Waiting for maneuver",
    3001: "PID_COLLECTING - Collecting response data",
    3110: "PID_OVERSHOOT_P_DOWN - Overshoot high, reducing P",
    3120: "PID_OVERSHOOT_D_UP - Overshoot high, raising D",
    3210: "PID_SLUGGISH_P_UP - Response slow, raising P",
    3220: "PID_SLUGGISH_D_DOWN - Response slow, reducing D",
    3310: "PID_OSCILLATION_D_UP - Oscillation detected, raising D",
    3320: "PID_OSCILLATION_P_DOWN - Oscillation detected, reducing P",
    3410: "PID_NOISE_D_DOWN - Noise high, reducing D",
    3420: "PID_DRIFT_I_UP - Drift detected, raising I",
    3430: "PID_BOUNCEBACK_I_DOWN - Bounceback detected, lowering I",
    3440: "PID_SLOW_OSC_I_DOWN - Slow oscillation, lowering I",
    3500: "PID_RESPONSE_GOOD - Response good, no change",
    3510: "PID_LAG_F_UP - Stick lag, raising F",
    3520: "PID_LEAD_F_DOWN - Gyro leading stick, lowering F",
    3530: "PID_PHASE_F_UP - Large phase lag, raising F",
    3540: "PID_VELOCITY_LAG_F_UP - High velocity stick lag, raising F",
    3999: "PID_COMPLETE - PID tune complete",
    
    # System (9xxx)
    9000: "IDLE - Autotune idle",
    9100: "GRACE_PERIOD - In grace period",
    9200: "DATA_INVALID - Data invalid, skipping",
    9300: "AT_LIMIT - Value at min/max limit",
}

def get_reason_name(code):
    return REASON_CODES.get(int(code), f"UNKNOWN_{int(code)}")

def analyze_reasoning_log(filename):
    print(f"\n{'='*60}")
    print(f"Autotune Reason Code Analysis - {filename}")
    print(f"{'='*60}\n")
    
    # Read CSV, skip header metadata (first 147 rows)
    df = pd.read_csv(filename, skiprows=147)
    
    print(f"Total samples: {len(df)}")
    duration_sec = (df['time'].iloc[-1] - df['time'].iloc[0]) / 1e6
    print(f"Duration: {duration_sec:.2f} seconds\n")
    
    # Parse mode and iteration from debug[1]
    debug1 = df['debug[1]'].astype(int)
    df['mode'] = debug1 % 10
    df['iteration'] = debug1 // 10
    
    # === State Distribution ===
    print("=== State Distribution ===")
    state_counts = df['debug[0]'].value_counts().sort_index()
    for state, count in state_counts.items():
        pct = count / len(df) * 100
        name = STATE_NAMES.get(int(state), "???")
        print(f"  {int(state)} ({name:10}): {count:6} samples ({pct:5.1f}%)")
    print()
    
    # === Mode Distribution ===
    print("=== Mode Distribution ===")
    mode_counts = df['mode'].value_counts().sort_index()
    for mode, count in mode_counts.items():
        pct = count / len(df) * 100
        name = MODE_NAMES.get(int(mode), "???")
        print(f"  {int(mode)} ({name:10}): {count:6} samples ({pct:5.1f}%)")
    print()
    
    # === Reason Code Distribution ===
    print("=== Reason Code Distribution ===")
    reason_counts = df['debug[7]'].value_counts().sort_index()
    for reason, count in reason_counts.items():
        pct = count / len(df) * 100
        name = get_reason_name(reason)
        print(f"  {int(reason):4}: {count:6} ({pct:5.1f}%) - {name}")
    print()
    
    # === Reason Code Changes (Timeline) ===
    print("=== Reason Code Changes (Timeline) ===")
    prev_reason = None
    prev_state = None
    changes = []
    
    for idx, row in df.iterrows():
        reason = int(row['debug[7]'])
        state = int(row['debug[0]'])
        if reason != prev_reason or state != prev_state:
            time_ms = row['time'] / 1000
            mode = int(row['mode'])
            iteration = int(row['iteration'])
            changes.append({
                'time_ms': time_ms,
                'state': state,
                'state_name': STATE_NAMES.get(state, "???"),
                'mode': mode,
                'mode_name': MODE_NAMES.get(mode, "???"),
                'iteration': iteration,
                'reason': reason,
                'reason_name': get_reason_name(reason),
                'd2': row['debug[2]'],
                'd3': row['debug[3]'],
                'd5': row['debug[5]'],
                'd6': row['debug[6]'],
            })
            prev_reason = reason
            prev_state = state
    
    print(f"Total reason/state changes: {len(changes)}\n")
    
    # Print all changes with context
    for i, c in enumerate(changes):
        time_sec = c['time_ms'] / 1000
        
        # Mode-specific context
        if c['mode'] == 4:  # Hover tune
            context = f"dterm_lpf={c['d2']}Hz D={c['d3']} bestRMS={c['d5']/10:.1f} currRMS={c['d6']/10:.1f}"
        elif c['mode'] == 3:  # Filter
            context = f"dterm_lpf1={c['d2']}Hz dterm_lpf2={c['d3']}Hz noise={c['d6']/10:.1f}"
        elif c['mode'] in [1, 2]:  # Roll/Pitch
            context = f"P={c['d2']} D={c['d3']} overshoot={c['d6']/10:.1f}%"
        else:
            context = ""
        
        print(f"[{time_sec:6.2f}s] iter={c['iteration']} {c['state_name']:10} {c['mode_name']:10} | {c['reason']:4} {c['reason_name']}")
        if context and c['state'] >= 2:  # Only show context for active states
            print(f"          {context}")
    
    print()
    
    # === Hover Tune Analysis ===
    hover_mode = df[df['mode'] == 4]
    if len(hover_mode) > 0:
        print("=== Hover Tune Summary ===")
        print(f"Samples in hover tune: {len(hover_mode)}")
        
        # Track dterm_lpf1 and D changes
        first = hover_mode.iloc[0]
        last = hover_mode.iloc[-1]
        print(f"  dterm_lpf1: {int(first['debug[2]'])} -> {int(last['debug[2]'])} Hz")
        print(f"  D gain:     {int(first['debug[3]'])} -> {int(last['debug[3]'])}")
        print(f"  Best RMS:   {last['debug[5]']/10:.1f}")
        print(f"  Final RMS:  {last['debug[6]']/10:.1f}")
        print()
    
    # === Filter Tune Analysis ===
    filter_mode = df[df['mode'] == 3]
    if len(filter_mode) > 0:
        print("=== Filter Tune Summary ===")
        print(f"Samples in filter mode: {len(filter_mode)}")
        
        first = filter_mode.iloc[0]
        last = filter_mode.iloc[-1]
        print(f"  dterm_lpf1: {int(first['debug[2]'])} -> {int(last['debug[2]'])} Hz")
        print(f"  dterm_lpf2: {int(first['debug[3]'])} -> {int(last['debug[3]'])} Hz")
        print()
    
    # === PID Tune Analysis ===
    roll_mode = df[df['mode'] == 1]
    pitch_mode = df[df['mode'] == 2]
    
    if len(roll_mode) > 0:
        print("=== Roll Tune Summary ===")
        print(f"Samples in roll mode: {len(roll_mode)}")
        first = roll_mode.iloc[0]
        last = roll_mode.iloc[-1]
        print(f"  P: {int(first['debug[2]'])} -> {int(last['debug[2]'])}")
        print(f"  D: {int(first['debug[3]'])} -> {int(last['debug[3]'])}")
        print()
    
    if len(pitch_mode) > 0:
        print("=== Pitch Tune Summary ===")
        print(f"Samples in pitch mode: {len(pitch_mode)}")
        first = pitch_mode.iloc[0]
        last = pitch_mode.iloc[-1]
        print(f"  P: {int(first['debug[2]'])} -> {int(last['debug[2]'])}")
        print(f"  D: {int(first['debug[3]'])} -> {int(last['debug[3]'])}")
        print()
    
    # === Adjustment Actions Summary ===
    adjusting = df[df['debug[0]'] == 6]  # ADJUSTING state
    if len(adjusting) > 0:
        print("=== Adjustment Actions ===")
        adj_reasons = adjusting['debug[7]'].value_counts().sort_index()
        for reason, count in adj_reasons.items():
            name = get_reason_name(reason)
            print(f"  {int(reason)}: {count:3}x - {name}")
        print()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "btfl_debug_reasoning.bbl.csv"
    
    analyze_reasoning_log(filename)
