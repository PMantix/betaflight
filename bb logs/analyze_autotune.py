#!/usr/bin/env python3
"""
Autotune Blackbox Log Analyzer
Analyzes autotune state transitions, gain changes, and metrics from blackbox logs.
"""

import pandas as pd
import numpy as np
import sys

# State definitions
STATE_NAMES = ['IDLE', 'ARMED', 'DETECTING', 'COLLECTING', 'SETTLING', 'ANALYZING', 'ADJUSTING', 'SIGNALING', 'COMPLETE', 'ABORTED']

# Mode definitions (matching autotune_types.h)
MODE_NAMES = ['NONE', 'ROLL', 'PITCH', 'FILTER']

# Status code definitions (matching autotune_types.h)
STATUS_CODES = {
    0: 'OK',
    1: 'WAITING_MANEUVER',
    2: 'MANEUVER_DETECTED',
    3: 'COLLECTING_DATA',
    4: 'WAITING_SETTLE',
    5: 'ANALYZING',
    6: 'ADJUSTING_P',
    7: 'ADJUSTING_D', 
    8: 'ADJUSTING_I',
    9: 'ADJUSTING_F',
    10: 'ADJUSTING_FILTER',
    11: 'SIGNAL_READY',
    12: 'COMPLETE',
    13: 'ABORT_ATTITUDE',
    14: 'ABORT_GYRO',
    15: 'ABORT_THROTTLE',
    16: 'ABORT_USER',
}

def find_header_row(filename):
    """Find the row containing column headers."""
    with open(filename, 'r') as f:
        for i, line in enumerate(f):
            if 'loopIteration' in line:
                return i
    return 0

def load_log(filename):
    """Load blackbox log CSV file."""
    header_row = find_header_row(filename)
    df = pd.read_csv(filename, skiprows=header_row)
    return df

def analyze_autotune(filename):
    """Analyze autotune data from blackbox log."""
    print(f"\n{'='*80}")
    print(f"Autotune Analysis: {filename}")
    print(f"{'='*80}\n")
    
    df = load_log(filename)
    print(f"Total samples: {len(df)}")
    
    # Check for debug columns
    debug_cols = [c for c in df.columns if 'debug' in c.lower()]
    if not debug_cols or len(debug_cols) < 5:
        print("ERROR: Debug columns not found. Make sure debug_mode = AUTOTUNE")
        return
    
    # Debug channel mapping:
    # [0] = state
    # [1] = tuneMode | (iteration << 4)  -- mode in lower 4 bits!
    # [2] = P gain
    # [3] = D gain  
    # [4] = status code
    # [5] = overshoot * 10
    # [6] = noise * 10
    # [7] = score * 100
    
    d0 = df['debug[0]'].values  # state
    d1 = df['debug[1]'].values  # mode|iter (mode in lower 4 bits, iter shifted left by 4)
    d2 = df['debug[2]'].values  # P
    d3 = df['debug[3]'].values  # D
    d4 = df['debug[4]'].values  # status
    d5 = df['debug[5]'].values if 'debug[5]' in df.columns else np.zeros(len(df))
    d6 = df['debug[6]'].values if 'debug[6]' in df.columns else np.zeros(len(df))
    d7 = df['debug[7]'].values if 'debug[7]' in df.columns else np.zeros(len(df))
    
    # State distribution
    print("\n=== State Distribution ===")
    for state_val in sorted(set(d0[~np.isnan(d0)].astype(int))):
        count = np.sum(d0 == state_val)
        state_name = STATE_NAMES[state_val] if state_val < len(STATE_NAMES) else f'STATE_{state_val}'
        pct = 100 * count / len(d0)
        print(f"  {state_name:12s}: {count:6d} samples ({pct:5.1f}%)")
    
    # Find state transitions
    print("\n=== State Transitions ===")
    print(f"{'Row':>8s} {'State':>12s} {'Mode':>8s} {'Iter':>4s} {'P':>4s} {'D':>4s} {'Status':>20s} {'Ovr%':>6s} {'Noise':>6s} {'Score':>6s}")
    print("-" * 90)
    
    prev_state = None
    iterations = []
    
    for i in range(len(d0)):
        state = int(d0[i]) if not pd.isna(d0[i]) else -1
        if state != prev_state and state >= 0:
            mode_iter = int(d1[i]) if not pd.isna(d1[i]) else 0
            mode = mode_iter & 0x0F  # Lower 4 bits = mode
            iteration = (mode_iter >> 4) & 0xFF  # Upper bits = iteration
            P = int(d2[i]) if not pd.isna(d2[i]) else 0
            D = int(d3[i]) if not pd.isna(d3[i]) else 0
            status = int(d4[i]) if not pd.isna(d4[i]) else 0
            overshoot = d5[i] / 10.0 if not pd.isna(d5[i]) else 0
            noise = d6[i] / 10.0 if not pd.isna(d6[i]) else 0
            score = d7[i] / 100.0 if not pd.isna(d7[i]) else 0
            
            state_name = STATE_NAMES[state] if state < len(STATE_NAMES) else f'STATE_{state}'
            mode_name = MODE_NAMES[mode] if mode < len(MODE_NAMES) else f'MODE_{mode}'
            status_name = STATUS_CODES.get(status, f'STATUS_{status}')
            
            print(f"{i:8d} {state_name:>12s} {mode_name:>8s} {iteration:4d} {P:4d} {D:4d} {status_name:>20s} {overshoot:6.1f} {noise:6.1f} {score:6.2f}")
            
            # Track iterations at ADJUSTING state
            if state == 6:  # ADJUSTING
                iterations.append({
                    'iteration': iteration,
                    'mode': mode_name,
                    'P': P,
                    'D': D,
                    'status': status_name,
                    'overshoot': overshoot,
                    'noise': noise,
                    'score': score
                })
            
            prev_state = state
    
    # Summary of iterations
    if iterations:
        print("\n=== Iteration Summary ===")
        print(f"{'Iter':>4s} {'Mode':>8s} {'P':>4s} {'D':>4s} {'Adjustment':>20s} {'Overshoot%':>10s} {'Noise':>8s} {'Score':>8s}")
        print("-" * 80)
        for it in iterations:
            print(f"{it['iteration']:4d} {it['mode']:>8s} {it['P']:4d} {it['D']:4d} {it['status']:>20s} {it['overshoot']:10.1f} {it['noise']:8.1f} {it['score']:8.2f}")
        
        # Gain progression
        print("\n=== Gain Progression ===")
        prev_P, prev_D = None, None
        for it in iterations:
            P_change = f" ({it['P'] - prev_P:+d})" if prev_P is not None else ""
            D_change = f" ({it['D'] - prev_D:+d})" if prev_D is not None else ""
            print(f"  Iter {it['iteration']}: P={it['P']}{P_change}, D={it['D']}{D_change}")
            prev_P, prev_D = it['P'], it['D']
    else:
        print("\n=== No completed iterations found ===")
    
    # Check for crashes/bad data
    print("\n=== Data Quality Check ===")
    abort_count = np.sum((d4 >= 20) & (d4 <= 24))
    if abort_count > 0:
        print(f"  WARNING: {abort_count} samples with abort status detected")
        for status_val in range(20, 25):
            count = np.sum(d4 == status_val)
            if count > 0:
                status_name = STATUS_CODES.get(status_val, f'STATUS_{status_val}')
                print(f"    {status_name}: {count} samples")
    else:
        print("  No abort conditions detected")
    
    # Check noise levels
    noise_vals = d6[~np.isnan(d6)] / 10.0
    if len(noise_vals) > 0:
        print(f"\n=== Noise Statistics ===")
        print(f"  Min:  {np.min(noise_vals):6.1f} deg/s")
        print(f"  Max:  {np.max(noise_vals):6.1f} deg/s")
        print(f"  Mean: {np.mean(noise_vals):6.1f} deg/s")
        print(f"  Std:  {np.std(noise_vals):6.1f} deg/s")
        
        if np.max(noise_vals) > 100:
            print("  WARNING: Very high noise detected - possible crash or prop strike")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_autotune.py <logfile.csv>")
        sys.exit(1)
    
    analyze_autotune(sys.argv[1])
