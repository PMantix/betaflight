#!/usr/bin/env python3
"""
Analyze autotune iterations from blackbox log
Shows gain changes and metrics for each tuning cycle
"""

import pandas as pd
import numpy as np
import sys

# State names from autotune_types.h
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

STATUS_NAMES = {
    0: "OK",
    1: "WAITING",
    2: "DETECTED",
    3: "COLLECTING",
    4: "SETTLE",
    5: "ANALYZING",
    6: "ADJ_P",
    7: "ADJ_D", 
    8: "ADJ_I",
    9: "ADJ_F",
    10: "ADJ_FILTER",
    11: "SIGNAL",
    12: "COMPLETE"
}

def main(filename):
    print(f"\n{'='*80}")
    print(f"AUTOTUNE ITERATION ANALYSIS - {filename}")
    print(f"{'='*80}\n")
    
    # Read CSV - first 147 rows are header metadata
    df = pd.read_csv(filename, skiprows=147)
    
    print(f"Total samples: {len(df)}")
    print(f"Duration: {(df['time'].iloc[-1] - df['time'].iloc[0])/1e6:.1f} seconds\n")
    
    # Find state transitions
    df['state_change'] = df['debug[0]'].diff().fillna(0) != 0
    transitions = df[df['state_change']].copy()
    
    # Track iterations by finding SIGNALING → ARMED transitions
    iterations = []
    current_iter = {}
    
    for idx, row in df.iterrows():
        state = int(row['debug[0]'])
        
        if state == 5:  # ANALYZING
            current_iter['P'] = int(row['debug[2]'])
            current_iter['D'] = int(row['debug[3]'])
            current_iter['status'] = int(row['debug[4]'])
            current_iter['overshoot_x10'] = int(row['debug[5]'])
            current_iter['noise_x10'] = int(row['debug[6]'])
            current_iter['score_x100'] = int(row['debug[7]'])
            current_iter['analyze_time'] = row['time']
            
        elif state == 6:  # ADJUSTING
            current_iter['adj_status'] = int(row['debug[4]'])
            
        elif state == 7:  # SIGNALING
            # Capture new gains after adjustment
            current_iter['new_P'] = int(row['debug[2]'])
            current_iter['new_D'] = int(row['debug[3]'])
            current_iter['iteration'] = int(row['debug[1]']) >> 4  # Upper bits are iteration
            
        elif state == 1 and current_iter:  # Back to ARMED
            if 'P' in current_iter:
                iterations.append(current_iter.copy())
            current_iter = {}
    
    # Don't forget last iteration if still in progress
    if current_iter and 'P' in current_iter:
        iterations.append(current_iter)
    
    print(f"{'='*80}")
    print(f"{'Iter':>4} {'P':>4}→{'P2':>4} {'D':>4}→{'D2':>4} {'Over%':>6} {'Noise':>6} {'Score':>7} {'Status'}")
    print(f"{'='*80}")
    
    for i, it in enumerate(iterations):
        p_before = it.get('P', 0)
        p_after = it.get('new_P', p_before)
        d_before = it.get('D', 0)
        d_after = it.get('new_D', d_before)
        overshoot = it.get('overshoot_x10', 0) / 10.0
        noise = it.get('noise_x10', 0) / 10.0
        score = it.get('score_x100', 0) / 100.0
        status = STATUS_NAMES.get(it.get('adj_status', 0), f"?{it.get('adj_status', 0)}")
        
        p_change = "→" if p_before == p_after else "↑" if p_after > p_before else "↓"
        d_change = "→" if d_before == d_after else "↑" if d_after > d_before else "↓"
        
        print(f"{i+1:4d} {p_before:4d}{p_change}{p_after:4d} {d_before:4d}{d_change}{d_after:4d} "
              f"{overshoot:6.1f} {noise:6.1f} {score:7.1f} {status}")
    
    print(f"{'='*80}\n")
    
    # Summary
    if iterations:
        first_p = iterations[0].get('P', 0)
        last_p = iterations[-1].get('new_P', iterations[-1].get('P', 0))
        first_d = iterations[0].get('D', 0)
        last_d = iterations[-1].get('new_D', iterations[-1].get('D', 0))
        
        print(f"Summary:")
        print(f"  Iterations completed: {len(iterations)}")
        print(f"  P: {first_p} → {last_p} ({'+' if last_p >= first_p else ''}{last_p - first_p})")
        print(f"  D: {first_d} → {last_d} ({'+' if last_d >= first_d else ''}{last_d - first_d})")
        
        scores = [it.get('score_x100', 0)/100.0 for it in iterations if it.get('score_x100', 0) < 32767]
        if scores:
            print(f"  Score: {scores[0]:.1f} → {scores[-1]:.1f}")
            print(f"  Best score: {min(scores):.1f}")
    
    # Show gyro rates during maneuvers
    print(f"\n{'='*80}")
    print("MANEUVER ANALYSIS")
    print(f"{'='*80}\n")
    
    detecting = df[df['debug[0]'] == 2]  # DETECTING state
    if len(detecting) > 0:
        roll_max = detecting['gyroADC[0]'].abs().max()
        pitch_max = detecting['gyroADC[1]'].abs().max()
        print(f"Peak roll rate during maneuvers: {roll_max:.0f}°/s")
        print(f"Peak pitch rate during maneuvers: {pitch_max:.0f}°/s")
        
        # Count maneuvers by looking at DETECTING periods
        detecting_periods = (df['debug[0]'] == 2).astype(int).diff().fillna(0)
        maneuver_starts = (detecting_periods == 1).sum()
        print(f"Number of maneuvers detected: {maneuver_starts}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        filename = "Abtfl_004_short.bbl.csv"
    else:
        filename = sys.argv[1]
    
    main(filename)
