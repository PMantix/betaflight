#!/usr/bin/env python3
"""
Analyze Newton's method autotune logs
Focus on diagnostic phases, improvement percentages, and convergence behavior.

Debug channels for hover diagnostic (DEBUG_AUTOTUNE):
[0] = State (0-9)
[1] = phase*10 + iteration (e.g., 21 = phase 2, iteration 1)
[2] = baseline RMS * 10
[3] = current RMS * 10
[4] = roll improvement % (signed)
[5] = pitch improvement %
[6] = filter improvement %
[7] = reason code (1500-1590 for diagnostic, 3xxx for PID tune)

Reason codes:
1500 = DIAG_BASELINE
1510 = DIAG_ROLL_TEST
1520 = DIAG_PITCH_TEST
1530 = DIAG_GYRO_LPF1_TEST
1540 = DIAG_DTERM_LPF1_TEST
1550 = DIAG_ANALYZING
1560 = DIAG_FIX_ROLL
1561 = DIAG_FIX_PITCH
1562 = DIAG_FIX_GYRO_LPF1
1563 = DIAG_FIX_DTERM_LPF1
1570 = DIAG_NO_IMPROVEMENT
1580 = DIAG_TARGET_REACHED
1590 = DIAG_MAX_ITERATIONS

3001 = PID_COLLECTING
3510 = PID_LAG_F_UP
9100 = GRACE_PERIOD
9300 = AT_LIMIT
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
    9: "ERROR"
}

REASON_CODES = {
    1000: "HOVER_WAITING",
    1001: "HOVER_MEASURING",
    1500: "DIAG_BASELINE",
    1510: "DIAG_ROLL_TEST",
    1520: "DIAG_PITCH_TEST",
    1530: "DIAG_GYRO_LPF1_TEST",
    1540: "DIAG_DTERM_LPF1_TEST",
    1550: "DIAG_ANALYZING",
    1560: "DIAG_FIX_ROLL",
    1561: "DIAG_FIX_PITCH",
    1562: "DIAG_FIX_GYRO_LPF1",
    1563: "DIAG_FIX_DTERM_LPF1",
    1570: "DIAG_NO_IMPROVEMENT",
    1580: "DIAG_TARGET_REACHED",
    1590: "DIAG_MAX_ITERATIONS",
    2001: "FILTER_COLLECTING",
    3000: "PID_WAITING",
    3001: "PID_COLLECTING",
    3110: "PID_OVERSHOOT_P_DOWN",
    3120: "PID_OVERSHOOT_D_UP",
    3210: "PID_SLUGGISH_P_UP",
    3510: "PID_LAG_F_UP",
    9000: "IDLE",
    9100: "GRACE_PERIOD",
    9200: "DATA_INVALID",
    9300: "AT_LIMIT",
}

def get_reason_name(code):
    return REASON_CODES.get(int(code), f"UNKNOWN_{int(code)}")

def main(filename):
    print(f"\n{'='*70}")
    print(f"Newton's Method Autotune Analysis - {filename}")
    print(f"{'='*70}\n")
    
    # First 147 rows are header metadata, row 148 has column names
    df = pd.read_csv(filename, skiprows=147)
    
    print(f"Total samples: {len(df)}")
    print(f"Duration: {(df['time'].max() - df['time'].min()) / 1e6:.1f} seconds")
    
    # State distribution
    print(f"\n{'='*50}")
    print("STATE DISTRIBUTION")
    print(f"{'='*50}")
    state_counts = df['debug[0]'].value_counts().sort_index()
    for state, count in state_counts.items():
        name = STATE_NAMES.get(int(state), "UNKNOWN")
        pct = count / len(df) * 100
        print(f"  {name:12s}: {count:6d} samples ({pct:5.1f}%)")
    
    # Reason code distribution
    print(f"\n{'='*50}")
    print("REASON CODE DISTRIBUTION")
    print(f"{'='*50}")
    reason_counts = df['debug[7]'].value_counts().sort_index()
    for reason, count in reason_counts.items():
        if count > 10:  # Skip noise
            name = get_reason_name(reason)
            pct = count / len(df) * 100
            print(f"  {int(reason):5d} ({name:25s}): {count:6d} ({pct:5.1f}%)")
    
    # Find all ADJUSTING phases and extract what was adjusted
    print(f"\n{'='*50}")
    print("ADJUSTMENT EVENTS (State=6)")
    print(f"{'='*50}")
    
    adjusting = df[df['debug[0]'] == 6].copy()
    if len(adjusting) > 0:
        # Group by time blocks (adjustments close together)
        adjusting['time_gap'] = adjusting['time'].diff() > 100000  # 100ms gap
        adjusting['adj_group'] = adjusting['time_gap'].cumsum()
        
        print(f"{'Time':>12} {'Reason':>25} {'Phase.Iter':>10} {'BaseRMS':>8} {'CurrRMS':>8} {'Roll%':>7} {'Pitch%':>7} {'Filt%':>7}")
        print("-" * 100)
        
        for group_id in adjusting['adj_group'].unique():
            group = adjusting[adjusting['adj_group'] == group_id]
            first = group.iloc[0]
            
            time_sec = first['time'] / 1e6
            reason = int(first['debug[7]'])
            reason_name = get_reason_name(reason)
            phase_iter = int(first['debug[1]'])
            phase = phase_iter // 10
            iteration = phase_iter % 10
            base_rms = first['debug[2]'] / 10.0
            curr_rms = first['debug[3]'] / 10.0
            roll_imp = first['debug[4]']
            pitch_imp = first['debug[5]']
            filt_imp = first['debug[6]']
            
            print(f"{time_sec:12.3f} {reason_name:>25} {phase}.{iteration:>8} {base_rms:8.1f} {curr_rms:8.1f} {roll_imp:7.0f} {pitch_imp:7.0f} {filt_imp:7.0f}")
    
    # Analyze RMS convergence over time
    print(f"\n{'='*50}")
    print("RMS CONVERGENCE ANALYSIS")
    print(f"{'='*50}")
    
    # Look at baseline RMS values over diagnostic iterations
    analyzing = df[df['debug[0]'] == 5].copy()  # ANALYZING state
    if len(analyzing) > 0:
        analyzing['time_gap'] = analyzing['time'].diff() > 100000
        analyzing['analyze_group'] = analyzing['time_gap'].cumsum()
        
        print(f"\n{'Iteration':>10} {'Time(s)':>10} {'BaseRMS':>10} {'CurrRMS':>10} {'Best Improv':>12}")
        print("-" * 60)
        
        for i, group_id in enumerate(analyzing['analyze_group'].unique()):
            group = analyzing[analyzing['analyze_group'] == group_id]
            first = group.iloc[0]
            
            time_sec = first['time'] / 1e6
            base_rms = first['debug[2]'] / 10.0
            curr_rms = first['debug[3]'] / 10.0
            roll_imp = first['debug[4]']
            pitch_imp = first['debug[5]']
            filt_imp = first['debug[6]']
            best_imp = max(roll_imp, pitch_imp, filt_imp)
            
            print(f"{i+1:10d} {time_sec:10.2f} {base_rms:10.1f} {curr_rms:10.1f} {best_imp:12.0f}%")
    
    # Look at PID tuning if present
    pid_tune = df[df['debug[7]'].isin([3001, 3110, 3120, 3210, 3510])]
    if len(pid_tune) > 0:
        print(f"\n{'='*50}")
        print("PID TUNE EVENTS")
        print(f"{'='*50}")
        
        pid_adjusting = df[(df['debug[0]'] == 6) & (df['debug[7]'].isin([3110, 3120, 3210, 3510]))]
        if len(pid_adjusting) > 0:
            pid_adjusting = pid_adjusting.copy()
            pid_adjusting['time_gap'] = pid_adjusting['time'].diff() > 100000
            pid_adjusting['pid_group'] = pid_adjusting['time_gap'].cumsum()
            
            print(f"\n{'Time(s)':>10} {'Reason':>25} {'d[1]':>8} {'d[2]':>8} {'d[3]':>8} {'d[4]':>8}")
            print("-" * 70)
            
            for group_id in pid_adjusting['pid_group'].unique():
                group = pid_adjusting[pid_adjusting['pid_group'] == group_id]
                first = group.iloc[0]
                
                time_sec = first['time'] / 1e6
                reason = int(first['debug[7]'])
                reason_name = get_reason_name(reason)
                
                print(f"{time_sec:10.2f} {reason_name:>25} {int(first['debug[1]']):8} {int(first['debug[2]']):8} {int(first['debug[3]']):8} {int(first['debug[4]']):8}")
    
    # Motor RMS during hover
    print(f"\n{'='*50}")
    print("MOTOR STATISTICS")
    print(f"{'='*50}")
    
    armed = df[df['debug[0]'] >= 1]  # Armed or later states
    if len(armed) > 0:
        for i in range(4):
            col = f'motor[{i}]'
            if col in df.columns:
                motor = armed[col]
                mean = motor.mean()
                std = motor.std()
                print(f"  Motor {i}: mean={mean:.0f}, std={std:.1f} (RMS proxy)")
    
    # Summary
    print(f"\n{'='*50}")
    print("SUMMARY")
    print(f"{'='*50}")
    
    n_adjustments = len(adjusting['adj_group'].unique()) if len(adjusting) > 0 else 0
    n_analyze = len(analyzing['analyze_group'].unique()) if len(analyzing) > 0 else 0
    n_signaling = (df['debug[0]'] == 7).sum()
    
    print(f"  Total diagnostic iterations: {n_analyze}")
    print(f"  Total adjustments applied: {n_adjustments}")
    print(f"  Wiggle signals: {n_signaling // 500 if n_signaling > 0 else 0} (approx)")
    
    # Check for PID tune vs hover tune
    hover_reasons = df['debug[7]'].isin([1500, 1510, 1520, 1530, 1540, 1550, 1560, 1561, 1562, 1563]).sum()
    pid_reasons = df['debug[7]'].isin([3001, 3110, 3120, 3210, 3510]).sum()
    
    if hover_reasons > 0:
        print(f"  Hover tune samples: {hover_reasons}")
    if pid_reasons > 0:
        print(f"  PID tune samples: {pid_reasons}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        filename = "btfl_002__newton_1.bbl.csv"
    else:
        filename = sys.argv[1]
    
    main(filename)
