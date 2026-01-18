#!/usr/bin/env python3
"""
Analyze autotune flight log to verify P0/P2/P3 fixes.
Focus on: P/D/F balance, relaxation phases, damping classification
"""

import pandas as pd
import numpy as np
import sys

# State and mode definitions
STATE_NAMES = ['IDLE', 'ARMED', 'DETECTING', 'COLLECTING', 'SETTLING', 'ANALYZING', 'ADJUSTING', 'SIGNALING', 'COMPLETE', 'ABORTED']
MODE_NAMES = ['NONE', 'ROLL', 'PITCH', 'FILTER', 'HOVER']  # HOVER=4

# Reason codes organized by category for analysis
REASON_CODES = {
    # Response classification (1000-1009)
    1000: 'EXCELLENT',
    1001: 'UNDERDAMPED', 
    1002: 'OVERDAMPED',
    1003: 'NO_RESPONSE',
    1004: 'NOISY',
    1005: 'CRITICAL',  # NEW - P3 damping detection
    
    # Hover diagnostic (1100-1139)
    1110: 'HOVER_ROLL_IMPROVED',
    1111: 'HOVER_PITCH_IMPROVED',
    1112: 'HOVER_GYRO_LPF1_IMPROVED',
    1113: 'HOVER_DTERM_LPF1_IMPROVED',
    1120: 'HOVER_APPLYING_BEST',
    1130: 'HOVER_MULTI_VAR_APPLIED',
    
    # Relaxation codes (1140-1149) - NEW P2
    1140: 'RELAX_ROLL_SUCCESS',
    1141: 'RELAX_PITCH_SUCCESS',
    1142: 'RELAX_GYRO_LPF1_SUCCESS',
    1143: 'RELAX_DTERM_LPF1_SUCCESS',
    
    # PID - P adjustments (3100-3199)
    3110: 'P_DOWN_OVERSHOOT',
    3120: 'D_UP_OVERSHOOT',
    3210: 'P_UP_SLUGGISH',
    3220: 'D_DOWN_SLUGGISH',
    3310: 'D_UP_OSCILLATION',
    3320: 'P_DOWN_OSCILLATION',
    3410: 'D_DOWN_NOISE',
    3420: 'I_UP_DRIFT',
    3430: 'I_DOWN_BOUNCEBACK',
    3440: 'I_DOWN_SLOW_OSC',
    3500: 'RESPONSE_GOOD',
    3510: 'F_UP_LAG',
    3520: 'F_DOWN_LEAD',
    3530: 'F_UP_PHASE',
    3600: 'AT_LIMIT',
    3999: 'PID_COMPLETE',
    
    # Filter adjustments
    9100: 'FILTER_BASELINE',
}

# Categorize reason codes by which parameter they adjust
P_ADJUSTMENT_CODES = {3110, 3210, 3320}  # P down/up
D_ADJUSTMENT_CODES = {3120, 3220, 3310, 3410}  # D up/down
I_ADJUSTMENT_CODES = {3420, 3430, 3440}  # I up/down
F_ADJUSTMENT_CODES = {3510, 3520, 3530}  # F up/down
RELAXATION_CODES = {1140, 1141, 1142, 1143}  # P2 relaxation
RESPONSE_CLASS_CODES = {1000, 1001, 1002, 1003, 1004, 1005}  # Response classification

def find_header_row(filename):
    """Find the row containing column headers."""
    with open(filename, 'r') as f:
        for i, line in enumerate(f):
            if 'loopIteration' in line:
                return i
    return 0

def main():
    # Use command line arg or default
    filename = sys.argv[1] if len(sys.argv) > 1 else 'btfl__mk4___subagent.bbl.csv'
    
    header_row = find_header_row(filename)
    df = pd.read_csv(filename, skiprows=header_row)
    
    print(f"=" * 80)
    print(f"AUTOTUNE LOG ANALYSIS: {filename}")
    print(f"=" * 80)
    print(f"Loaded {len(df)} samples")
    
    # Get debug columns
    d0 = df['debug[0]'].values  # state
    d1 = df['debug[1]'].values  # mode|iter
    d2 = df['debug[2]'].values  # P or filter1
    d3 = df['debug[3]'].values  # D or filter2
    d4 = df['debug[4]'].values if 'debug[4]' in df.columns else np.zeros(len(df))
    d5 = df['debug[5]'].values if 'debug[5]' in df.columns else np.zeros(len(df))
    d6 = df['debug[6]'].values if 'debug[6]' in df.columns else np.zeros(len(df))
    d7 = df['debug[7]'].values if 'debug[7]' in df.columns else np.zeros(len(df))
    
    # Calculate relative time  
    time_us = df['time'].values
    rel_time = (time_us - time_us[0]) / 1e6
    
    # Decode mode
    mode_vals = (d1 % 10).astype(int)
    iter_vals = (d1 // 10).astype(int)
    
    # =========================================================================
    # P0 VERIFICATION: Attribution Balance
    # =========================================================================
    print("\n" + "=" * 80)
    print("P0 VERIFICATION: P/D/F Attribution Balance")
    print("=" * 80)
    
    # Count reason codes by category
    p_adj_count = 0
    d_adj_count = 0
    i_adj_count = 0
    f_adj_count = 0
    
    for r in d7[~np.isnan(d7)]:
        r = int(r)
        if r in P_ADJUSTMENT_CODES:
            p_adj_count += 1
        elif r in D_ADJUSTMENT_CODES:
            d_adj_count += 1
        elif r in I_ADJUSTMENT_CODES:
            i_adj_count += 1
        elif r in F_ADJUSTMENT_CODES:
            f_adj_count += 1
    
    total_adj = p_adj_count + d_adj_count + i_adj_count + f_adj_count
    
    print(f"\nAdjustment Distribution (by reason codes in ADJUSTING state):")
    print(f"  P adjustments: {p_adj_count:6d} ({100*p_adj_count/max(1,total_adj):5.1f}%)")
    print(f"  D adjustments: {d_adj_count:6d} ({100*d_adj_count/max(1,total_adj):5.1f}%)")
    print(f"  I adjustments: {i_adj_count:6d} ({100*i_adj_count/max(1,total_adj):5.1f}%)")
    print(f"  F adjustments: {f_adj_count:6d} ({100*f_adj_count/max(1,total_adj):5.1f}%)")
    
    if f_adj_count > 0.8 * total_adj and total_adj > 100:
        print("\n  ⚠️  WARNING: F still dominates (>80%) - P0 fix may not be working")
    elif p_adj_count + d_adj_count > 0:
        print("\n  ✅ P and/or D adjustments detected - P0 fix appears to be working")
    
    # =========================================================================
    # P2 VERIFICATION: Relaxation Phases
    # =========================================================================
    print("\n" + "=" * 80)
    print("P2 VERIFICATION: Bidirectional Exploration (Relaxation)")
    print("=" * 80)
    
    relax_count = 0
    relax_codes_seen = {}
    for r in d7[~np.isnan(d7)]:
        r = int(r)
        if r in RELAXATION_CODES:
            relax_count += 1
            relax_codes_seen[r] = relax_codes_seen.get(r, 0) + 1
    
    if relax_count > 0:
        print(f"\n  ✅ Relaxation activity detected: {relax_count} samples")
        for code, count in relax_codes_seen.items():
            print(f"     {REASON_CODES.get(code, f'CODE_{code}')}: {count}")
    else:
        print(f"\n  ℹ️  No relaxation codes found")
        print(f"     (Relaxation only triggers when RMS < 8.0)")
        
        # Check if RMS was ever low enough
        rms_vals = d6[~np.isnan(d6)] / 10.0
        low_rms_count = np.sum(rms_vals < 8.0)
        print(f"     Samples with RMS < 8.0: {low_rms_count}")
    
    # =========================================================================
    # P3 VERIFICATION: Critical Damping Detection
    # =========================================================================
    print("\n" + "=" * 80)
    print("P3 VERIFICATION: Critical Damping Classification")
    print("=" * 80)
    
    response_class_counts = {}
    for r in d7[~np.isnan(d7)]:
        r = int(r)
        if r in RESPONSE_CLASS_CODES:
            response_class_counts[r] = response_class_counts.get(r, 0) + 1
    
    print(f"\nResponse Classifications:")
    for code in sorted(response_class_counts.keys()):
        name = REASON_CODES.get(code, f'CODE_{code}')
        count = response_class_counts[code]
        print(f"  {name:20s}: {count:6d}")
    
    if 1005 in response_class_counts:
        print(f"\n  ✅ RESPONSE_CRITICAL (1005) detected - P3 damping classification working")
    else:
        print(f"\n  ℹ️  RESPONSE_CRITICAL not seen (may not have hit 0.5-0.7 damping range)")
    
    # =========================================================================
    # GAIN CHANGE TRACKING
    # =========================================================================
    print("\n" + "=" * 80)
    print("GAIN CHANGE TIMELINE")
    print("=" * 80)
    
    pid_mask = (mode_vals == 1) | (mode_vals == 2)  # ROLL or PITCH
    adjusting_pid_mask = pid_mask & (d0 == 6)  # In ADJUSTING state
    
    if adjusting_pid_mask.any():
        pid_adj_indices = np.where(adjusting_pid_mask)[0]
        
        # Track gain changes
        prev_p, prev_d, prev_i, prev_f = None, None, None, None
        gain_changes = []
        
        for i in pid_adj_indices[::max(1, len(pid_adj_indices)//100)]:
            p = int(d2[i])
            d = int(d3[i])
            iv = int(d4[i])
            f = int(d5[i])
            t = rel_time[i]
            reason = int(d7[i]) if not pd.isna(d7[i]) else 0
            
            if prev_p is not None:
                if p != prev_p or d != prev_d or iv != prev_i or f != prev_f:
                    gain_changes.append({
                        'time': t,
                        'dP': p - prev_p,
                        'dD': d - prev_d,
                        'dI': iv - prev_i,
                        'dF': f - prev_f,
                        'P': p, 'D': d, 'I': iv, 'F': f,
                        'reason': reason,
                        'reason_name': REASON_CODES.get(reason, f'CODE_{reason}')
                    })
            
            prev_p, prev_d, prev_i, prev_f = p, d, iv, f
        
        if gain_changes:
            print(f"\n{'Time':>6s} | {'P':>4s} {'D':>4s} {'I':>4s} {'F':>5s} | {'ΔP':>4s} {'ΔD':>4s} {'ΔI':>4s} {'ΔF':>5s} | {'Reason':>25s}")
            print("-" * 85)
            for gc in gain_changes[:40]:
                print(f"{gc['time']:6.1f} | {gc['P']:4d} {gc['D']:4d} {gc['I']:4d} {gc['F']:5d} | {gc['dP']:+4d} {gc['dD']:+4d} {gc['dI']:+4d} {gc['dF']:+5d} | {gc['reason_name']:>25s}")
            
            if len(gain_changes) > 40:
                print(f"  ... ({len(gain_changes) - 40} more changes)")
            
            # Summary
            print(f"\nTotal changes: {len(gain_changes)}")
            p_changes = sum(1 for gc in gain_changes if gc['dP'] != 0)
            d_changes = sum(1 for gc in gain_changes if gc['dD'] != 0)
            i_changes = sum(1 for gc in gain_changes if gc['dI'] != 0)
            f_changes = sum(1 for gc in gain_changes if gc['dF'] != 0)
            print(f"  P changed: {p_changes} times")
            print(f"  D changed: {d_changes} times")
            print(f"  I changed: {i_changes} times")
            print(f"  F changed: {f_changes} times")
            
            total_dP = sum(gc['dP'] for gc in gain_changes)
            total_dD = sum(gc['dD'] for gc in gain_changes)
            total_dI = sum(gc['dI'] for gc in gain_changes)
            total_dF = sum(gc['dF'] for gc in gain_changes)
            print(f"\nNet changes: ΔP={total_dP:+d}, ΔD={total_dD:+d}, ΔI={total_dI:+d}, ΔF={total_dF:+d}")
            
            # Final gains
            if gain_changes:
                final = gain_changes[-1]
                print(f"Final gains: P={final['P']}, D={final['D']}, I={final['I']}, F={final['F']}")
    
    # =========================================================================
    # CONVERGENCE ANALYSIS
    # =========================================================================
    print("\n" + "=" * 80)
    print("CONVERGENCE ANALYSIS")
    print("=" * 80)
    
    # Check how many iterations per mode
    roll_iters = iter_vals[mode_vals == 1]
    pitch_iters = iter_vals[mode_vals == 2]
    
    if len(roll_iters) > 0:
        print(f"\nRoll mode: {roll_iters.max()} iterations")
    if len(pitch_iters) > 0:
        print(f"Pitch mode: {pitch_iters.max()} iterations")
    
    # RMS progression
    rms_vals = d6 / 10.0
    analyzing_mask = (d0 == 5)  # ANALYZING state
    if analyzing_mask.any():
        analyzing_rms = rms_vals[analyzing_mask]
        if len(analyzing_rms) > 0:
            print(f"\nNoise (RMS) during ANALYZING:")
            print(f"  Initial: {analyzing_rms[0]:.1f}")
            print(f"  Final:   {analyzing_rms[-1]:.1f}")
            print(f"  Min:     {np.min(analyzing_rms):.1f}")
            print(f"  Max:     {np.max(analyzing_rms):.1f}")
    
    # =========================================================================
    # REASON CODE SUMMARY
    # =========================================================================
    print("\n" + "=" * 80)
    print("TOP REASON CODES")
    print("=" * 80)
    
    reason_counts = {}
    for r in d7[~np.isnan(d7)]:
        r = int(r)
        if r > 0:
            reason_counts[r] = reason_counts.get(r, 0) + 1
    
    for reason, count in sorted(reason_counts.items(), key=lambda x: -x[1])[:15]:
        reason_name = REASON_CODES.get(reason, f'CODE_{reason}')
        print(f"  {reason:5d} {reason_name:35s}: {count:6d}")

if __name__ == "__main__":
    main()
