#!/usr/bin/env python3
"""
Deep analysis of autotune metrics to understand why P isn't being adjusted.
"""

import pandas as pd
import numpy as np
import sys

def find_header_row(filename):
    with open(filename, 'r') as f:
        for i, line in enumerate(f):
            if 'loopIteration' in line:
                return i
    return 0

def main():
    filename = sys.argv[1] if len(sys.argv) > 1 else 'btfl__mk4___subagent_2.bbl.csv'
    
    header_row = find_header_row(filename)
    df = pd.read_csv(filename, skiprows=header_row)
    
    print(f"Loaded {len(df)} samples from {filename}")
    
    # Get relevant columns
    time_us = df['time'].values
    rel_time = (time_us - time_us[0]) / 1e6
    
    # Debug columns
    d0 = df['debug[0]'].values  # state
    d1 = df['debug[1]'].values  # mode|iter
    d2 = df['debug[2]'].values  # P 
    d3 = df['debug[3]'].values  # D
    d4 = df['debug[4]'].values if 'debug[4]' in df.columns else np.zeros(len(df))
    d5 = df['debug[5]'].values if 'debug[5]' in df.columns else np.zeros(len(df))
    d6 = df['debug[6]'].values if 'debug[6]' in df.columns else np.zeros(len(df))  # overshoot*10 or RMS*10
    d7 = df['debug[7]'].values if 'debug[7]' in df.columns else np.zeros(len(df))
    
    # Gyro and setpoint
    gyro_roll = df['gyroADC[0]'].values if 'gyroADC[0]' in df.columns else np.zeros(len(df))
    gyro_pitch = df['gyroADC[1]'].values if 'gyroADC[1]' in df.columns else np.zeros(len(df))
    setpoint_roll = df['setpoint[0]'].values if 'setpoint[0]' in df.columns else np.zeros(len(df))
    setpoint_pitch = df['setpoint[1]'].values if 'setpoint[1]' in df.columns else np.zeros(len(df))
    
    mode_vals = (d1 % 10).astype(int)
    iter_vals = (d1 // 10).astype(int)
    
    # Find ROLL mode periods
    roll_mask = (mode_vals == 1)
    
    # Find ANALYZING state periods in ROLL mode
    analyzing_mask = (d0 == 5) & roll_mask  # ANALYZING = 5
    
    print("\n" + "=" * 80)
    print("RESPONSE ANALYSIS - Looking at actual gyro vs setpoint")
    print("=" * 80)
    
    # Sample rate estimation
    if len(time_us) > 1:
        avg_dt = np.mean(np.diff(time_us[:1000]))
        sample_rate = 1e6 / avg_dt
        print(f"Sample rate: {sample_rate:.0f} Hz")
    
    # Find maneuvers (high setpoint periods)
    maneuver_threshold = 100  # deg/s
    in_maneuver = np.abs(setpoint_roll) > maneuver_threshold
    
    # Find maneuver starts
    maneuver_starts = []
    for i in range(1, len(in_maneuver)):
        if in_maneuver[i] and not in_maneuver[i-1]:
            maneuver_starts.append(i)
    
    print(f"\nFound {len(maneuver_starts)} roll maneuvers")
    
    # Analyze first few maneuvers
    for m_idx, start_idx in enumerate(maneuver_starts[:5]):
        print(f"\n--- Maneuver {m_idx+1} at t={rel_time[start_idx]:.2f}s ---")
        
        # Find end of maneuver
        end_idx = start_idx
        for i in range(start_idx, min(start_idx + 2000, len(in_maneuver))):
            if not in_maneuver[i]:
                end_idx = i
                break
        
        # Get response window (extend 500ms after maneuver end)
        window_end = min(end_idx + int(0.5 * sample_rate), len(gyro_roll))
        
        # Slice data
        t_window = rel_time[start_idx:window_end]
        gyro_window = gyro_roll[start_idx:window_end]
        sp_window = setpoint_roll[start_idx:window_end]
        
        # Key metrics
        peak_setpoint = np.max(np.abs(sp_window))
        peak_gyro = np.max(np.abs(gyro_window))
        
        # Rise time: time from 10% to 90% of peak setpoint
        threshold_10 = 0.1 * peak_setpoint
        threshold_90 = 0.9 * peak_setpoint
        
        idx_10 = None
        idx_90 = None
        for i, g in enumerate(gyro_window):
            if idx_10 is None and np.abs(g) > threshold_10:
                idx_10 = i
            if idx_90 is None and np.abs(g) > threshold_90:
                idx_90 = i
                break
        
        if idx_10 is not None and idx_90 is not None and idx_90 > idx_10:
            rise_time_ms = (idx_90 - idx_10) / sample_rate * 1000
        else:
            rise_time_ms = 0
        
        # Overshoot
        if peak_setpoint > 0:
            overshoot_pct = max(0, (peak_gyro - peak_setpoint) / peak_setpoint * 100)
        else:
            overshoot_pct = 0
        
        # Tracking error during maneuver
        tracking_error = np.mean(np.abs(gyro_window[:len(sp_window)] - sp_window))
        
        print(f"  Peak setpoint: {peak_setpoint:.1f} deg/s")
        print(f"  Peak gyro:     {peak_gyro:.1f} deg/s")
        print(f"  Rise time:     {rise_time_ms:.1f} ms")
        print(f"  Overshoot:     {overshoot_pct:.1f}%")
        print(f"  Tracking err:  {tracking_error:.1f} deg/s")
        
        # What would damping ratio be?
        if overshoot_pct > 0:
            from math import log, sqrt, pi
            ln_os = log(overshoot_pct / 100)
            damping = -ln_os / sqrt(pi**2 + ln_os**2)
            print(f"  Damping ratio: {damping:.2f}")
        else:
            print(f"  Damping ratio: >1.0 (no overshoot)")
        
        # Check lag
        # Find delay between setpoint rise and gyro rise
        sp_rise_idx = None
        for i, s in enumerate(sp_window):
            if np.abs(s) > 10:
                sp_rise_idx = i
                break
        
        gyro_rise_idx = None
        for i, g in enumerate(gyro_window):
            if np.abs(g) > 10:
                gyro_rise_idx = i
                break
        
        if sp_rise_idx is not None and gyro_rise_idx is not None:
            lag_ms = (gyro_rise_idx - sp_rise_idx) / sample_rate * 1000
            print(f"  Response lag:  {lag_ms:.1f} ms")
    
    # Overall tracking quality
    print("\n" + "=" * 80)
    print("OVERALL TRACKING QUALITY")
    print("=" * 80)
    
    # During maneuvers, how well does gyro track setpoint?
    maneuver_mask = in_maneuver & (mode_vals == 1)
    if maneuver_mask.any():
        maneuver_gyro = gyro_roll[maneuver_mask]
        maneuver_sp = setpoint_roll[maneuver_mask]
        
        avg_tracking_error = np.mean(np.abs(maneuver_gyro - maneuver_sp))
        rms_tracking_error = np.sqrt(np.mean((maneuver_gyro - maneuver_sp)**2))
        max_tracking_error = np.max(np.abs(maneuver_gyro - maneuver_sp))
        
        # Correlation
        if np.std(maneuver_gyro) > 0 and np.std(maneuver_sp) > 0:
            correlation = np.corrcoef(maneuver_gyro, maneuver_sp)[0, 1]
        else:
            correlation = 0
        
        print(f"During {np.sum(maneuver_mask)} maneuver samples:")
        print(f"  Avg tracking error:  {avg_tracking_error:.1f} deg/s")
        print(f"  RMS tracking error:  {rms_tracking_error:.1f} deg/s")
        print(f"  Max tracking error:  {max_tracking_error:.1f} deg/s")
        print(f"  Correlation:         {correlation:.3f}")
        
        # Check if gyro is consistently below setpoint (sluggish)
        gyro_below = np.sum(np.abs(maneuver_gyro) < np.abs(maneuver_sp) * 0.9)
        gyro_above = np.sum(np.abs(maneuver_gyro) > np.abs(maneuver_sp) * 1.1)
        print(f"  Gyro < 90% setpoint: {gyro_below} samples ({100*gyro_below/len(maneuver_gyro):.1f}%)")
        print(f"  Gyro > 110% setpoint: {gyro_above} samples ({100*gyro_above/len(maneuver_gyro):.1f}%)")
        
        if gyro_below > gyro_above * 2:
            print("\n  ⚠️  SLUGGISH: Gyro consistently below setpoint - P is too low!")
        elif gyro_above > gyro_below * 2:
            print("\n  ⚠️  OVERSHOOTING: Gyro consistently above setpoint - P is too high!")
    
    # Check what gains we ended up with
    print("\n" + "=" * 80)
    print("FINAL GAINS")
    print("=" * 80)
    
    # Get non-zero P values
    valid_p = d2[d2 > 0]
    valid_d = d3[d3 > 0]
    
    if len(valid_p) > 0:
        print(f"P: {int(valid_p[-1])} (range: {int(np.min(valid_p))}-{int(np.max(valid_p))})")
    if len(valid_d) > 0:
        print(f"D: {int(valid_d[-1])} (range: {int(np.min(valid_d))}-{int(np.max(valid_d))})")

if __name__ == "__main__":
    main()
