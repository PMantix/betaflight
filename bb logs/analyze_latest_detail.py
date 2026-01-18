#!/usr/bin/env python3
"""Detailed look at each ROLL maneuver"""
import pandas as pd
import numpy as np

df = pd.read_csv('btfl___milestone_4_log_started weird but ended good.bbl.csv', skiprows=147)
log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6

df['iter_mode'] = df['debug[1]']
df['mode'] = df['iter_mode'] % 10
df['state'] = df['debug[0]']
df['setpoint_roll'] = df['setpoint[0]']
df['gyro_roll'] = df['gyroADC[0]']

maneuver_threshold = 150
df['abs_setpoint'] = df['setpoint_roll'].abs()
df['in_maneuver'] = df['abs_setpoint'] > maneuver_threshold
df['maneuver_start'] = df['in_maneuver'] & ~df['in_maneuver'].shift(1).fillna(False)

roll_maneuver_starts = df[df['maneuver_start'] & (df['mode'] == 1)]

def analyze_maneuver(start_t, window_ms=300):
    window = df[(df['rel_time'] >= start_t - 0.02) & (df['rel_time'] <= start_t + window_ms/1000)]
    if len(window) < 10:
        return None
    
    setpoint = window['setpoint_roll'].values
    gyro = window['gyro_roll'].values
    t = (window['rel_time'].values - start_t) * 1000
    
    # Get F value at this time
    f_vals = window['debug[5]'] if 'debug[5]' in window.columns else None
    f = int(f_vals.median()) if f_vals is not None else 0
    
    tracking_error = np.sqrt(np.mean((setpoint - gyro)**2))
    
    sp_peak_idx = np.argmax(np.abs(setpoint))
    gyro_peak_idx = np.argmax(np.abs(gyro))
    sp_peak = setpoint[sp_peak_idx]
    gyro_peak = gyro[gyro_peak_idx]
    
    sp_peak_t = t[sp_peak_idx]
    gyro_peak_t = t[gyro_peak_idx]
    lag_ms = gyro_peak_t - sp_peak_t
    
    if abs(sp_peak) > 10:
        overshoot = (gyro_peak - sp_peak) / sp_peak * 100 * np.sign(sp_peak)
        peak_ratio = gyro_peak / sp_peak
    else:
        overshoot = 0
        peak_ratio = 1.0
    
    return {'time': start_t, 'F': f, 'error': tracking_error, 'lag_ms': lag_ms, 
            'overshoot': overshoot, 'ratio': peak_ratio, 'sp_peak': sp_peak}

print("="*100)
print("ALL ROLL MANEUVERS - DETAILED")
print("="*100)
print(f"\n{'#':>3} {'Time':>7} {'F':>5} {'Error':>7} {'Lag(ms)':>8} {'Overshoot':>10} {'Ratio':>7} {'SP Peak':>9}")
print("-" * 70)

results = []
for idx, row in roll_maneuver_starts.iterrows():
    result = analyze_maneuver(row['rel_time'])
    if result:
        results.append(result)
        i = len(results)
        print(f"{i:3d} {result['time']:7.1f} {result['F']:5d} {result['error']:7.1f} {result['lag_ms']:8.1f} "
              f"{result['overshoot']:9.1f}% {result['ratio']:7.2f} {result['sp_peak']:9.1f}")

if results:
    results_df = pd.DataFrame(results)
    
    # Show convergence of F with corresponding response quality
    print("\n" + "="*100)
    print("F GAIN vs RESPONSE QUALITY")
    print("="*100)
    
    # Group by approximate F value
    results_df['F_bucket'] = (results_df['F'] // 20) * 20  # Round to nearest 20
    for f_bucket in sorted(results_df['F_bucket'].unique()):
        bucket_data = results_df[results_df['F_bucket'] == f_bucket]
        if len(bucket_data) > 0:
            print(f"F ~ {f_bucket:3d}: error={bucket_data['error'].mean():5.1f}, "
                  f"lag={bucket_data['lag_ms'].mean():5.1f}ms, "
                  f"overshoot={bucket_data['overshoot'].mean():5.1f}%, "
                  f"ratio={bucket_data['ratio'].mean():.2f}  ({len(bucket_data)} maneuvers)")
