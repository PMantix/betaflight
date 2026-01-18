#!/usr/bin/env python3
"""Deep analysis of roll response - compare early vs late F values"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the log
df = pd.read_csv('btfl___milestone_4_log_2.bbl.csv', skiprows=147)

# Calculate relative time
log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6

print(f'Log: {len(df)} samples, {df["rel_time"].max():.1f}s duration')

# setpoint[0] = roll, gyroADC[0] = roll
df['setpoint_roll'] = df['setpoint[0]']
df['gyro_roll'] = df['gyroADC[0]']

print()
print('='*100)
print('FINDING ROLL MANEUVERS')
print('='*100)

# Find roll maneuvers (significant setpoint)
maneuver_threshold = 150  # deg/s
df['abs_setpoint'] = df['setpoint_roll'].abs()
df['in_maneuver'] = df['abs_setpoint'] > maneuver_threshold
df['maneuver_start'] = df['in_maneuver'] & ~df['in_maneuver'].shift(1).fillna(False)

maneuver_starts = df[df['maneuver_start']]
print(f"Found {len(maneuver_starts)} roll maneuvers")

# Group maneuvers by F value period
# From the debug output we know F changes at specific times
f_periods = [
    (27.0, 30.5, 65),
    (30.5, 34.0, 70),
    (34.0, 36.5, 75),
    (36.5, 39.0, 80),
    (39.0, 42.5, 85),
    (42.5, 44.5, 90),
    (44.5, 49.0, 95),
    (49.0, 52.5, 100),
    (52.5, 55.0, 105),
    (55.0, 58.0, 110),
    (70.0, 73.0, 135),
    (85.0, 88.0, 165),
    (100.0, 103.0, 199),
    (110.0, 113.0, 213),
]

print()
print('='*100)
print('MANEUVER ANALYSIS BY F VALUE')
print('='*100)

def analyze_maneuver(start_t, window_ms=300):
    """Analyze a single maneuver starting at start_t"""
    window = df[(df['rel_time'] >= start_t - 0.02) & (df['rel_time'] <= start_t + window_ms/1000)]
    
    if len(window) < 10:
        return None
    
    setpoint = window['setpoint_roll'].values
    gyro = window['gyro_roll'].values
    t = (window['rel_time'].values - start_t) * 1000  # ms
    
    # Find peak setpoint and peak gyro
    sp_peak_idx = np.argmax(np.abs(setpoint))
    gyro_peak_idx = np.argmax(np.abs(gyro))
    
    sp_peak_t = t[sp_peak_idx]
    gyro_peak_t = t[gyro_peak_idx]
    
    sp_peak_val = setpoint[sp_peak_idx]
    gyro_peak_val = gyro[gyro_peak_idx]
    
    # Lag = time difference between peaks
    lag_ms = gyro_peak_t - sp_peak_t
    
    # Tracking error = RMS difference
    tracking_error = np.sqrt(np.mean((setpoint - gyro)**2))
    
    # Peak ratio
    if abs(sp_peak_val) > 10:
        peak_ratio = gyro_peak_val / sp_peak_val
    else:
        peak_ratio = 1.0
    
    return {
        'sp_peak': sp_peak_val,
        'gyro_peak': gyro_peak_val,
        'lag_ms': lag_ms,
        'tracking_error': tracking_error,
        'peak_ratio': peak_ratio
    }

# Analyze maneuvers in each F period
for start, end, f_val in f_periods:
    period_maneuvers = maneuver_starts[(maneuver_starts['rel_time'] >= start) & 
                                        (maneuver_starts['rel_time'] <= end)]
    
    if len(period_maneuvers) == 0:
        continue
    
    print(f"\nF = {f_val} ({start:.0f}s - {end:.0f}s): {len(period_maneuvers)} maneuvers")
    
    lags = []
    errors = []
    ratios = []
    
    for idx, row in period_maneuvers.iterrows():
        result = analyze_maneuver(row['rel_time'])
        if result:
            lags.append(result['lag_ms'])
            errors.append(result['tracking_error'])
            ratios.append(result['peak_ratio'])
            print(f"  t={row['rel_time']:.2f}s: sp={result['sp_peak']:.0f} gyro={result['gyro_peak']:.0f} lag={result['lag_ms']:.1f}ms err={result['tracking_error']:.1f} ratio={result['peak_ratio']:.2f}")
    
    if lags:
        print(f"  --> Avg lag={np.mean(lags):.1f}ms, Avg error={np.mean(errors):.1f}, Avg ratio={np.mean(ratios):.2f}")

# Now let's plot a few representative maneuvers
print()
print('='*100)
print('PLOTTING COMPARISON: LOW F vs HIGH F')
print('='*100)

fig, axes = plt.subplots(2, 3, figsize=(15, 8))

# Low F maneuvers (around F=70)
low_f_maneuvers = maneuver_starts[(maneuver_starts['rel_time'] >= 27) & 
                                   (maneuver_starts['rel_time'] <= 35)].head(3)

# High F maneuvers (around F=200)
high_f_maneuvers = maneuver_starts[(maneuver_starts['rel_time'] >= 100) & 
                                    (maneuver_starts['rel_time'] <= 115)].head(3)

for i, (idx, row) in enumerate(low_f_maneuvers.iterrows()):
    if i >= 3:
        break
    ax = axes[0, i]
    start_t = row['rel_time']
    window = df[(df['rel_time'] >= start_t - 0.02) & (df['rel_time'] <= start_t + 0.25)]
    t = (window['rel_time'] - start_t) * 1000
    
    ax.plot(t, window['setpoint_roll'], label='Setpoint', linewidth=2)
    ax.plot(t, window['gyro_roll'], label='Gyro', linewidth=2, alpha=0.8)
    ax.axhline(0, color='gray', linestyle='--', alpha=0.3)
    ax.axvline(0, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Rate (deg/s)')
    ax.set_title(f'LOW F (~70) at t={start_t:.1f}s')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-20, 250)

for i, (idx, row) in enumerate(high_f_maneuvers.iterrows()):
    if i >= 3:
        break
    ax = axes[1, i]
    start_t = row['rel_time']
    window = df[(df['rel_time'] >= start_t - 0.02) & (df['rel_time'] <= start_t + 0.25)]
    t = (window['rel_time'] - start_t) * 1000
    
    ax.plot(t, window['setpoint_roll'], label='Setpoint', linewidth=2)
    ax.plot(t, window['gyro_roll'], label='Gyro', linewidth=2, alpha=0.8)
    ax.axhline(0, color='gray', linestyle='--', alpha=0.3)
    ax.axvline(0, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Rate (deg/s)')
    ax.set_title(f'HIGH F (~200) at t={start_t:.1f}s')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-20, 250)

plt.tight_layout()
plt.savefig('m4_low_vs_high_f.png', dpi=150)
print("Saved to m4_low_vs_high_f.png")

# Also plot the progression of metrics over time
print()
print('='*100)
print('TRACKING METRICS OVER TIME')
print('='*100)

all_results = []
for idx, row in maneuver_starts.iterrows():
    result = analyze_maneuver(row['rel_time'])
    if result:
        result['time'] = row['rel_time']
        all_results.append(result)

results_df = pd.DataFrame(all_results)

if len(results_df) > 0:
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    ax = axes[0, 0]
    ax.scatter(results_df['time'], results_df['lag_ms'], alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Lag (ms)')
    ax.set_title('Peak Lag Over Time (as F increases)')
    ax.axhline(0, color='red', linestyle='--', alpha=0.5)
    ax.grid(True, alpha=0.3)
    
    ax = axes[0, 1]
    ax.scatter(results_df['time'], results_df['tracking_error'], alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Tracking Error (RMS)')
    ax.set_title('Tracking Error Over Time')
    ax.grid(True, alpha=0.3)
    
    ax = axes[1, 0]
    ax.scatter(results_df['time'], results_df['peak_ratio'], alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Peak Ratio (gyro/setpoint)')
    ax.set_title('Peak Ratio Over Time')
    ax.axhline(1, color='red', linestyle='--', alpha=0.5)
    ax.grid(True, alpha=0.3)
    
    ax = axes[1, 1]
    ax.scatter(results_df['time'], results_df['sp_peak'].abs(), alpha=0.5, label='Setpoint peak')
    ax.scatter(results_df['time'], results_df['gyro_peak'].abs(), alpha=0.5, label='Gyro peak')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Peak Rate (deg/s)')
    ax.set_title('Peak Rates Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('m4_tracking_metrics.png', dpi=150)
    print("Saved to m4_tracking_metrics.png")
    
    # Print summary
    print()
    print("Summary statistics:")
    print(f"  Lag: mean={results_df['lag_ms'].mean():.1f}ms, std={results_df['lag_ms'].std():.1f}ms")
    print(f"  Tracking error: mean={results_df['tracking_error'].mean():.1f}, std={results_df['tracking_error'].std():.1f}")
    print(f"  Peak ratio: mean={results_df['peak_ratio'].mean():.2f}, std={results_df['peak_ratio'].std():.2f}")
