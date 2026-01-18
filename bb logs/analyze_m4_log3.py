#!/usr/bin/env python3
"""Analysis of milestone 4 log 3 - check convergence and final tuning quality"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the log
df = pd.read_csv('btfl___milestone_4_log_3.bbl.csv', skiprows=147)

# Calculate relative time
log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6

print(f'Log: {len(df)} samples, {df["rel_time"].max():.1f}s duration')

# setpoint[0] = roll, gyroADC[0] = roll
df['setpoint_roll'] = df['setpoint[0]']
df['gyro_roll'] = df['gyroADC[0]']

# Check for debug columns that might show F value
debug_cols = [c for c in df.columns if 'debug' in c.lower()]
print(f"Debug columns: {debug_cols}")

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
    
    # Overshoot detection
    sign = np.sign(sp_peak_val)
    if sign != 0:
        overshoot = (gyro_peak_val - sp_peak_val) * sign / abs(sp_peak_val) * 100
    else:
        overshoot = 0
    
    return {
        'sp_peak': sp_peak_val,
        'gyro_peak': gyro_peak_val,
        'lag_ms': lag_ms,
        'tracking_error': tracking_error,
        'peak_ratio': peak_ratio,
        'overshoot': overshoot
    }

# Analyze all maneuvers
print()
print('='*100)
print('ALL MANEUVERS ANALYSIS')
print('='*100)

all_results = []
for idx, row in maneuver_starts.iterrows():
    result = analyze_maneuver(row['rel_time'])
    if result:
        result['time'] = row['rel_time']
        all_results.append(result)

results_df = pd.DataFrame(all_results)

if len(results_df) > 0:
    # Divide into early/middle/late phases
    total_time = results_df['time'].max()
    early = results_df[results_df['time'] < total_time * 0.33]
    middle = results_df[(results_df['time'] >= total_time * 0.33) & (results_df['time'] < total_time * 0.66)]
    late = results_df[results_df['time'] >= total_time * 0.66]
    
    print(f"\nEARLY PHASE (t < {total_time*0.33:.0f}s): {len(early)} maneuvers")
    if len(early) > 0:
        print(f"  Lag: {early['lag_ms'].mean():.1f}ms ± {early['lag_ms'].std():.1f}")
        print(f"  Tracking error: {early['tracking_error'].mean():.1f} ± {early['tracking_error'].std():.1f}")
        print(f"  Overshoot: {early['overshoot'].mean():.1f}% ± {early['overshoot'].std():.1f}")
        print(f"  Peak ratio: {early['peak_ratio'].mean():.2f} ± {early['peak_ratio'].std():.2f}")
    
    print(f"\nMIDDLE PHASE ({total_time*0.33:.0f}s - {total_time*0.66:.0f}s): {len(middle)} maneuvers")
    if len(middle) > 0:
        print(f"  Lag: {middle['lag_ms'].mean():.1f}ms ± {middle['lag_ms'].std():.1f}")
        print(f"  Tracking error: {middle['tracking_error'].mean():.1f} ± {middle['tracking_error'].std():.1f}")
        print(f"  Overshoot: {middle['overshoot'].mean():.1f}% ± {middle['overshoot'].std():.1f}")
        print(f"  Peak ratio: {middle['peak_ratio'].mean():.2f} ± {middle['peak_ratio'].std():.2f}")
    
    print(f"\nLATE PHASE (t >= {total_time*0.66:.0f}s): {len(late)} maneuvers")
    if len(late) > 0:
        print(f"  Lag: {late['lag_ms'].mean():.1f}ms ± {late['lag_ms'].std():.1f}")
        print(f"  Tracking error: {late['tracking_error'].mean():.1f} ± {late['tracking_error'].std():.1f}")
        print(f"  Overshoot: {late['overshoot'].mean():.1f}% ± {late['overshoot'].std():.1f}")
        print(f"  Peak ratio: {late['peak_ratio'].mean():.2f} ± {late['peak_ratio'].std():.2f}")
    
    # Convergence check - last 5 maneuvers
    last5 = results_df.tail(5)
    print(f"\nLAST 5 MANEUVERS (final tuning quality):")
    for i, (idx, row) in enumerate(last5.iterrows()):
        print(f"  {i+1}. t={row['time']:.1f}s: lag={row['lag_ms']:.1f}ms, err={row['tracking_error']:.1f}, "
              f"overshoot={row['overshoot']:.1f}%, ratio={row['peak_ratio']:.2f}")
    
    print(f"\n  Average of last 5:")
    print(f"    Lag: {last5['lag_ms'].mean():.1f}ms")
    print(f"    Tracking error: {last5['tracking_error'].mean():.1f}")
    print(f"    Overshoot: {last5['overshoot'].mean():.1f}%")
    print(f"    Peak ratio: {last5['peak_ratio'].mean():.2f}")
    
    # Plot convergence
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    ax = axes[0, 0]
    ax.scatter(results_df['time'], results_df['lag_ms'], alpha=0.6, s=30)
    # Add rolling average
    if len(results_df) > 5:
        rolling = results_df['lag_ms'].rolling(5, center=True).mean()
        ax.plot(results_df['time'], rolling, 'r-', linewidth=2, label='5-point rolling avg')
    ax.axhline(0, color='green', linestyle='--', alpha=0.7, label='Ideal (0ms)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Lag (ms)')
    ax.set_title('Response Lag Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    ax = axes[0, 1]
    ax.scatter(results_df['time'], results_df['tracking_error'], alpha=0.6, s=30)
    if len(results_df) > 5:
        rolling = results_df['tracking_error'].rolling(5, center=True).mean()
        ax.plot(results_df['time'], rolling, 'r-', linewidth=2, label='5-point rolling avg')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Tracking Error (RMS)')
    ax.set_title('Tracking Error Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    ax = axes[1, 0]
    ax.scatter(results_df['time'], results_df['overshoot'], alpha=0.6, s=30)
    if len(results_df) > 5:
        rolling = results_df['overshoot'].rolling(5, center=True).mean()
        ax.plot(results_df['time'], rolling, 'r-', linewidth=2, label='5-point rolling avg')
    ax.axhline(0, color='green', linestyle='--', alpha=0.7, label='No overshoot')
    ax.axhline(10, color='orange', linestyle='--', alpha=0.7, label='10% target')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Overshoot (%)')
    ax.set_title('Overshoot Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    ax = axes[1, 1]
    ax.scatter(results_df['time'], results_df['peak_ratio'], alpha=0.6, s=30)
    if len(results_df) > 5:
        rolling = results_df['peak_ratio'].rolling(5, center=True).mean()
        ax.plot(results_df['time'], rolling, 'r-', linewidth=2, label='5-point rolling avg')
    ax.axhline(1.0, color='green', linestyle='--', alpha=0.7, label='Ideal (1.0)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Peak Ratio (gyro/setpoint)')
    ax.set_title('Peak Amplitude Ratio Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.suptitle('Autotune Convergence Analysis - Log 3', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('m4_log3_convergence.png', dpi=150)
    print("\nSaved convergence plot to m4_log3_convergence.png")
    
    # Plot early vs late comparison
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Find representative maneuvers from early and late
    early_maneuvers = maneuver_starts[maneuver_starts['rel_time'] < total_time * 0.2].head(3)
    late_maneuvers = maneuver_starts[maneuver_starts['rel_time'] > total_time * 0.8].tail(3)
    
    ax = axes[0]
    colors = plt.cm.Blues(np.linspace(0.4, 0.8, len(early_maneuvers)))
    for i, (idx, row) in enumerate(early_maneuvers.iterrows()):
        start_t = row['rel_time']
        window = df[(df['rel_time'] >= start_t - 0.02) & (df['rel_time'] <= start_t + 0.25)]
        t = (window['rel_time'] - start_t) * 1000
        ax.plot(t, window['setpoint_roll'], '--', color=colors[i], alpha=0.7, label=f'SP t={start_t:.1f}s')
        ax.plot(t, window['gyro_roll'], '-', color=colors[i], alpha=0.9)
    ax.axhline(0, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Rate (deg/s)')
    ax.set_title('EARLY Phase Maneuvers (before tuning)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-20, 250)
    
    ax = axes[1]
    colors = plt.cm.Greens(np.linspace(0.4, 0.8, len(late_maneuvers)))
    for i, (idx, row) in enumerate(late_maneuvers.iterrows()):
        start_t = row['rel_time']
        window = df[(df['rel_time'] >= start_t - 0.02) & (df['rel_time'] <= start_t + 0.25)]
        t = (window['rel_time'] - start_t) * 1000
        ax.plot(t, window['setpoint_roll'], '--', color=colors[i], alpha=0.7, label=f'SP t={start_t:.1f}s')
        ax.plot(t, window['gyro_roll'], '-', color=colors[i], alpha=0.9)
    ax.axhline(0, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Rate (deg/s)')
    ax.set_title('LATE Phase Maneuvers (after tuning)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-20, 250)
    
    plt.suptitle('Early vs Late Maneuver Comparison - Log 3', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('m4_log3_comparison.png', dpi=150)
    print("Saved comparison plot to m4_log3_comparison.png")
    
    # Iteration count estimate
    print()
    print('='*100)
    print('CONVERGENCE SPEED ANALYSIS')
    print('='*100)
    
    # Estimate when we reached "good" tuning (tracking error < 20, lag < 10ms)
    good_threshold_error = 25
    good_threshold_lag = 15
    
    good_results = results_df[(results_df['tracking_error'] < good_threshold_error) & 
                               (abs(results_df['lag_ms']) < good_threshold_lag)]
    
    if len(good_results) > 0:
        first_good_time = good_results['time'].iloc[0]
        print(f"First 'good' result (err<{good_threshold_error}, |lag|<{good_threshold_lag}ms): t={first_good_time:.1f}s")
        print(f"Time to reach good tuning: {first_good_time:.1f}s")
        print(f"Iterations before good: ~{len(results_df[results_df['time'] < first_good_time])} maneuvers")
    else:
        print("Did not reach 'good' threshold in this log")
    
    # Check if still improving at end
    if len(results_df) >= 10:
        last10 = results_df.tail(10)
        first5_of_last10 = last10.head(5)
        last5_of_last10 = last10.tail(5)
        
        improvement = first5_of_last10['tracking_error'].mean() - last5_of_last10['tracking_error'].mean()
        print(f"\nStill improving? Last 10 maneuvers:")
        print(f"  First 5 avg error: {first5_of_last10['tracking_error'].mean():.1f}")
        print(f"  Last 5 avg error: {last5_of_last10['tracking_error'].mean():.1f}")
        print(f"  Improvement: {improvement:.1f} ({'still improving' if improvement > 2 else 'converged'})")

else:
    print("No maneuvers found to analyze!")

plt.show()
