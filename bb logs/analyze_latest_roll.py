#!/usr/bin/env python3
"""Analyze latest log - filter for ROLL axis only"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the log
df = pd.read_csv('btfl___milestone_4_log_started weird but ended good.bbl.csv', skiprows=147)

log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6

print(f'Log: {len(df)} samples, {df["rel_time"].max():.1f}s duration')

df['iter_mode'] = df['debug[1]']
df['mode'] = df['iter_mode'] % 10
df['state'] = df['debug[0]']

# Filter for ROLL mode only (mode=1), non-ARMED state
roll_mask = (df['mode'] == 1) & (df['state'] != 1)
roll_df = df[roll_mask]

print(f"\nROLL axis data: {len(roll_df)} samples")

print("\n" + "="*100)
print("ROLL AXIS PID GAINS PROGRESSION")
print("="*100)

sample_times = np.arange(45, 110, 3)
print(f"\n{'Time':>6} {'P':>6} {'D':>6} {'I':>6} {'F':>6}")
print("-" * 40)

gains_history = []
for t in sample_times:
    mask = (roll_df['rel_time'] >= t) & (roll_df['rel_time'] < t + 2)
    if mask.any():
        p = int(roll_df.loc[mask, 'debug[2]'].median())
        d = int(roll_df.loc[mask, 'debug[3]'].median())
        i = int(roll_df.loc[mask, 'debug[4]'].median()) if 'debug[4]' in roll_df.columns else 0
        f = int(roll_df.loc[mask, 'debug[5]'].median()) if 'debug[5]' in roll_df.columns else 0
        print(f"{t:6.0f} {p:6d} {d:6d} {i:6d} {f:6d}")
        gains_history.append({'time': t, 'P': p, 'D': d, 'I': i, 'F': f})

if gains_history:
    gains_df = pd.DataFrame(gains_history)
    
    print(f"\nROLL Gain ranges:")
    print(f"  P: {gains_df['P'].min()} - {gains_df['P'].max()}")
    print(f"  D: {gains_df['D'].min()} - {gains_df['D'].max()}")
    print(f"  I: {gains_df['I'].min()} - {gains_df['I'].max()}")
    print(f"  F: {gains_df['F'].min()} - {gains_df['F'].max()}")
    
    # F-term stability check
    if len(gains_df) >= 5:
        last5 = gains_df.tail(5)
        print(f"\nLast 5 ROLL readings:")
        print(f"  F values: {list(last5['F'].values)}")
        print(f"  F std: {last5['F'].std():.1f} (stable if < 10)")

# Now analyze just ROLL maneuvers
print("\n" + "="*100)
print("ROLL AXIS MANEUVER RESPONSE")
print("="*100)

df['setpoint_roll'] = df['setpoint[0]']
df['gyro_roll'] = df['gyroADC[0]']

maneuver_threshold = 150
df['abs_setpoint'] = df['setpoint_roll'].abs()
df['in_maneuver'] = df['abs_setpoint'] > maneuver_threshold
df['maneuver_start'] = df['in_maneuver'] & ~df['in_maneuver'].shift(1).fillna(False)

# Only count maneuvers when in ROLL mode
roll_maneuver_starts = df[df['maneuver_start'] & (df['mode'] == 1)]
print(f"\nFound {len(roll_maneuver_starts)} ROLL maneuvers")

def analyze_maneuver(start_t, window_ms=300):
    window = df[(df['rel_time'] >= start_t - 0.02) & (df['rel_time'] <= start_t + window_ms/1000)]
    if len(window) < 10:
        return None
    
    setpoint = window['setpoint_roll'].values
    gyro = window['gyro_roll'].values
    t = (window['rel_time'].values - start_t) * 1000
    
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
    
    return {'tracking_error': tracking_error, 'overshoot': overshoot, 'lag_ms': lag_ms, 'peak_ratio': peak_ratio}

results = []
for idx, row in roll_maneuver_starts.iterrows():
    result = analyze_maneuver(row['rel_time'])
    if result:
        result['time'] = row['rel_time']
        results.append(result)

if results:
    results_df = pd.DataFrame(results)
    
    total_time = results_df['time'].max() - results_df['time'].min()
    t_min = results_df['time'].min()
    early = results_df[results_df['time'] < t_min + total_time * 0.33]
    late = results_df[results_df['time'] >= t_min + total_time * 0.66]
    
    print(f"\nEARLY ROLL ({len(early)} maneuvers):")
    if len(early) > 0:
        print(f"  Tracking error: {early['tracking_error'].mean():.1f}")
        print(f"  Lag: {early['lag_ms'].mean():.1f}ms")
        print(f"  Overshoot: {early['overshoot'].mean():.1f}%")
        print(f"  Peak ratio: {early['peak_ratio'].mean():.2f}")
    
    print(f"\nLATE ROLL ({len(late)} maneuvers):")
    if len(late) > 0:
        print(f"  Tracking error: {late['tracking_error'].mean():.1f}")
        print(f"  Lag: {late['lag_ms'].mean():.1f}ms")
        print(f"  Overshoot: {late['overshoot'].mean():.1f}%")
        print(f"  Peak ratio: {late['peak_ratio'].mean():.2f}")
    
    if len(early) > 0 and len(late) > 0:
        early_err = early['tracking_error'].mean()
        late_err = late['tracking_error'].mean()
        if late_err < early_err * 0.8:
            print(f"\n✓ ROLL TUNING IMPROVED! Error: {early_err:.1f} → {late_err:.1f}")
        elif late_err > early_err * 1.2:
            print(f"\n⚠️ ROLL tuning got worse. Error: {early_err:.1f} → {late_err:.1f}")
        else:
            print(f"\n~ ROLL tuning stable. Error: {early_err:.1f} → {late_err:.1f}")

# Plot F evolution for ROLL only
fig, ax = plt.subplots(figsize=(12, 5))
step = max(1, len(roll_df) // 1000)
ax.plot(roll_df['rel_time'].iloc[::step], roll_df['debug[5]'].iloc[::step], 'r-', alpha=0.7, linewidth=2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('F gain')
ax.set_title('F Gain Evolution - ROLL Axis Only')
ax.grid(True, alpha=0.3)
ax.axhline(160, color='green', linestyle='--', alpha=0.5, label='Target ~160')
ax.legend()
plt.tight_layout()
plt.savefig('m4_latest_roll_f.png', dpi=150)
print(f"\nSaved F evolution plot to m4_latest_roll_f.png")

plt.show()
