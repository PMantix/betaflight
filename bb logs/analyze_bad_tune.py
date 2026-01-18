#!/usr/bin/env python3
"""Analyze the bad tune log to see what Newton's method did wrong"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the log
df = pd.read_csv('btfl___milestone_4_log_bad_tune.bbl.csv', skiprows=147)

# Calculate relative time
log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6

print(f'Log: {len(df)} samples, {df["rel_time"].max():.1f}s duration')

# Decode mode from debug[1]
def decode_iter_mode(val):
    mode = int(val) % 10
    iteration = int(val) // 10
    return iteration, mode

df['iter_mode'] = df['debug[1]']
df['mode'] = df['iter_mode'] % 10
df['state'] = df['debug[0]']
df['iteration'] = df['iter_mode'] // 10

print("\n" + "="*100)
print("DEBUG VALUES OVER TIME")
print("="*100)

# Sample at regular intervals
max_time = df['rel_time'].max()
sample_times = np.arange(5, min(max_time, 120), 3)

print(f"\n{'Time':>6} {'State':>7} {'Mode':>6} {'Iter':>5} | {'P':>5} {'D':>5} {'I':>5} {'F':>5} | {'Reason':>6}")
print("-" * 80)

mode_names = {0: 'NONE', 1: 'ROLL', 2: 'PITCH', 3: 'FILTER'}
state_names = {0: 'IDLE', 1: 'ARMED', 2: 'DETECT', 3: 'COLLCT', 4: 'SETTLE', 5: 'ANALYZ', 6: 'ADJUST', 7: 'SIGNAL'}

for t in sample_times:
    mask = (df['rel_time'] >= t) & (df['rel_time'] < t + 1)
    if mask.any():
        state = int(df.loc[mask, 'debug[0]'].median())
        iter_mode = int(df.loc[mask, 'debug[1]'].median())
        iteration, mode = decode_iter_mode(iter_mode)
        
        d2 = int(df.loc[mask, 'debug[2]'].median())
        d3 = int(df.loc[mask, 'debug[3]'].median())
        d4 = int(df.loc[mask, 'debug[4]'].median()) if 'debug[4]' in df.columns else 0
        d5 = int(df.loc[mask, 'debug[5]'].median()) if 'debug[5]' in df.columns else 0
        d7 = int(df.loc[mask, 'debug[7]'].median()) if 'debug[7]' in df.columns else 0
        
        mode_str = mode_names.get(mode, f'?{mode}')
        state_str = state_names.get(state, f'?{state}')
        
        # In PID mode, debug values are P, D, I, F (but D shows hoverThrottle when ARMED)
        if mode in [1, 2]:  # ROLL or PITCH
            p, i, f = d2, d4, d5
            d = d3 if state != 1 else "hthr"
            print(f"{t:6.0f} {state_str:>7} {mode_str:>6} {iteration:5d} | {p:5d} {str(d):>5} {i:5d} {f:5d} | {d7:6d}")
        elif mode == 3:  # FILTER
            print(f"{t:6.0f} {state_str:>7} {mode_str:>6} {iteration:5d} | lpf1={d2:3d} lpf2={d3:3d} gyro1={d4:3d} gyro2={d5:3d} | {d7:6d}")

# Extract just PID mode, non-ARMED data for actual gains
print("\n" + "="*100)
print("ACTUAL PID GAINS OVER TIME (PID mode, non-ARMED states only)")
print("="*100)

pid_mask = (df['mode'].isin([1, 2])) & (df['state'] != 1)
pid_df = df[pid_mask]

if len(pid_df) > 0:
    print(f"\n{'Time':>6} {'P':>6} {'D':>6} {'I':>6} {'F':>6}")
    print("-" * 40)
    
    gains_history = []
    for t in sample_times:
        mask = (pid_df['rel_time'] >= t) & (pid_df['rel_time'] < t + 2)
        if mask.any():
            p = int(pid_df.loc[mask, 'debug[2]'].median())
            d = int(pid_df.loc[mask, 'debug[3]'].median())
            i = int(pid_df.loc[mask, 'debug[4]'].median()) if 'debug[4]' in pid_df.columns else 0
            f = int(pid_df.loc[mask, 'debug[5]'].median()) if 'debug[5]' in pid_df.columns else 0
            print(f"{t:6.0f} {p:6d} {d:6d} {i:6d} {f:6d}")
            gains_history.append({'time': t, 'P': p, 'D': d, 'I': i, 'F': f})
    
    if gains_history:
        gains_df = pd.DataFrame(gains_history)
        
        # Show the range of each gain
        print(f"\nGain ranges:")
        print(f"  P: {gains_df['P'].min()} - {gains_df['P'].max()} (delta: {gains_df['P'].max() - gains_df['P'].min()})")
        print(f"  D: {gains_df['D'].min()} - {gains_df['D'].max()} (delta: {gains_df['D'].max() - gains_df['D'].min()})")
        print(f"  I: {gains_df['I'].min()} - {gains_df['I'].max()} (delta: {gains_df['I'].max() - gains_df['I'].min()})")
        print(f"  F: {gains_df['F'].min()} - {gains_df['F'].max()} (delta: {gains_df['F'].max() - gains_df['F'].min()})")

# Analyze maneuver response quality
print("\n" + "="*100)
print("MANEUVER RESPONSE ANALYSIS")
print("="*100)

df['setpoint_roll'] = df['setpoint[0]']
df['gyro_roll'] = df['gyroADC[0]']

# Find maneuvers
maneuver_threshold = 150
df['abs_setpoint'] = df['setpoint_roll'].abs()
df['in_maneuver'] = df['abs_setpoint'] > maneuver_threshold
df['maneuver_start'] = df['in_maneuver'] & ~df['in_maneuver'].shift(1).fillna(False)

maneuver_starts = df[df['maneuver_start']]
print(f"\nFound {len(maneuver_starts)} roll maneuvers")

def analyze_maneuver(start_t, window_ms=300):
    window = df[(df['rel_time'] >= start_t - 0.02) & (df['rel_time'] <= start_t + window_ms/1000)]
    if len(window) < 10:
        return None
    
    setpoint = window['setpoint_roll'].values
    gyro = window['gyro_roll'].values
    
    tracking_error = np.sqrt(np.mean((setpoint - gyro)**2))
    
    sp_peak_idx = np.argmax(np.abs(setpoint))
    gyro_peak_idx = np.argmax(np.abs(gyro))
    sp_peak = setpoint[sp_peak_idx]
    gyro_peak = gyro[gyro_peak_idx]
    
    if abs(sp_peak) > 10:
        overshoot = (gyro_peak - sp_peak) / sp_peak * 100 * np.sign(sp_peak)
    else:
        overshoot = 0
    
    return {'tracking_error': tracking_error, 'overshoot': overshoot, 'sp_peak': sp_peak, 'gyro_peak': gyro_peak}

results = []
for idx, row in maneuver_starts.iterrows():
    result = analyze_maneuver(row['rel_time'])
    if result:
        result['time'] = row['rel_time']
        results.append(result)

if results:
    results_df = pd.DataFrame(results)
    
    # Early vs Late comparison
    mid_time = results_df['time'].median()
    early = results_df[results_df['time'] < mid_time]
    late = results_df[results_df['time'] >= mid_time]
    
    print(f"\nEARLY ({len(early)} maneuvers, t < {mid_time:.0f}s):")
    print(f"  Tracking error: {early['tracking_error'].mean():.1f} ± {early['tracking_error'].std():.1f}")
    print(f"  Overshoot: {early['overshoot'].mean():.1f}% ± {early['overshoot'].std():.1f}")
    
    print(f"\nLATE ({len(late)} maneuvers, t >= {mid_time:.0f}s):")
    print(f"  Tracking error: {late['tracking_error'].mean():.1f} ± {late['tracking_error'].std():.1f}")
    print(f"  Overshoot: {late['overshoot'].mean():.1f}% ± {late['overshoot'].std():.1f}")
    
    # Did it get better or worse?
    early_err = early['tracking_error'].mean()
    late_err = late['tracking_error'].mean()
    if late_err > early_err * 1.2:
        print(f"\n⚠️  TUNING GOT WORSE! Error increased from {early_err:.1f} to {late_err:.1f}")
    elif late_err < early_err * 0.8:
        print(f"\n✓ Tuning improved. Error decreased from {early_err:.1f} to {late_err:.1f}")
    else:
        print(f"\n~ Tuning roughly stable. Error: {early_err:.1f} -> {late_err:.1f}")

# Plot gains evolution
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

step = max(1, len(pid_df) // 1000) if len(pid_df) > 0 else 1

if len(pid_df) > 0:
    ax = axes[0, 0]
    ax.plot(pid_df['rel_time'].iloc[::step], pid_df['debug[2]'].iloc[::step], 'b-', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('P gain')
    ax.set_title('P Gain Evolution')
    ax.grid(True, alpha=0.3)
    
    ax = axes[0, 1]
    ax.plot(pid_df['rel_time'].iloc[::step], pid_df['debug[3]'].iloc[::step], 'g-', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('D gain')
    ax.set_title('D Gain Evolution')
    ax.grid(True, alpha=0.3)
    
    if 'debug[4]' in pid_df.columns:
        ax = axes[1, 0]
        ax.plot(pid_df['rel_time'].iloc[::step], pid_df['debug[4]'].iloc[::step], 'orange', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('I gain')
        ax.set_title('I Gain Evolution')
        ax.grid(True, alpha=0.3)
    
    if 'debug[5]' in pid_df.columns:
        ax = axes[1, 1]
        ax.plot(pid_df['rel_time'].iloc[::step], pid_df['debug[5]'].iloc[::step], 'r-', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('F gain')
        ax.set_title('F Gain Evolution')
        ax.grid(True, alpha=0.3)

plt.suptitle('PID Gains - Bad Tune Log', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('m4_bad_tune_gains.png', dpi=150)
print(f"\nSaved plot to m4_bad_tune_gains.png")

plt.show()
