#!/usr/bin/env python3
"""Correct interpretation of debug values in log 5 - accounting for state-dependent meanings"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the log
df = pd.read_csv('btfl___milestone_4_log_5.bbl.csv', skiprows=147)

# Calculate relative time
log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6

print(f'Log: {len(df)} samples, {df["rel_time"].max():.1f}s duration')

# Debug column mapping depends on mode and state:
# 
# FILTER MODE (mode=3):
#   debug[2] = dterm_lpf1 Hz
#   debug[3] = dterm_lpf2 Hz
#   debug[4] = gyro_lpf1 Hz
#   debug[5] = gyro_lpf2 Hz
#
# PID MODE (mode=1,2):
#   debug[2] = P gain (or throttle% if !hoverCalibrated)
#   debug[3] = D gain (or hoverThrottle if state==ARMED && hoverCalibrated)
#   debug[4] = I gain
#   debug[5] = F gain
#
# State values:
#   0 = IDLE
#   1 = ARMED (waiting for maneuver) - shows hoverThrottle in debug[3]!
#   2 = DETECTING
#   3 = COLLECTING
#   4 = SETTLING
#   5 = ANALYZING
#   6 = ADJUSTING
#   7 = SIGNALING

# Mode is encoded in debug[1] as: iteration * 10 + mode
# mode: 0=NONE, 1=ROLL, 2=PITCH, 3=FILTER

def decode_iter_mode(val):
    """Decode iteration and mode from debug[1]"""
    mode = int(val) % 10
    iteration = int(val) // 10
    return iteration, mode

print("\n" + "="*100)
print("CORRECTLY INTERPRETED DEBUG VALUES")
print("="*100)

sample_times = np.arange(5, 50, 2)
print(f"\n{'Time':>6} {'State':>6} {'Mode':>6} {'Iter':>6} | {'debug2':>8} {'debug3':>8} | {'Meaning debug2':>20} {'Meaning debug3':>20}")
print("-" * 110)

for t in sample_times:
    mask = (df['rel_time'] >= t) & (df['rel_time'] < t + 0.5)
    if mask.any():
        state = int(df.loc[mask, 'debug[0]'].median())
        iter_mode = int(df.loc[mask, 'debug[1]'].median())
        d2 = int(df.loc[mask, 'debug[2]'].median())
        d3 = int(df.loc[mask, 'debug[3]'].median())
        d4 = int(df.loc[mask, 'debug[4]'].median()) if 'debug[4]' in df.columns else 0
        d5 = int(df.loc[mask, 'debug[5]'].median()) if 'debug[5]' in df.columns else 0
        
        iteration, mode = decode_iter_mode(iter_mode)
        
        mode_names = {0: 'NONE', 1: 'ROLL', 2: 'PITCH', 3: 'FILTER'}
        state_names = {0: 'IDLE', 1: 'ARMED', 2: 'DETECT', 3: 'COLLECT', 4: 'SETTLE', 5: 'ANALYZE', 6: 'ADJUST', 7: 'SIGNAL'}
        
        mode_str = mode_names.get(mode, f'?{mode}')
        state_str = state_names.get(state, f'?{state}')
        
        # Interpret debug values based on mode and state
        if mode == 3:  # FILTER mode
            d2_meaning = f"dterm_lpf1={d2}Hz"
            d3_meaning = f"dterm_lpf2={d3}Hz"
        else:  # PID mode
            d2_meaning = f"P={d2}"
            if state == 1:  # ARMED state shows hoverThrottle
                d3_meaning = f"hoverThr={d3}%"
            else:
                d3_meaning = f"D={d3}"
        
        print(f"{t:6.0f} {state_str:>6} {mode_str:>6} {iteration:6d} | {d2:8d} {d3:8d} | {d2_meaning:>20} {d3_meaning:>20}")

# Now extract just the actual D gain values (when not in ARMED state and not in FILTER mode)
print("\n" + "="*100)
print("ACTUAL D GAIN VALUES (filtered for PID mode, non-ARMED states)")
print("="*100)

# Get rows where we're in PID mode (mode != 3) and not ARMED (state != 1)
df['iter_mode'] = df['debug[1]']
df['mode'] = df['iter_mode'] % 10
df['state'] = df['debug[0]']

pid_mode_mask = (df['mode'] != 3) & (df['state'] != 1)
pid_mode_df = df[pid_mode_mask]

if len(pid_mode_df) > 0:
    # Sample at intervals
    for t in sample_times:
        mask = (pid_mode_df['rel_time'] >= t) & (pid_mode_df['rel_time'] < t + 1)
        if mask.any():
            d_vals = pid_mode_df.loc[mask, 'debug[3]']
            f_vals = pid_mode_df.loc[mask, 'debug[5]'] if 'debug[5]' in pid_mode_df.columns else pd.Series([0])
            print(f"t={t:4.0f}s: D={d_vals.median():.0f}, F={f_vals.median():.0f}")
else:
    print("No PID mode data found in non-ARMED states")

# Plot the actual gains over time, with correct interpretation
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# Filter for PID mode only
pid_df = df[df['mode'].isin([1, 2])].copy()  # ROLL or PITCH mode

if len(pid_df) > 0:
    # For D gain, exclude ARMED state samples
    d_df = pid_df[pid_df['state'] != 1].copy()
    
    step = max(1, len(d_df) // 1000)
    
    ax = axes[0, 0]
    ax.plot(d_df['rel_time'].iloc[::step], d_df['debug[2]'].iloc[::step], 'b-', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('P gain')
    ax.set_title('P Gain (PID mode only)')
    ax.grid(True, alpha=0.3)
    
    ax = axes[0, 1]
    ax.plot(d_df['rel_time'].iloc[::step], d_df['debug[3]'].iloc[::step], 'g-', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('D gain')
    ax.set_title('D Gain (PID mode, non-ARMED state)')
    ax.grid(True, alpha=0.3)
    
    if 'debug[4]' in d_df.columns:
        ax = axes[1, 0]
        ax.plot(d_df['rel_time'].iloc[::step], d_df['debug[4]'].iloc[::step], 'orange', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('I gain')
        ax.set_title('I Gain (PID mode only)')
        ax.grid(True, alpha=0.3)
    
    if 'debug[5]' in d_df.columns:
        ax = axes[1, 1]
        ax.plot(d_df['rel_time'].iloc[::step], d_df['debug[5]'].iloc[::step], 'r-', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('F gain')
        ax.set_title('F Gain (PID mode only)')
        ax.grid(True, alpha=0.3)

plt.suptitle('Actual PID Gains - Log 5 (Correctly Interpreted)', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('m4_log5_pid_correct.png', dpi=150)
print(f"\nSaved corrected PID plot to m4_log5_pid_correct.png")

plt.show()
