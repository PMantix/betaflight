#!/usr/bin/env python3
"""Analyze PID value changes in log 5"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the log
df = pd.read_csv('btfl___milestone_4_log_5.bbl.csv', skiprows=147)

# Calculate relative time
log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6

print(f'Log: {len(df)} samples, {df["rel_time"].max():.1f}s duration')

# Debug column mapping (from autotune.c):
# debug[0] = state (0-7)
# debug[1] = iteration info (iteration*10 + tuneMode)
# debug[2] = P gain (or hover throttle before calibrated)
# debug[3] = D gain
# debug[4] = I gain
# debug[5] = F gain
# debug[6] = overshoot * 10
# debug[7] = reason code

print("\n" + "="*80)
print("PID VALUE PROGRESSION")
print("="*80)

# Check which debug columns exist
debug_cols = [c for c in df.columns if 'debug' in c.lower()]
print(f"Available debug columns: {debug_cols}")

# Sample at regular intervals
sample_times = np.arange(5, 50, 2)
print(f"\n{'Time':>6} {'State':>6} {'Iter':>6} {'P':>6} {'D':>6} {'I':>6} {'F':>6} {'Ovsh':>6} {'Reason':>8}")
print("-" * 70)

for t in sample_times:
    mask = (df['rel_time'] >= t) & (df['rel_time'] < t + 0.5)
    if mask.any():
        state = df.loc[mask, 'debug[0]'].median() if 'debug[0]' in df.columns else np.nan
        iter_info = df.loc[mask, 'debug[1]'].median() if 'debug[1]' in df.columns else np.nan
        p = df.loc[mask, 'debug[2]'].median() if 'debug[2]' in df.columns else np.nan
        d = df.loc[mask, 'debug[3]'].median() if 'debug[3]' in df.columns else np.nan
        i = df.loc[mask, 'debug[4]'].median() if 'debug[4]' in df.columns else np.nan
        f = df.loc[mask, 'debug[5]'].median() if 'debug[5]' in df.columns else np.nan
        ovsh = df.loc[mask, 'debug[6]'].median() / 10 if 'debug[6]' in df.columns else np.nan
        reason = df.loc[mask, 'debug[7]'].median() if 'debug[7]' in df.columns else np.nan
        print(f"{t:6.0f} {state:6.0f} {iter_info:6.0f} {p:6.0f} {d:6.0f} {i:6.0f} {f:6.0f} {ovsh:6.1f} {reason:8.0f}")

# Plot PID evolution over time
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# Downsample for plotting
step = max(1, len(df) // 2000)

if 'debug[2]' in df.columns:
    ax = axes[0, 0]
    ax.plot(df['rel_time'].iloc[::step], df['debug[2]'].iloc[::step], 'b-', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('P gain')
    ax.set_title('P Gain Evolution')
    ax.grid(True, alpha=0.3)

if 'debug[3]' in df.columns:
    ax = axes[0, 1]
    ax.plot(df['rel_time'].iloc[::step], df['debug[3]'].iloc[::step], 'g-', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('D gain')
    ax.set_title('D Gain Evolution')
    ax.grid(True, alpha=0.3)

if 'debug[4]' in df.columns:
    ax = axes[1, 0]
    ax.plot(df['rel_time'].iloc[::step], df['debug[4]'].iloc[::step], 'orange', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('I gain')
    ax.set_title('I Gain Evolution')
    ax.grid(True, alpha=0.3)

if 'debug[5]' in df.columns:
    ax = axes[1, 1]
    ax.plot(df['rel_time'].iloc[::step], df['debug[5]'].iloc[::step], 'r-', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('F gain')
    ax.set_title('F Gain (Feedforward) Evolution')
    ax.grid(True, alpha=0.3)

plt.suptitle('PID Gains Over Time - Log 5', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('m4_log5_pid_evolution.png', dpi=150)
print("\nSaved PID evolution plot to m4_log5_pid_evolution.png")

# Also check reason codes
if 'debug[7]' in df.columns:
    print("\n" + "="*80)
    print("REASON CODES (what autotune is deciding)")
    print("="*80)
    reasons = df['debug[7]'].value_counts().sort_index()
    print(reasons)

plt.show()
