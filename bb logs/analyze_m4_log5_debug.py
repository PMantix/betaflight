#!/usr/bin/env python3
"""Analyze debug values in log 5 to see PID changes"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the log
df = pd.read_csv('btfl___milestone_4_log_5.bbl.csv', skiprows=147)

# Calculate relative time
log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6

print(f'Log: {len(df)} samples, {df["rel_time"].max():.1f}s duration')

# Debug columns typically contain autotune state info
print("\n" + "="*80)
print("DEBUG VALUE ANALYSIS")
print("="*80)

# Sample debug values at different times
sample_times = [5, 10, 15, 20, 25, 30, 35, 40, 45]
print("\nDebug values over time:")
print(f"{'Time':>6} {'debug[0]':>10} {'debug[1]':>10} {'debug[2]':>10} {'debug[3]':>10}")
print("-" * 50)

for t in sample_times:
    mask = (df['rel_time'] >= t) & (df['rel_time'] < t + 0.5)
    if mask.any():
        d0 = df.loc[mask, 'debug[0]'].median()
        d1 = df.loc[mask, 'debug[1]'].median()
        d2 = df.loc[mask, 'debug[2]'].median()
        d3 = df.loc[mask, 'debug[3]'].median()
        print(f"{t:6.0f} {d0:10.0f} {d1:10.0f} {d2:10.0f} {d3:10.0f}")

# Check for unique values to understand what they represent
print("\n" + "="*80)
print("UNIQUE DEBUG VALUES (likely autotune state/phase)")
print("="*80)

for i in range(4):
    col = f'debug[{i}]'
    unique_vals = sorted(df[col].unique())
    if len(unique_vals) < 20:
        print(f"{col}: {unique_vals}")
    else:
        print(f"{col}: {len(unique_vals)} unique values, range {min(unique_vals)} to {max(unique_vals)}")

# Plot debug values over time
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

for i, ax in enumerate(axes.flatten()):
    col = f'debug[{i}]'
    # Downsample for plotting
    step = max(1, len(df) // 2000)
    ax.plot(df['rel_time'].iloc[::step], df[col].iloc[::step], alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(col)
    ax.set_title(f'{col} over time')
    ax.grid(True, alpha=0.3)

plt.suptitle('Debug Values - Log 5 (Autotune State)', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('m4_log5_debug.png', dpi=150)
print("\nSaved debug plot to m4_log5_debug.png")

# Try to correlate debug changes with maneuvers
print("\n" + "="*80)
print("DEBUG VALUE CHANGES (potential PID adjustments)")
print("="*80)

# Find where debug[2] changes (likely F value based on range 0-667)
df['d2_change'] = df['debug[2]'].diff().abs() > 10
changes = df[df['d2_change']]
print(f"\nDebug[2] changes at {len(changes)} points:")
for idx, row in changes.head(20).iterrows():
    prev_idx = max(0, idx - 1)
    prev_val = df.loc[prev_idx, 'debug[2]']
    print(f"  t={row['rel_time']:.2f}s: {prev_val:.0f} -> {row['debug[2']:.0f}")

# Check if debug values map to PIDF
print("\n" + "="*80)
print("INTERPRETATION")
print("="*80)
print("Based on ranges:")
print("  debug[0]: 0-7 -> Likely autotune STATE or PHASE")
print("  debug[1]: 0-83 -> Likely P or D gain")  
print("  debug[2]: 0-667 -> Likely F gain (feedforward)")
print("  debug[3]: 0-755 -> Could be I gain, iteration count, or metric")

plt.show()
