#!/usr/bin/env python3
"""Analyze autotune detection around specific times"""

import pandas as pd
import numpy as np

# Read the log - header at row 147, data starts at 148
df = pd.read_csv('btfl_004___fail_to_detect_roll.bbl.csv', skiprows=148, low_memory=False)

# Convert time to seconds
df['time_s'] = df['time'].astype(float) / 1e6

# Look at times around 18.77s where false detection occurred
start_time = 18.0
end_time = 20.0

mask = (df['time_s'] >= start_time) & (df['time_s'] <= end_time)
subset = df[mask].copy()

print(f"=== Analysis from {start_time}s to {end_time}s ===\n")

# Extract debug values
subset['state'] = subset['debug[0]'].astype(int)
subset['mode_iter'] = subset['debug[1]'].astype(int)
subset['mode'] = subset['mode_iter'] % 10
subset['iteration'] = subset['mode_iter'] // 10

# RC commands - need to decode
# rcCommand is -500 to 500 for sticks, 1000-2000 for throttle
# getRcDeflection normalizes to -1 to 1
subset['roll_stick'] = subset['rcCommand[0]'].astype(float) / 500.0  # Approximate deflection
subset['pitch_stick'] = subset['rcCommand[1]'].astype(float) / 500.0
subset['yaw_stick'] = subset['rcCommand[2]'].astype(float) / 500.0
subset['throttle'] = subset['rcCommand[3]'].astype(float)

# Gyro rates
subset['roll_rate'] = subset['gyroADC[0]'].astype(float)
subset['pitch_rate'] = subset['gyroADC[1]'].astype(float)
subset['yaw_rate'] = subset['gyroADC[2]'].astype(float)

# Find state transitions
prev_state = None
prev_mode = None

print("State transitions:")
print("-" * 80)
for idx, row in subset.iterrows():
    if row['state'] != prev_state or row['mode'] != prev_mode:
        max_stick = max(abs(row['roll_stick']), abs(row['pitch_stick']), abs(row['yaw_stick']))
        max_rate = max(abs(row['roll_rate']), abs(row['pitch_rate']))
        print(f"t={row['time_s']:.3f}s: state={row['state']}, mode={row['mode']}, "
              f"throttle={row['throttle']:.0f}, max_stick={max_stick:.2f}, max_rate={max_rate:.1f}")
        prev_state = row['state']
        prev_mode = row['mode']

print("\n" + "=" * 80)
print("\nDetailed view around 18.77s:")
print("-" * 80)

# Look at 18.5s to 19.0s in detail
detail_mask = (df['time_s'] >= 18.5) & (df['time_s'] <= 19.0)
detail = df[detail_mask].copy()

detail['state'] = detail['debug[0]'].astype(int)
detail['mode'] = detail['debug[1]'].astype(int) % 10
detail['roll_stick'] = detail['rcCommand[0]'].astype(float) / 500.0
detail['pitch_stick'] = detail['rcCommand[1]'].astype(float) / 500.0
detail['throttle'] = detail['rcCommand[3]'].astype(float)
detail['roll_rate'] = detail['gyroADC[0]'].astype(float)
detail['pitch_rate'] = detail['gyroADC[1]'].astype(float)

# Sample every 50 rows (about every 25ms at 2kHz)
for i in range(0, len(detail), 50):
    row = detail.iloc[i]
    time_s = row['time'].astype(float) / 1e6
    max_stick = max(abs(row['roll_stick']), abs(row['pitch_stick']))
    max_rate = max(abs(row['roll_rate']), abs(row['pitch_rate']))
    sticks_centered = max_stick < 0.5
    print(f"t={time_s:.3f}s: state={row['state']}, mode={row['mode']}, "
          f"thr={row['throttle']:.0f}, roll_stk={row['roll_stick']:.2f}, "
          f"roll_rate={row['roll_rate']:.0f}, centered={sticks_centered}")

# Also look at what values might indicate hover threshold
print("\n" + "=" * 80)
print("\nThrottle analysis:")
print("-" * 80)

# Find hover calibration (state transitions around hover)
all_data = df.copy()
all_data['state'] = all_data['debug[0]'].astype(int)
all_data['throttle'] = all_data['rcCommand[3]'].astype(float)
all_data['time_s'] = all_data['time'].astype(float) / 1e6

# Find when state goes to 1 (ARMED) - this is after hover calibration
armed_mask = all_data['state'] == 1
if armed_mask.any():
    first_armed_idx = armed_mask.idxmax()
    # Look at debug[3] which might show hover throttle
    print(f"First ARMED state at t={all_data.loc[first_armed_idx, 'time_s']:.3f}s")
    print(f"debug[3] (hover throttle?) = {all_data.loc[first_armed_idx, 'debug[3]']}")
