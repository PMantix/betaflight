#!/usr/bin/env python3
"""
Analyze Abtfl_003_short.bbl.csv - Pilot-driven autotune analysis
Uses column index style from older scripts.

Debug channels (DEBUG_AUTOTUNE):
[0] = State (0-9)
[1] = Tune mode (0=none, 1=roll, 2=pitch, 3=filter)  
[2] = Iteration
[3] = Axis (0=roll, 1=pitch, 2=yaw)
[4] = P gain * 10
[5] = D gain * 10
[6] = Score * 100 (or status code)
[7] = Attribution (P/D/I/F bits)
"""

import pandas as pd
import numpy as np
import sys

# State names from autotune_types.h
STATE_NAMES = {
    0: "IDLE",
    1: "ARMED",
    2: "DETECTING",
    3: "COLLECTING",
    4: "SETTLING",
    5: "ANALYZING",
    6: "ADJUSTING",
    7: "SIGNALING",
    8: "COMPLETE",
    9: "ERROR"
}

TUNE_MODE_NAMES = {
    0: "NONE",
    1: "ROLL",
    2: "PITCH",
    3: "FILTER"
}

def main(filename):
    print(f"\n=== Autotune Short Log Analysis - {filename} ===\n")
    
    # First 147 rows are header metadata, row 148 has column names
    df = pd.read_csv(filename, skiprows=147)
    
    print(f"Total samples: {len(df)}")
    print(f"Columns: {list(df.columns)}")
    
    # Check for debug columns
    debug_cols = [c for c in df.columns if 'debug' in c.lower()]
    print(f"Debug columns: {debug_cols}")
    
    # Show first few debug values
    print("\n=== First 10 debug samples ===")
    if debug_cols:
        print(df[debug_cols].head(10).to_string())
    
    # Check unique debug[0] values (states)
    if 'debug[0]' in df.columns:
        states = df['debug[0]'].unique()
        print(f"\n=== Unique debug[0] (state) values: {sorted(states)} ===")
        
        state_counts = df['debug[0]'].value_counts().sort_index()
        for state, count in state_counts.items():
            name = STATE_NAMES.get(int(state), "UNKNOWN")
            print(f"  State {int(state)} ({name}): {count} samples")
    
    # Look at gyro rates to check for maneuvers
    print("\n=== Gyro Rate Summary (degrees/sec) ===")
    gyro_scale = 1.7453292519943295e-8  # From header
    
    for axis, name in [(0, 'Roll'), (1, 'Pitch'), (2, 'Yaw')]:
        col = f'gyroADC[{axis}]'
        if col in df.columns:
            gyro_raw = df[col]
            # Raw values are in degrees/sec already from CSV export
            max_rate = gyro_raw.abs().max()
            mean_rate = gyro_raw.abs().mean()
            print(f"  {name}: max={max_rate:.1f}°/s, mean={mean_rate:.1f}°/s")
    
    # Check throttle
    if 'rcCommand[3]' in df.columns:
        throttle = df['rcCommand[3]']
        print(f"\n=== Throttle: min={throttle.min()}, max={throttle.max()}, mean={throttle.mean():.0f} ===")
    
    # Look for any non-zero debug values
    print("\n=== Non-zero debug analysis ===")
    for i in range(8):
        col = f'debug[{i}]'
        if col in df.columns:
            non_zero = df[df[col] != 0]
            if len(non_zero) > 0:
                print(f"  {col}: {len(non_zero)} non-zero samples, range=[{df[col].min()}, {df[col].max()}]")
            else:
                print(f"  {col}: all zeros")
    
    # Check motor RPM to see if motors were spinning
    print("\n=== Motor/eRPM data ===")
    for i in range(4):
        col = f'eRPM[{i}]'
        if col in df.columns:
            rpm = df[col]
            print(f"  Motor {i}: min={rpm.min()}, max={rpm.max()}, mean={rpm.mean():.0f}")
    
    print("\n=== Conclusion ===")
    if 'debug[0]' in df.columns:
        if (df['debug[0]'] == 0).all():
            print("All debug[0] values are 0 - autotune may not be enabled or wrong debug mode")
            print("Expected: State changes (IDLE=0 -> ARMED=1 -> DETECTING=2 -> etc)")
        else:
            print("Autotune states detected! Analyzing state transitions...\n")
            
            # Find state transitions
            state_changes = df[df['debug[0]'].diff() != 0].copy()
            print("=== State Transitions ===")
            print(f"{'Time':>12} {'State':>6} {'Name':>12} {'d[1]':>6} {'d[2]':>6} {'d[3]':>6} {'d[4]':>6} {'d[7]':>8}")
            print("-" * 70)
            for idx, row in state_changes.iterrows():
                state = int(row['debug[0]'])
                name = STATE_NAMES.get(state, "UNKNOWN")
                print(f"{row['time']:12.0f} {state:6d} {name:>12} {int(row['debug[1]']):6d} "
                      f"{int(row['debug[2]']):6d} {int(row['debug[3]']):6d} {int(row['debug[4]']):6d} "
                      f"{int(row['debug[7]']):8d}")
            
            # Analyze ERROR states specifically
            error_samples = df[df['debug[0]'] == 9]
            if len(error_samples) > 0:
                print(f"\n=== ERROR State Analysis ({len(error_samples)} samples) ===")
                # Check what values are in debug channels during ERROR
                print("Debug values during first ERROR:")
                first_error = error_samples.iloc[0]
                for i in range(8):
                    print(f"  debug[{i}] = {int(first_error[f'debug[{i}]'])}")
                
                # Look at what happened before ERROR
                first_error_idx = error_samples.index[0]
                print(f"\n5 samples before first ERROR (idx {first_error_idx}):")
                before_error = df.loc[max(0, first_error_idx-5):first_error_idx-1]
                print(before_error[['time', 'debug[0]', 'debug[1]', 'debug[2]', 'debug[3]', 'debug[4]', 'debug[7]',
                                    'gyroADC[0]', 'gyroADC[1]', 'rcCommand[3]']].to_string())
            
            # Check for any COLLECTING, ANALYZING, ADJUSTING states
            for state_num, state_name in STATE_NAMES.items():
                if state_num >= 3:  # Post-detection states
                    count = (df['debug[0]'] == state_num).sum()
                    if count > 0:
                        print(f"\n{state_name} state: {count} samples found")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        # Default to short log
        filename = "Abtfl_003_short.bbl.csv"
    else:
        filename = sys.argv[1]
    
    main(filename)
