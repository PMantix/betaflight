#!/usr/bin/env python3
import pandas as pd
import sys

filename = sys.argv[1] if len(sys.argv) > 1 else "Abtfl_004_short.bbl.csv"
df = pd.read_csv(filename, skiprows=147)

print(f"\n=== SIGNALING State (Wiggle) Analysis ===\n")

# Look at SIGNALING state
signaling = df[df['debug[0]'] == 7]
if len(signaling) > 0:
    print(f"SIGNALING state: {len(signaling)} samples")
    print(f"  Duration: {(signaling['time'].max() - signaling['time'].min())/1000:.0f}ms")
    print(f"\nGyro during SIGNALING:")
    print(f"  Roll:  min={signaling['gyroADC[0]'].min():6.0f}, max={signaling['gyroADC[0]'].max():6.0f}, mean={signaling['gyroADC[0]'].mean():6.1f}")
    print(f"  Pitch: min={signaling['gyroADC[1]'].min():6.0f}, max={signaling['gyroADC[1]'].max():6.0f}, mean={signaling['gyroADC[1]'].mean():6.1f}")
    
    print(f"\nSetpoint during SIGNALING:")
    print(f"  Roll:  min={signaling['setpoint[0]'].min():6.0f}, max={signaling['setpoint[0]'].max():6.0f}")
    print(f"  Pitch: min={signaling['setpoint[1]'].min():6.0f}, max={signaling['setpoint[1]'].max():6.0f}")
    
    print(f"\nMotor outputs during SIGNALING:")
    for i in range(4):
        m = signaling[f'motor[{i}]']
        print(f"  Motor[{i}]: min={m.min():4.0f}, max={m.max():4.0f}, range={m.max()-m.min():4.0f}")
    
    # Check if there's any oscillation pattern (wiggle should be ~10Hz)
    gyro_roll = signaling['gyroADC[0]'].values
    if len(gyro_roll) > 10:
        # Count zero crossings
        zero_crossings = 0
        mean_val = gyro_roll.mean()
        for i in range(1, len(gyro_roll)):
            if (gyro_roll[i-1] - mean_val) * (gyro_roll[i] - mean_val) < 0:
                zero_crossings += 1
        duration_sec = (signaling['time'].max() - signaling['time'].min()) / 1e6
        if duration_sec > 0:
            freq_estimate = zero_crossings / (2 * duration_sec)
            print(f"\n  Estimated oscillation frequency: {freq_estimate:.1f} Hz (wiggle should be ~10Hz)")
else:
    print("No SIGNALING state found!")

# Show state transitions around SIGNALING
print(f"\n=== State Transitions ===")
state_changes = df[df['debug[0]'].diff() != 0]
for idx, row in state_changes.iterrows():
    state = int(row['debug[0]'])
    names = {0:"IDLE", 1:"ARMED", 2:"DETECTING", 3:"COLLECTING", 4:"SETTLING", 
             5:"ANALYZING", 6:"ADJUSTING", 7:"SIGNALING", 8:"COMPLETE", 9:"ABORTED"}
    print(f"  t={row['time']/1e6:8.3f}s: State {state} ({names.get(state, '?')})")
