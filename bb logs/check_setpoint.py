#!/usr/bin/env python3
import pandas as pd
import sys

filename = sys.argv[1] if len(sys.argv) > 1 else "Abtfl_004_short.bbl.csv"
df = pd.read_csv(filename, skiprows=147)

print(f"\n=== Setpoint/Gyro Analysis during Autotune States ===\n")

# Look at DETECTING state periods
detecting = df[df['debug[0]'] == 2]
if len(detecting) > 0:
    print(f"DETECTING state: {len(detecting)} samples")
    print(f"  setpoint[0] range: {detecting['setpoint[0]'].min()} to {detecting['setpoint[0]'].max()}")
    print(f"  gyroADC[0] range: {detecting['gyroADC[0]'].min()} to {detecting['gyroADC[0]'].max()}")

# Look at COLLECTING state
collecting = df[df['debug[0]'] == 3]
if len(collecting) > 0:
    print(f"\nCOLLECTING state: {len(collecting)} samples")
    print(f"  setpoint[0] range: {collecting['setpoint[0]'].min()} to {collecting['setpoint[0]'].max()}")
    print(f"  gyroADC[0] range: {collecting['gyroADC[0]'].min()} to {collecting['gyroADC[0]'].max()}")

# Look at SETTLING state
settling = df[df['debug[0]'] == 4]
if len(settling) > 0:
    print(f"\nSETTLING state: {len(settling)} samples")
    print(f"  setpoint[0] range: {settling['setpoint[0]'].min()} to {settling['setpoint[0]'].max()}")
    print(f"  gyroADC[0] range: {settling['gyroADC[0]'].min()} to {settling['gyroADC[0]'].max()}")

# Calculate what noise RMS would be in settling
if len(settling) > 0:
    gyro_settling = settling['gyroADC[0]']
    rms = (gyro_settling ** 2).mean() ** 0.5
    print(f"\n  Gyro RMS in SETTLING: {rms:.1f} deg/s (this should be the noise measurement)")

# Look at what autotune is reporting
print(f"\n=== Autotune Debug Values ===")
analyzing = df[df['debug[0]'] == 5]
if len(analyzing) > 0:
    sample = analyzing.iloc[0]
    print(f"  debug[5] (overshoot x10): {sample['debug[5]']} -> {sample['debug[5]']/10:.1f}%")
    print(f"  debug[6] (noise x10): {sample['debug[6]']} -> {sample['debug[6]']/10:.1f} deg/s")
    print(f"  debug[7] (score x100): {sample['debug[7]']} -> {sample['debug[7]']/100:.1f}")
