"""Analyze hover mode debug data for throttle impulses and stuck reason codes."""
import pandas as pd
import numpy as np

# Load data
df = pd.read_csv('btfl_debug_reasoning_6.bbl.csv', skiprows=147, low_memory=False)
df['time_s'] = pd.to_numeric(df['time'], errors='coerce') / 1e6
df['time_rel'] = df['time_s'] - df['time_s'].min()  # Time from log start

print("=== FULL LOG SUMMARY ===")
print(f"Duration: {df['time_rel'].max():.1f}s")
print()

print("=== REASON CODES OVER FULL LOG ===")
print(df['debug[7]'].value_counts().sort_index())
print()

print("=== STATE (debug[0]) OVER FULL LOG ===")
print(df['debug[0]'].value_counts().sort_index())
print()

# Look for transitions in reason code
df['reason_change'] = df['debug[7]'].diff().fillna(0) != 0
transitions = df[df['reason_change']]
print("=== REASON CODE TRANSITIONS ===")
print(f"Number of transitions: {len(transitions)}")
if len(transitions) > 0:
    print(transitions[['time_rel', 'debug[0]', 'debug[7]']].head(30))
print()

# Look for motor/throttle spikes - check motor difference from moving average
df['motor_avg'] = (df['motor[0]'] + df['motor[1]'] + df['motor[2]'] + df['motor[3]']) / 4
df['motor_smooth'] = df['motor_avg'].rolling(50, center=True).mean()
df['motor_delta'] = df['motor_avg'] - df['motor_smooth']

# Find sharp motor increases (impulses)
impulse_threshold = 50  # Motor units above baseline
impulses = df[df['motor_delta'] > impulse_threshold]
print(f"=== MOTOR IMPULSES (delta > {impulse_threshold}) ===")
print(f"Count: {len(impulses)}")
if len(impulses) > 0:
    print("Times of impulses:", impulses['time_rel'].head(20).values)
print()

# Check if setpoint[3] differs from rcCommand[3] - this would indicate autotune throttle manipulation
df['throttle_diff'] = df['setpoint[3]'] - df['rcCommand[3]']
throttle_diffs = df[df['throttle_diff'].abs() > 10]
print("=== THROTTLE SETPOINT vs RC COMMAND DIFFERENCES ===")
print(f"Count of rows with diff > 10: {len(throttle_diffs)}")
if len(throttle_diffs) > 0:
    print(throttle_diffs[['time_rel', 'rcCommand[3]', 'setpoint[3]', 'throttle_diff']].head(20))
print()

# Look at gyro noise - is there 40Hz oscillation?
print("=== GYRO ROLL (gyroADC[0]) STATS ===")
print(df['gyroADC[0]'].describe())
print()

# Check P-term for roll oscillation
print("=== ROLL P-TERM (axisP[0]) STATS ===")
print(df['axisP[0]'].describe())
print()

# Check D-term for roll
print("=== ROLL D-TERM (axisD[0]) STATS ===") 
print(df['axisD[0]'].describe())
print()

# FFT on gyro roll to find 40Hz component
from scipy.fft import rfft, rfftfreq
# Sample rate ~1000Hz (from loop data), use chunk of stable hover data
stable_mask = (df['time_rel'] >= 10) & (df['time_rel'] <= 15)
stable = df[stable_mask]['gyroADC[0]'].values
if len(stable) > 500:
    n = len(stable)
    dt = 0.001  # ~1ms between samples
    yf = np.abs(rfft(stable))
    xf = rfftfreq(n, dt)
    # Find peaks
    top_indices = np.argsort(yf)[-10:][::-1]
    print("=== GYRO ROLL FFT TOP FREQUENCIES ===")
    for i in top_indices:
        if xf[i] > 5:  # Ignore DC and very low freq
            print(f"  {xf[i]:.1f} Hz: amplitude {yf[i]:.1f}")
