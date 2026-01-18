#!/usr/bin/env python3
"""Analyze Milestone 4 log - PID tuning of roll axis
Focus on F-term adjustments and whether response is actually changing"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the log
df = pd.read_csv('btfl___milestone_4_log_1.bbl.csv', skiprows=147)

# Calculate relative time from log start
log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6

print(f'Log: {len(df)} samples, {df["rel_time"].max():.1f}s duration')
print(f'Log starts at absolute time: {log_start/1e6:.3f}s')
print()

REASON_CODES = {
    1000: 'HOVER_WAITING',
    1001: 'HOVER_MEASURING',
    1500: 'DIAG_BASELINE',
    1510: 'DIAG_ROLL_TEST',
    1520: 'DIAG_PITCH_TEST',
    1530: 'DIAG_GYRO_LPF1',
    1540: 'DIAG_DTERM_LPF1',
    1545: 'DIAG_VERIFY_BL',
    1550: 'DIAG_ANALYZING',
    1560: 'DIAG_FIX_ROLL',
    1561: 'DIAG_FIX_PITCH',
    1562: 'DIAG_FIX_GYRO',
    1563: 'DIAG_FIX_DTERM',
    1570: 'DIAG_NO_IMPROV',
    1580: 'DIAG_TARGET_OK',
    1590: 'DIAG_MAX_ITER',
    2000: 'FILTER_WAITING',
    2001: 'FILTER_COLLECT',
    2110: 'FILT_RES_LPF',
    2210: 'FILT_NOISE_LPF',
    2310: 'FILT_LOW_LPF',
    2400: 'FILT_NOISE_OK',
    2500: 'FILT_NO_CHANGE',
    2600: 'FILT_AT_LIMIT',
    2999: 'FILT_COMPLETE',
    3000: 'PID_WAITING',
    3001: 'PID_COLLECTING',
    3110: 'PID_OVER_P_DN',
    3120: 'PID_OVER_D_UP',
    3210: 'PID_SLUG_P_UP',
    3220: 'PID_SLUG_D_DN',
    3310: 'PID_OSC_D_UP',
    3320: 'PID_OSC_P_DN',
    3410: 'PID_NOISE_D_DN',
    3420: 'PID_DRIFT_I_UP',
    3430: 'PID_BOUNCE_I_DN',
    3440: 'PID_SLOWOSC_I',
    3500: 'PID_GOOD',
    3510: 'PID_LAG_F_UP',
    3520: 'PID_LEAD_F_DN',
    3530: 'PID_PHASE_F_UP',
    3600: 'PID_AT_LIMIT',
    3910: 'ROLL_COMPLETE',
    3920: 'PITCH_COMPLETE',
    3999: 'PID_COMPLETE',
    9000: 'IDLE',
    9100: 'GRACE_PERIOD',
    9200: 'DATA_INVALID',
    9300: 'AT_LIMIT',
}

STATE_NAMES = {0:'IDLE', 1:'ARMED', 2:'DETECT', 3:'COLLECT', 4:'SETTLE', 5:'ANALYZE', 6:'ADJUST', 7:'SIGNAL'}

# First, show all state/reason transitions to understand the flow
print('='*120)
print('FULL STATE/REASON SEQUENCE')
print('='*120)

df['state_change'] = df['debug[0]'].diff() != 0
df['reason_change'] = df['debug[7]'].diff() != 0
transitions = df[df['state_change'] | df['reason_change']]

print(f"{'Time':>8} {'State':>8} {'d[1]':>6} {'d[2]':>6} {'d[3]':>6} {'d[4]':>6} {'d[5]':>6} {'d[6]':>6} {'Reason':>6} ReasonName")
print('-'*120)

for idx, row in transitions.head(100).iterrows():
    t = row['rel_time']
    state = int(row['debug[0]'])
    state_name = STATE_NAMES.get(state, f'{state}?')
    reason = int(row['debug[7]'])
    reason_name = REASON_CODES.get(reason, f'?{reason}')
    
    print(f"{t:8.3f} {state_name:>8} {int(row['debug[1]']):6} {int(row['debug[2]']):6} {int(row['debug[3]']):6} {int(row['debug[4]']):6} {int(row['debug[5]']):6} {int(row['debug[6]']):6} {reason:6} {reason_name}")

# Now find PID tune iterations (reason 3xxx range, specifically 3510 for F-up)
print()
print('='*120)
print('PID TUNE ITERATIONS (F-TERM ADJUSTMENTS)')
print('='*120)

# During PID tune, debug channels are:
# d[1] = iteration * 10 + tuneMode (tuneMode 1=Roll, 2=Pitch)
# d[2] = current P
# d[3] = current D  
# d[4] = current I
# d[5] = current F
# d[6] = score or metric

pid_adjusts = df[(df['debug[0]'] == 6) & (df['debug[7]'] >= 3000) & (df['debug[7]'] < 4000)]

print(f"Found {len(pid_adjusts)} samples in PID ADJUST state")
print()

# Group by iteration (when d[1] changes)
if len(pid_adjusts) > 0:
    pid_adjusts = pid_adjusts.copy()
    pid_adjusts['iter_change'] = pid_adjusts['debug[1]'].diff() != 0
    iter_starts = pid_adjusts[pid_adjusts['iter_change'] | (pid_adjusts.index == pid_adjusts.index[0])]
    
    print(f"{'Time':>8} {'Iter':>5} {'P':>4} {'D':>4} {'I':>4} {'F':>5} {'d[6]':>6} {'Reason':>6} ReasonName")
    print('-'*80)
    
    for idx, row in iter_starts.iterrows():
        t = row['rel_time']
        d1 = int(row['debug[1]'])
        iteration = d1 // 10
        mode = d1 % 10
        
        P = int(row['debug[2]'])
        D = int(row['debug[3]'])
        I = int(row['debug[4]'])
        F = int(row['debug[5]'])
        d6 = int(row['debug[6]'])
        
        reason = int(row['debug[7]'])
        reason_name = REASON_CODES.get(reason, f'?{reason}')
        
        print(f"{t:8.3f} {iteration:5} {P:4} {D:4} {I:4} {F:5} {d6:6} {reason:6} {reason_name}")

# Now let's look at the actual gyro vs setpoint during maneuvers
print()
print('='*120)
print('ROLL MANEUVER ANALYSIS - GYRO VS SETPOINT TRACKING')
print('='*120)

# Find roll maneuvers by looking for high setpointRoll values
if 'setpointRoll' in df.columns and 'gyroADC[0]' in df.columns:
    # Find periods with significant roll setpoint
    df['abs_setpoint_roll'] = df['setpointRoll'].abs()
    maneuver_threshold = 200  # deg/s
    
    # Find starts of maneuvers
    df['in_maneuver'] = df['abs_setpoint_roll'] > maneuver_threshold
    df['maneuver_start'] = df['in_maneuver'] & ~df['in_maneuver'].shift(1).fillna(False)
    
    maneuver_starts = df[df['maneuver_start']]
    print(f"Found {len(maneuver_starts)} roll maneuvers (setpoint > {maneuver_threshold} deg/s)")
    print()
    
    # Analyze first few maneuvers
    maneuver_times = maneuver_starts['rel_time'].values[:10]
    
    for i, start_t in enumerate(maneuver_times):
        # Get 500ms window around maneuver
        window = df[(df['rel_time'] >= start_t - 0.1) & (df['rel_time'] <= start_t + 0.4)]
        
        if len(window) > 0:
            max_setpoint = window['setpointRoll'].abs().max()
            max_gyro = window['gyroADC[0]'].abs().max()
            
            # Calculate lag: find peak times
            setpoint_peak_idx = window['setpointRoll'].abs().idxmax()
            gyro_peak_idx = window['gyroADC[0]'].abs().idxmax()
            
            setpoint_peak_t = window.loc[setpoint_peak_idx, 'rel_time']
            gyro_peak_t = window.loc[gyro_peak_idx, 'rel_time']
            
            lag_ms = (gyro_peak_t - setpoint_peak_t) * 1000
            
            # Get current gains at this time
            state_at_time = df[df['rel_time'] <= start_t].iloc[-1]
            P = int(state_at_time['debug[2]']) if state_at_time['debug[0]'] >= 2 else 0
            D = int(state_at_time['debug[3]']) if state_at_time['debug[0]'] >= 2 else 0
            I = int(state_at_time['debug[4]']) if state_at_time['debug[0]'] >= 2 else 0
            F = int(state_at_time['debug[5]']) if state_at_time['debug[0]'] >= 2 else 0
            
            print(f"Maneuver {i+1} at {start_t:.3f}s: setpoint={max_setpoint:.0f} gyro={max_gyro:.0f} lag={lag_ms:.1f}ms  (P={P} D={D} I={I} F={F})")
else:
    print("setpointRoll or gyroADC[0] columns not found")

# Plot a few maneuvers to visualize
print()
print('='*120)
print('PLOTTING MANEUVERS (saving to file)')
print('='*120)

if 'setpointRoll' in df.columns and 'gyroADC[0]' in df.columns and len(maneuver_starts) > 0:
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    axes = axes.flatten()
    
    for i, start_t in enumerate(maneuver_times[:6]):
        ax = axes[i] if i < 6 else None
        if ax is None:
            break
            
        # Get 400ms window
        window = df[(df['rel_time'] >= start_t - 0.05) & (df['rel_time'] <= start_t + 0.35)]
        
        if len(window) > 0:
            t = (window['rel_time'] - start_t) * 1000  # ms from start
            ax.plot(t, window['setpointRoll'], label='Setpoint', alpha=0.8)
            ax.plot(t, window['gyroADC[0]'], label='Gyro', alpha=0.8)
            ax.axhline(0, color='gray', linestyle='--', alpha=0.3)
            ax.axvline(0, color='gray', linestyle='--', alpha=0.3)
            ax.set_xlabel('Time (ms)')
            ax.set_ylabel('Rate (deg/s)')
            ax.set_title(f'Maneuver at {start_t:.2f}s')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('m4_maneuvers.png', dpi=150)
    print("Saved plot to m4_maneuvers.png")

# Finally, look at the progression of F values
print()
print('='*120)
print('F-TERM PROGRESSION OVER TIME')
print('='*120)

# Get F values from ADJUST state (debug[5] during PID mode)
adjust_states = df[(df['debug[0]'] == 6) & (df['debug[7]'] >= 3000) & (df['debug[7]'] < 4000)]

if len(adjust_states) > 0:
    adjust_states = adjust_states.copy()
    adjust_states['F'] = adjust_states['debug[5]']
    
    # Group by distinct F values
    adjust_states['F_change'] = adjust_states['F'].diff() != 0
    f_changes = adjust_states[adjust_states['F_change'] | (adjust_states.index == adjust_states.index[0])]
    
    print(f"{'Time':>8} {'F':>5} {'Reason':>6} ReasonName")
    print('-'*50)
    
    for idx, row in f_changes.iterrows():
        t = row['rel_time']
        F = int(row['F'])
        reason = int(row['debug[7]'])
        reason_name = REASON_CODES.get(reason, f'?{reason}')
        print(f"{t:8.3f} {F:5} {reason:6} {reason_name}")
