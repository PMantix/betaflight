#!/usr/bin/env python3
"""Analyze hover diagnostic window with CORRECT relative time"""
import pandas as pd
import numpy as np

df = pd.read_csv('btfl_002__newton_1.bbl.csv', skiprows=147)

# Calculate relative time from log start
log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6  # in seconds

print(f'Log starts at absolute time: {log_start/1e6:.3f}s')
print(f'Relative time range: {df["rel_time"].min():.3f}s to {df["rel_time"].max():.3f}s')
print()

REASON_CODES = {
    1000: 'HOVER_WAITING',
    1001: 'HOVER_MEASURING',
    1500: 'DIAG_BASELINE',
    1510: 'DIAG_ROLL_TEST',
    1520: 'DIAG_PITCH_TEST',
    1530: 'DIAG_GYRO_LPF1',
    1540: 'DIAG_DTERM_LPF1',
    1550: 'DIAG_ANALYZING',
    1560: 'DIAG_FIX_ROLL',
    1561: 'DIAG_FIX_PITCH',
    1562: 'DIAG_FIX_GYRO',
    1563: 'DIAG_FIX_DTERM',
    1570: 'DIAG_NO_IMPROV',
    1580: 'DIAG_TARGET_OK',
    1590: 'DIAG_MAX_ITER',
    9100: 'GRACE_PERIOD',
    9300: 'AT_LIMIT',
}

STATE_NAMES = {0:'IDLE', 1:'ARMED', 2:'DETECT', 3:'COLLECT', 4:'SETTLE', 5:'ANALYZE', 6:'ADJUST', 7:'SIGNAL'}

# Find first occurrence of each diagnostic phase (using relative time)
print('='*100)
print('DIAGNOSTIC PHASES (relative time from log start)')
print('='*100)

for code, name in sorted(REASON_CODES.items()):
    matches = df[df['debug[7]'] == code]
    if len(matches) > 0:
        first_t = matches.iloc[0]['rel_time']
        last_t = matches.iloc[-1]['rel_time']
        print(f'{code:5} {name:20} first={first_t:8.3f}s  last={last_t:8.3f}s  count={len(matches)}')

print()
print('='*100)
print('FIRST DIAGNOSTIC CYCLE (3.4s to 11s relative time)')
print('='*100)

# Filter to hover diagnostic window (3.4s to 11s RELATIVE time)
cycle1 = df[(df['rel_time'] >= 3.4) & (df['rel_time'] <= 11.0)].copy()
print(f'Samples in window: {len(cycle1)}')
print()

# Show state/reason transitions
cycle1['state_change'] = cycle1['debug[0]'].diff() != 0
cycle1['reason_change'] = cycle1['debug[7]'].diff() != 0
changes = cycle1[cycle1['state_change'] | cycle1['reason_change']]

print(f"{'RelTime':>10} {'State':>8} {'Phase.It':>8} {'BaseRMS':>8} {'CurrRMS':>8} {'RollImp':>8} {'PitchImp':>8} {'FiltImp':>8} Reason")
print('-'*110)

for idx, row in changes.iterrows():
    t = row['rel_time']
    state = int(row['debug[0]'])
    state_name = STATE_NAMES.get(state, '?')
    reason = int(row['debug[7]'])
    reason_name = REASON_CODES.get(reason, f'?{reason}')
    
    # d[1] = phase*10 + iteration
    d1 = int(row['debug[1]'])
    phase = d1 // 10
    iteration = d1 % 10
    phase_str = f'{phase}.{iteration}'
    
    # d[2] = baseRMS*10, d[3] = currRMS*10
    base_rms = int(row['debug[2]']) / 10.0
    curr_rms = int(row['debug[3]']) / 10.0
    
    # d[4,5,6] = improvements in %
    roll_imp = int(row['debug[4]'])
    pitch_imp = int(row['debug[5]'])
    filt_imp = int(row['debug[6]'])
    
    print(f'{t:10.3f} {state_name:>8} {phase_str:>8} {base_rms:8.1f} {curr_rms:8.1f} {roll_imp:8}% {pitch_imp:8}% {filt_imp:8}% {reason_name}')

print()
print('='*100)
print('MEASUREMENT ANALYSIS - FIRST CYCLE')
print('='*100)

# Find the measurements from each test phase
for test_code, test_name in [(1510, 'ROLL_TEST'), (1520, 'PITCH_TEST'), (1530, 'GYRO_LPF1'), (1540, 'DTERM_LPF1')]:
    test_data = cycle1[cycle1['debug[7]'] == test_code]
    if len(test_data) > 0:
        last = test_data.iloc[-1]
        base_rms = int(last['debug[2]']) / 10.0
        curr_rms = int(last['debug[3]']) / 10.0
        if base_rms > 0:
            improvement = 100.0 * (base_rms - curr_rms) / base_rms
        else:
            improvement = 0
        print(f'{test_name:12}: baseRMS={base_rms:6.1f} currRMS={curr_rms:6.1f} calculated_improvement={improvement:6.1f}%')

print()
print('='*100)
print('FIXES APPLIED')
print('='*100)

for fix_code, fix_name in [(1560, 'FIX_ROLL'), (1561, 'FIX_PITCH'), (1562, 'FIX_GYRO'), (1563, 'FIX_DTERM')]:
    fix_data = cycle1[cycle1['debug[7]'] == fix_code]
    if len(fix_data) > 0:
        first = fix_data.iloc[0]
        t = first['rel_time']
        roll_imp = int(first['debug[4]'])
        pitch_imp = int(first['debug[5]'])
        filt_imp = int(first['debug[6]'])
        curr_rms = int(first['debug[3]']) / 10.0
        print(f'{fix_name:12} at {t:.3f}s: currRMS={curr_rms:6.1f}, stored improvements: Roll={roll_imp}% Pitch={pitch_imp}% Filter={filt_imp}%')
