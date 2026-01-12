#!/usr/bin/env python3
"""Analyze the 12.4s area where 9300 appears despite filters at initial values"""
import pandas as pd
import numpy as np

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
    2999: 'FILT_COMPLETE',
    3000: 'PID_WAITING',
    3001: 'PID_COLLECTING',
    9000: 'IDLE',
    9100: 'GRACE_PERIOD',
    9200: 'DATA_INVALID',
    9300: 'AT_LIMIT',
}

STATE_NAMES = {0:'IDLE', 1:'ARMED', 2:'DETECT', 3:'COLLECT', 4:'SETTLE', 5:'ANALYZE', 6:'ADJUST', 7:'SIGNAL'}

# Read the log
df = pd.read_csv('btfl_netwon_2___12p4s.bbl.csv', skiprows=147)

# Calculate relative time from log start
log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6

print(f'Log: {len(df)} samples, {df["rel_time"].max():.1f}s duration')
print(f'Log starts at absolute time: {log_start/1e6:.3f}s')
print()

# Focus on the 12.4s area (say 11s to 14s)
window = df[(df['rel_time'] >= 11.0) & (df['rel_time'] <= 14.0)].copy()
print(f'Window 11-14s: {len(window)} samples')
print()

print('='*140)
print('STATE/REASON TRANSITIONS (11s-14s)')
print('='*140)

window['state_change'] = window['debug[0]'].diff() != 0
window['reason_change'] = window['debug[7]'].diff() != 0
changes = window[window['state_change'] | window['reason_change']]

print(f"{'RelTime':>10} {'State':>8} {'d[1]':>6} {'d[2]':>6} {'d[3]':>6} {'d[4]':>6} {'d[5]':>6} {'d[6]':>6} {'d[7]':>6} Reason")
print('-'*140)

for idx, row in changes.iterrows():
    t = row['rel_time']
    state = int(row['debug[0]'])
    state_name = STATE_NAMES.get(state, '?')
    reason = int(row['debug[7]'])
    reason_name = REASON_CODES.get(reason, f'?{reason}')
    
    d1 = int(row['debug[1]'])
    d2 = int(row['debug[2]'])
    d3 = int(row['debug[3]'])
    d4 = int(row['debug[4]'])
    d5 = int(row['debug[5]'])
    d6 = int(row['debug[6]'])
    
    print(f'{t:10.3f} {state_name:>8} {d1:6} {d2:6} {d3:6} {d4:6} {d5:6} {d6:6} {reason:6} {reason_name}')

# Now specifically look at 9300 occurrences
print()
print('='*140)
print('ALL 9300 (AT_LIMIT) OCCURRENCES')
print('='*140)

at_limit = df[df['debug[7]'] == 9300]
print(f'Total 9300 samples: {len(at_limit)}')
print()

# Group by time to find distinct occurrences
if len(at_limit) > 0:
    # Find first sample of each distinct 9300 period
    at_limit_first = at_limit[at_limit['debug[7]'].diff() != 0]
    if len(at_limit_first) == 0:
        at_limit_first = at_limit.head(1)
    
    print(f"{'RelTime':>10} {'State':>8} {'d[1]':>6} {'d[2]':>6} {'d[3]':>6} {'d[4]':>6} {'d[5]':>6} {'d[6]':>6}")
    print('-'*80)
    
    for idx, row in at_limit.head(20).iterrows():
        t = row['rel_time']
        state = int(row['debug[0]'])
        state_name = STATE_NAMES.get(state, '?')
        
        d1 = int(row['debug[1]'])
        d2 = int(row['debug[2]'])
        d3 = int(row['debug[3]'])
        d4 = int(row['debug[4]'])
        d5 = int(row['debug[5]'])
        d6 = int(row['debug[6]'])
        
        print(f'{t:10.3f} {state_name:>8} {d1:6} {d2:6} {d3:6} {d4:6} {d5:6} {d6:6}')

# Check what's happening just before and during the first 9300
print()
print('='*140)
print('CONTEXT AROUND FIRST 9300')
print('='*140)

if len(at_limit) > 0:
    first_9300_time = at_limit.iloc[0]['rel_time']
    context = df[(df['rel_time'] >= first_9300_time - 1.0) & (df['rel_time'] <= first_9300_time + 0.5)]
    
    context['state_change'] = context['debug[0]'].diff() != 0
    context['reason_change'] = context['debug[7]'].diff() != 0
    changes = context[context['state_change'] | context['reason_change']]
    
    print(f"Context around {first_9300_time:.3f}s:")
    print()
    print(f"{'RelTime':>10} {'State':>8} {'d[1]':>6} {'d[2]':>6} {'d[3]':>6} {'d[4]':>6} {'d[5]':>6} {'d[6]':>6} {'d[7]':>6} Reason")
    print('-'*140)
    
    for idx, row in changes.iterrows():
        t = row['rel_time']
        state = int(row['debug[0]'])
        state_name = STATE_NAMES.get(state, '?')
        reason = int(row['debug[7]'])
        reason_name = REASON_CODES.get(reason, f'?{reason}')
        
        d1 = int(row['debug[1]'])
        d2 = int(row['debug[2]'])
        d3 = int(row['debug[3]'])
        d4 = int(row['debug[4]'])
        d5 = int(row['debug[5]'])
        d6 = int(row['debug[6]'])
        
        marker = " <-- 9300" if reason == 9300 else ""
        print(f'{t:10.3f} {state_name:>8} {d1:6} {d2:6} {d3:6} {d4:6} {d5:6} {d6:6} {reason:6} {reason_name}{marker}')
