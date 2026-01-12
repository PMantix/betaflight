#!/usr/bin/env python3
"""Find when diagnostic phases actually happen"""
import pandas as pd
import numpy as np

df = pd.read_csv('btfl_002__newton_1.bbl.csv', skiprows=147)

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

# Find first occurrence of each diagnostic phase
print('='*100)
print('FIRST OCCURRENCE OF EACH DIAGNOSTIC PHASE')
print('='*100)

for code, name in sorted(REASON_CODES.items()):
    matches = df[df['debug[7]'] == code]
    if len(matches) > 0:
        first_t = matches.iloc[0]['time'] / 1e6
        last_t = matches.iloc[-1]['time'] / 1e6
        print(f'{code:5} {name:20} first={first_t:8.3f}s  last={last_t:8.3f}s  count={len(matches)}')

print()
print('='*100)
print('ALL STATE/REASON TRANSITIONS (first 100)')
print('='*100)

STATE_NAMES = {0:'IDLE', 1:'ARMED', 2:'DETECT', 3:'COLLECT', 4:'SETTLE', 5:'ANALYZE', 6:'ADJUST', 7:'SIGNAL'}

df['state_change'] = df['debug[0]'].diff() != 0
df['reason_change'] = df['debug[7]'].diff() != 0
changes = df[df['state_change'] | df['reason_change']].head(100)

print(f"{'Time(s)':>10} {'State':>8} {'d[1]':>8} {'d[2]':>8} {'d[3]':>8} {'d[4]':>8} {'d[5]':>8} {'d[6]':>8} {'d[7]':>8} Reason")
print('-'*120)

for idx, row in changes.iterrows():
    t = row['time'] / 1e6
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
    
    print(f'{t:10.3f} {state_name:>8} {d1:8} {d2:8} {d3:8} {d4:8} {d5:8} {d6:8} {reason:8} {reason_name}')
