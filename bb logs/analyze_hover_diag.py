#!/usr/bin/env python3
"""Analyze hover diagnostic window specifically"""
import pandas as pd
import numpy as np

df = pd.read_csv('btfl_002__newton_1.bbl.csv', skiprows=147)

# Filter to hover diagnostic window (3.4s to 11.4s)
start_us = 3.4 * 1e6
end_us = 11.4 * 1e6

hover = df[(df['time'] >= start_us) & (df['time'] <= end_us)].copy()
print(f'Hover diagnostic window: {len(hover)} samples ({start_us/1e6:.1f}s to {end_us/1e6:.1f}s)')
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

# Show state transitions in this window
print('='*100)
print('STATE TRANSITIONS (3.4s - 11.4s)')
print('='*100)

hover['state_change'] = hover['debug[0]'].diff() != 0
hover['reason_change'] = hover['debug[7]'].diff() != 0
changes = hover[hover['state_change'] | hover['reason_change']].copy()

print(f"{'Time(s)':>10} {'State':>8} {'d[1]':>8} {'d[2]':>8} {'d[3]':>8} {'d[4]':>8} {'d[5]':>8} {'d[6]':>8} {'d[7]':>8} Reason")
print('-'*110)

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

print()
print('='*100)
print('DIAGNOSTIC PHASE ANALYSIS')
print('='*100)

# Look at each diagnostic phase
for phase_code, phase_name in [(1500, 'BASELINE'), (1510, 'ROLL_TEST'), (1520, 'PITCH_TEST'), (1530, 'GYRO_LPF1'), (1540, 'DTERM_LPF1')]:
    phase_data = hover[hover['debug[7]'] == phase_code]
    if len(phase_data) > 0:
        first = phase_data.iloc[0]
        last = phase_data.iloc[-1]
        duration = (last['time'] - first['time']) / 1e6
        base_rms = last['debug[2]'] / 10.0
        curr_rms = last['debug[3]'] / 10.0
        print(f'{phase_name:12}: {len(phase_data):5} samples, {duration:.3f}s, baseRMS={base_rms:.1f}, currRMS={curr_rms:.1f}')

print()
print('='*100)
print('IMPROVEMENTS DETECTED (from d[4], d[5], d[6])')
print('='*100)

# Get the analyzing phase to see improvements
analyzing = hover[hover['debug[7]'] == 1550]  # DIAG_ANALYZING
if len(analyzing) > 0:
    for idx, row in analyzing.iterrows():
        t = row['time'] / 1e6
        roll_imp = int(row['debug[4]'])
        pitch_imp = int(row['debug[5]'])
        filt_imp = int(row['debug[6]'])
        print(f'At {t:.3f}s: Roll={roll_imp}%, Pitch={pitch_imp}%, Filter={filt_imp}%')
else:
    # Maybe it's stored differently - check ADJUST state
    adjusting = hover[hover['debug[0]'] == 6]
    if len(adjusting) > 0:
        first = adjusting.iloc[0]
        t = first['time'] / 1e6
        roll_imp = int(first['debug[4]'])
        pitch_imp = int(first['debug[5]'])
        filt_imp = int(first['debug[6]'])
        print(f'At ADJUST {t:.3f}s: Roll={roll_imp}%, Pitch={pitch_imp}%, Filter={filt_imp}%')

print()
print('='*100)
print('FIX APPLIED')
print('='*100)

# What fix was applied in this window?
for fix_code, fix_name in [(1560, 'ROLL'), (1561, 'PITCH'), (1562, 'GYRO_LPF1'), (1563, 'DTERM_LPF1')]:
    fix_data = hover[hover['debug[7]'] == fix_code]
    if len(fix_data) > 0:
        first = fix_data.iloc[0]
        t = first['time'] / 1e6
        print(f'FIX: {fix_name} at {t:.3f}s')
        print(f'  d[1]={int(first["debug[1]"])} d[2]={int(first["debug[2]"])} d[3]={int(first["debug[3]"])}')
        print(f'  d[4]={int(first["debug[4]"])} d[5]={int(first["debug[5]"])} d[6]={int(first["debug[6]"])}')

print()
print('='*100)
print('RAW DEBUG VALUES AT KEY POINTS')
print('='*100)

# Show actual values at each phase transition for the first diagnostic cycle
cycle1 = hover[(hover['time'] >= 3.4e6) & (hover['time'] <= 11.4e6)]
print(f"\nFirst diagnostic cycle ({len(cycle1)} samples):")

# Get first sample of each phase
for reason_code in [1500, 1510, 1520, 1530, 1540]:
    phase_samples = cycle1[cycle1['debug[7]'] == reason_code]
    if len(phase_samples) > 0:
        # Show first and last sample of this phase
        first = phase_samples.iloc[0]
        last = phase_samples.iloc[-1]
        phase_name = REASON_CODES.get(reason_code, '?')
        print(f"\n{phase_name}:")
        print(f"  Start: t={first['time']/1e6:.3f}s d[2]={int(first['debug[2]'])} d[3]={int(first['debug[3]'])}")
        print(f"  End:   t={last['time']/1e6:.3f}s d[2]={int(last['debug[2]'])} d[3]={int(last['debug[3]'])}")
