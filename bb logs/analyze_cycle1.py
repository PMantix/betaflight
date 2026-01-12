#!/usr/bin/env python3
"""Analyze first diagnostic cycle in detail"""
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

STATE_NAMES = {0:'IDLE', 1:'ARMED', 2:'DETECT', 3:'COLLECT', 4:'SETTLE', 5:'ANALYZE', 6:'ADJUST', 7:'SIGNAL'}

print('='*120)
print('FIRST DIAGNOSTIC CYCLE: 11.5s to 18s')
print('='*120)

# Filter to first diagnostic cycle
cycle1 = df[(df['time'] >= 11.5e6) & (df['time'] <= 18e6)].copy()
print(f'Samples in window: {len(cycle1)}')
print()

# Show all state/reason transitions
cycle1['state_change'] = cycle1['debug[0]'].diff() != 0
cycle1['reason_change'] = cycle1['debug[7]'].diff() != 0
changes = cycle1[cycle1['state_change'] | cycle1['reason_change']]

print(f"{'Time(s)':>10} {'State':>8} {'Phase/It':>8} {'BaseRMS':>8} {'CurrRMS':>8} {'RollImp':>8} {'PitchImp':>8} {'FiltImp':>8} Reason")
print('-'*120)

for idx, row in changes.iterrows():
    t = row['time'] / 1e6
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
print('='*120)
print('INTERPRETATION OF FIRST CYCLE:')
print('='*120)
print('''
Debug channel encoding:
  d[0] = state (IDLE/ARMED/DETECT/COLLECT/SETTLE/ANALYZE/ADJUST/SIGNAL)
  d[1] = phase*10 + iteration  (phase: 1=BASELINE,2=ROLL,3=PITCH,4=GYRO,5=DTERM,7=FIX)
  d[2] = baseRMS * 10 
  d[3] = currRMS * 10
  d[4] = roll improvement %
  d[5] = pitch improvement %
  d[6] = filter improvement %
  d[7] = reason code

Expected flow for diagnostic cycle:
  1. DIAG_BASELINE (1500): Measure baseline noise, store in baseRMS
  2. DIAG_ROLL_TEST (1510): Apply roll impulse, measure currRMS
  3. DIAG_PITCH_TEST (1520): Apply pitch impulse, measure currRMS  
  4. DIAG_GYRO_LPF1 (1530): Tighten gyro filter, measure currRMS
  5. DIAG_DTERM_LPF1 (1540): Tighten D-term filter, measure currRMS
  6. Calculate improvements and queue fixes
  7. Apply each fix sequentially (FIX_ROLL, FIX_PITCH, FIX_GYRO, FIX_DTERM)
''')

# Calculate actual improvements from the measurements
print('='*120)
print('NEWTON METHOD ANALYSIS - FIRST CYCLE')
print('='*120)

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
        print(f'{test_name:12}: baseRMS={base_rms:6.1f} currRMS={curr_rms:6.1f} improvement={improvement:6.1f}%')

print()
print('='*120)
print('FIXES APPLIED AND THEIR IMPACT')
print('='*120)

# Show each fix phase
for fix_code, fix_name in [(1560, 'FIX_ROLL'), (1561, 'FIX_PITCH'), (1562, 'FIX_GYRO'), (1563, 'FIX_DTERM')]:
    fix_data = cycle1[cycle1['debug[7]'] == fix_code]
    if len(fix_data) > 0:
        first = fix_data.iloc[0]
        t = first['time'] / 1e6
        roll_imp = int(first['debug[4]'])
        pitch_imp = int(first['debug[5]'])
        filt_imp = int(first['debug[6]'])
        curr_rms = int(first['debug[3]']) / 10.0
        print(f'{fix_name:12} at {t:.3f}s: currRMS={curr_rms:6.1f}, improvements: Roll={roll_imp}% Pitch={pitch_imp}% Filter={filt_imp}%')

# Check what the baseline RMS was for calculation
baseline = cycle1[cycle1['debug[7]'] == 1500]
if len(baseline) > 0:
    # At baseline start, d[2]=0, d[3]=0 - baseline not measured yet
    # At baseline end, d[2] should have baseRMS
    last_baseline = baseline.iloc[-1]
    print(f'\nBaseline end: d[2]={int(last_baseline["debug[2]"])}, d[3]={int(last_baseline["debug[3]"])}')
    
# Show roll test end values
roll_test = cycle1[cycle1['debug[7]'] == 1510]
if len(roll_test) > 0:
    last_roll = roll_test.iloc[-1]
    print(f'Roll test end: d[2]={int(last_roll["debug[2]"])}, d[3]={int(last_roll["debug[3]"])}')
    print(f'  baseRMS={int(last_roll["debug[2]"])/10:.1f}, currRMS={int(last_roll["debug[3]"])/10:.1f}')
