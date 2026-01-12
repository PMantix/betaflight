#!/usr/bin/env python3
"""Analyze all diagnostic cycles to see Newton convergence"""
import pandas as pd
import numpy as np

df = pd.read_csv('btfl_002__newton_1.bbl.csv', skiprows=147)

# Find all cycle starts (DIAG_BASELINE = 1500)
baseline_starts = df[df['debug[7]'] == 1500].copy()
baseline_starts = baseline_starts.drop_duplicates(subset=['debug[1]'])  # Unique iterations

print('='*120)
print('NEWTON METHOD CONVERGENCE ACROSS ALL CYCLES')
print('='*120)

print(f"\n{'Cycle':>5} {'Start(s)':>10} {'BaseRMS':>10} {'RollImp':>10} {'PitchImp':>10} {'FiltImp':>10} {'Fixes':>30}")
print('-'*120)

cycle_num = 0
for idx, baseline_row in baseline_starts.iterrows():
    cycle_num += 1
    start_time = baseline_row['time'] / 1e6
    
    # Find next baseline or end of log
    next_baselines = baseline_starts[baseline_starts['time'] > baseline_row['time']]
    if len(next_baselines) > 0:
        end_time = next_baselines.iloc[0]['time'] / 1e6
    else:
        end_time = df['time'].max() / 1e6
    
    # Get all data for this cycle
    cycle_data = df[(df['time'] >= baseline_row['time']) & (df['time'] < end_time * 1e6)]
    
    # Find the baseRMS for this cycle (from ROLL_TEST phase)
    roll_test = cycle_data[cycle_data['debug[7]'] == 1510]
    if len(roll_test) > 0:
        base_rms = int(roll_test.iloc[0]['debug[2]']) / 10.0
    else:
        base_rms = 0
    
    # Find improvements (from FIX phases)
    fix_phases = cycle_data[cycle_data['debug[7]'].isin([1560, 1561, 1562, 1563])]
    if len(fix_phases) > 0:
        first_fix = fix_phases.iloc[0]
        roll_imp = int(first_fix['debug[4]'])
        pitch_imp = int(first_fix['debug[5]'])
        filt_imp = int(first_fix['debug[6]'])
    else:
        roll_imp = 0
        pitch_imp = 0
        filt_imp = 0
    
    # Which fixes were applied?
    fixes = []
    if 1560 in cycle_data['debug[7]'].values:
        fixes.append('ROLL')
    if 1561 in cycle_data['debug[7]'].values:
        fixes.append('PITCH')
    if 1562 in cycle_data['debug[7]'].values:
        fixes.append('GYRO')
    if 1563 in cycle_data['debug[7]'].values:
        fixes.append('DTERM')
    
    fixes_str = ','.join(fixes) if fixes else 'none'
    
    print(f'{cycle_num:5} {start_time:10.3f} {base_rms:10.1f} {roll_imp:10}% {pitch_imp:10}% {filt_imp:10}% {fixes_str:>30}')

print()
print('='*120)
print('RMS CONVERGENCE OVER TIME')
print('='*120)

# Track baseRMS over cycles
cycles = []
for idx, baseline_row in baseline_starts.iterrows():
    start_time = baseline_row['time'] / 1e6
    
    # Find next baseline or end of log
    next_baselines = baseline_starts[baseline_starts['time'] > baseline_row['time']]
    if len(next_baselines) > 0:
        end_time = next_baselines.iloc[0]['time'] / 1e6
    else:
        end_time = df['time'].max() / 1e6
    
    cycle_data = df[(df['time'] >= baseline_row['time']) & (df['time'] < end_time * 1e6)]
    
    roll_test = cycle_data[cycle_data['debug[7]'] == 1510]
    if len(roll_test) > 0:
        base_rms = int(roll_test.iloc[0]['debug[2]']) / 10.0
        cycles.append({'time': start_time, 'baseRMS': base_rms})

if len(cycles) > 1:
    first_rms = cycles[0]['baseRMS']
    last_rms = cycles[-1]['baseRMS']
    if first_rms > 0:
        overall_improvement = 100.0 * (first_rms - last_rms) / first_rms
    else:
        overall_improvement = 0
    
    print(f'\nFirst cycle baseRMS: {first_rms:.1f}')
    print(f'Last cycle baseRMS:  {last_rms:.1f}')
    print(f'Overall improvement: {overall_improvement:.1f}%')
    
    print(f'\nBaseRMS progression: ', end='')
    print(' → '.join([f'{c["baseRMS"]:.0f}' for c in cycles]))
