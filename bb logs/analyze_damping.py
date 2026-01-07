import csv

# Debug channel layout:
# debug[0]: State (0=IDLE, 1=WAIT_STAB, 2=COLLECT, 3=BUFFER, 4=TRIGGER, 5=ADJUST, 6=VALIDATE, 7=APPLY, 8=COMPLETE)
# debug[1]: Global iteration
# debug[2]: Phase (0-3)
# debug[3]: Phase iteration
# debug[4]: Current P gain
# debug[5]: Current D gain
# debug[6]: Status code
# debug[7]: (dampingState × 100) + zeroCrossings

# Skip the 147 header rows, column names are on row 148
with open('btfl_0019.bbl.csv') as f:
    for _ in range(147):
        next(f)
    data = list(csv.DictReader(f))

# Find rows where debug[0] indicates ADJUST state (state 5) - this is where classification happens
adjust_samples = [(i, d) for i, d in enumerate(data) if d.get('debug[0]') == '5']

print('Phase 0 Sweep Analysis from debug channels (ADJUST state):')
print('Iter | Phase | PhaseIter | P  | D  | D/P Ratio | Status | Damping State      | Zero X')
print('-----|-------|-----------|----|----|-----------|--------|--------------------|---------')

last_iter = -1
for i, d in adjust_samples:
    iter_num = d.get('debug[1]', '?')
    if iter_num != last_iter and iter_num != '?':
        phase = d.get('debug[2]', '?')
        phase_iter = d.get('debug[3]', '?')
        p_gain = d.get('debug[4]', '?')
        d_gain = d.get('debug[5]', '?')
        status = d.get('debug[6]', '?')
        damping_raw = d.get('debug[7]', '?')
        
        # Calculate ratio
        try:
            ratio = float(d_gain) / float(p_gain)
            ratio_str = f'{ratio:.3f}'
        except:
            ratio_str = '?'
        
        # Decode damping: (state * 100) + zero_crossings
        if damping_raw != '?':
            damp_val = int(damping_raw)
            damp_state = damp_val // 100
            zero_cross = damp_val % 100
            damp_names = ['UNKNOWN', 'UNDERDAMP', 'CRITICAL', 'OVERDAMP']
            damp_str = damp_names[damp_state] if damp_state < 4 else '?'
        else:
            damp_str = '?'
            zero_cross = '?'
        
        print(f'{iter_num:4s} | {phase:5s} | {phase_iter:9s} | {p_gain:2s} | {d_gain:2s} | {ratio_str:9s} | {status:6s} | {damp_str:18s} | {str(zero_cross):7s}')
        last_iter = iter_num

print('\n\nKey Status Codes:')
print('  1 = Underdamped (continuing sweep)')
print('  2 = Critically damped (continuing sweep)')
print('  3 = Overdamped (found!)')
print('  50 = Completion marker')
print('  100 = Phase 1 sweep failed (hit D limit)')
print('  101 = Max iterations without overdamped')
print('  112 = Noise warning')
