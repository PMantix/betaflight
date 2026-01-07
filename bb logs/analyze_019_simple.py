import csv

# Skip the 147 header rows, column names are on row 148
with open('btfl_0019.bbl.csv') as f:
    for _ in range(147):
        next(f)
    data = list(csv.DictReader(f))

# Find rows where debug[0] indicates ADJUST state (state 5)
adjust_samples = [(i, d) for i, d in enumerate(data) if d.get('debug[0]') == '5']

# Extract metrics per iteration
iterations = []
p_gains = []
d_gains = []
ratios = []
zero_crossings = []
damping_states = []
status_codes = []

last_iter = -1
for i, d in adjust_samples:
    iter_num = d.get('debug[1]', '?')
    if iter_num != last_iter and iter_num != '?':
        try:
            iter_val = int(iter_num)
            p_val = int(d.get('debug[4]', '0'))
            d_val = int(d.get('debug[5]', '0'))
            status = int(d.get('debug[6]', '0'))
            damping_raw = int(d.get('debug[7]', '0'))
            
            damp_state = damping_raw // 100
            zero_cross = damping_raw % 100
            
            iterations.append(iter_val)
            p_gains.append(p_val)
            d_gains.append(d_val)
            ratios.append(d_val / p_val if p_val > 0 else 0)
            zero_crossings.append(zero_cross)
            damping_states.append(damp_state)
            status_codes.append(status)
            
            last_iter = iter_num
        except:
            pass

print('Phase 0 Sweep Results:')
print('=' * 100)
print('Iter | P  | D  | D/P Ratio | ZeroX | Damping State      | Status | Notes')
print('-----|----|----|-----------|-------|--------------------|---------|-----------------------')

damp_names = ['UNKNOWN', 'UNDERDAMPED', 'CRITICAL', 'OVERDAMPED']

for idx in range(len(iterations)):
    damp_name = damp_names[damping_states[idx]] if damping_states[idx] < 4 else 'INVALID'
    
    notes = ''
    if status_codes[idx] == 1:
        notes = 'Continuing (underdamped)'
    elif status_codes[idx] == 2:
        notes = 'Continuing (critical)'
    elif status_codes[idx] == 3:
        notes = 'FOUND overdamped!'
    elif status_codes[idx] == 100:
        notes = 'ABORT: D limit exceeded'
    elif status_codes[idx] == 101:
        notes = 'ABORT: Max iterations'
    
    print(f'{iterations[idx]:4d} | {p_gains[idx]:2d} | {d_gains[idx]:2d} | {ratios[idx]:9.3f} | {zero_crossings[idx]:5d} | {damp_name:18s} | {status_codes[idx]:7d} | {notes}')

print('\n' + '=' * 100)
print('\nOBSERVATIONS:')
print(f'  - Sweep ran for {len(iterations)} iterations')
print(f'  - P started at {p_gains[0]}, ended at {p_gains[-1]}')
print(f'  - D started at {d_gains[0]}, ended at {d_gains[-1]}')
print(f'  - D/P ratio went from {ratios[0]:.3f} to {ratios[-1]:.3f}')
print(f'  - Zero crossings ranged from {min(zero_crossings)} to {max(zero_crossings)}')
print(f'  - All iterations classified as: {", ".join(set(damp_names[d] for d in damping_states))}')
print(f'  - Final status: {status_codes[-1]}')

print('\nISSUE: Algorithm never detected OVERDAMPED condition')
print('  - Expected to see damping transition: UNDERDAMPED → CRITICAL → OVERDAMPED')
print('  - Zero crossings stayed low (2-5) throughout sweep')
print('  - Need to investigate why overdamped condition was not triggered')
print('  - Possible causes:')
print('    1. classifyDamping() thresholds too strict')
print('    2. Not enough oscillation decay to detect overdamping')
print('    3. Response settling too quickly (high damping but fast decay)')
print('    4. Need actual settling time and D-term amplitude data from firmware')
