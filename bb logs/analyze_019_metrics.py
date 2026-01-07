import csv
import math

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

last_iter = -1
for i, d in adjust_samples:
    iter_num = d.get('debug[1]', '?')
    if iter_num != last_iter and iter_num != '?':
        try:
            iter_val = int(iter_num)
            p_val = int(d.get('debug[4]', '0'))
            d_val = int(d.get('debug[5]', '0'))
            damping_raw = int(d.get('debug[7]', '0'))
            
            damp_state = damping_raw // 100
            zero_cross = damping_raw % 100
            
            iterations.append(iter_val)
            p_gains.append(p_val)
            d_gains.append(d_val)
            ratios.append(d_val / p_val if p_val > 0 else 0)
            zero_crossings.append(zero_cross)
            damping_states.append(damp_state)
            
            last_iter = iter_num
        except:
            pass

# Find all the COLLECT state samples to extract gyro and dterm data
collect_samples = [d for d in data if d.get('debug[0]') == '2']

# Group by iteration and analyze response characteristics
print('Analyzing response characteristics per iteration...\n')
print('Iter | P  | D  | D/P   | ZeroX | DampSt | Rise Time (ms) | Overshoot (%) | Notes')
print('-----|----|----|-------|-------|--------|----------------|---------------|-------')

for idx, iter_val in enumerate(iterations):
    # Find COLLECT samples for this iteration
    iter_samples = [d for d in collect_samples if d.get('debug[1]') == str(iter_val)]
    
    if len(iter_samples) > 50:
        # Extract gyro and setpoint to analyze response
        gyro_vals = [float(d.get('gyroADC[0]', '0')) for d in iter_samples]
        setpoint_vals = [float(d.get('rcCommand[0]', '0')) for d in iter_samples]
        dterm_vals = [float(d.get('axisD[0]', '0')) for d in iter_samples]
        
        # Find step start (where setpoint changes significantly)
        setpoint_changes = [abs(setpoint_vals[i] - setpoint_vals[i-1]) for i in range(1, len(setpoint_vals))]
        step_indices = [i for i, change in enumerate(setpoint_changes) if change > 50]
        
        if len(step_indices) > 0:
            step_idx = step_indices[0]
            response_window = 100  # samples
            
            if step_idx + response_window < len(gyro_vals):
                gyro_response = gyro_vals[step_idx:step_idx+response_window]
                setpoint = setpoint_vals[step_idx + 10]  # steady setpoint after step
                
                # Calculate rise time (10% to 90% of step)
                initial = gyro_vals[step_idx]
                step_size = setpoint - initial
                if abs(step_size) > 10:
                    target_10 = initial + 0.1 * step_size
                    target_90 = initial + 0.9 * step_size
                    
                    rise_start = None
                    rise_end = None
                    for i, val in enumerate(gyro_response):
                        if rise_start is None and abs(val - target_10) < abs(step_size) * 0.1:
                            rise_start = i
                        if rise_end is None and abs(val - target_90) < abs(step_size) * 0.1:
                            rise_end = i
                            break
                    
                    rise_time = (rise_end - rise_start) * 0.5 if (rise_start and rise_end) else 0  # ms (500us per sample)
                    
                    # Calculate overshoot
                    peak = max(gyro_response) if step_size > 0 else min(gyro_response)
                    overshoot = abs((peak - setpoint) / step_size * 100) if step_size != 0 else 0
                    
                    # D-term RMS amplitude
                    dterm_response = dterm_vals[step_idx:step_idx+response_window]
                    dterm_rms = math.sqrt(sum(x*x for x in dterm_response) / len(dterm_response))
                    
                    notes = ''
                    if overshoot > 30:
                        notes = 'High overshoot'
                    elif overshoot < 5 and rise_time > 15:
                        notes = 'Sluggish'
                    elif 5 < overshoot < 15 and 8 < rise_time < 15:
                        notes = 'Good response'
                    
                    print(f'{iter_val:4d} | {p_gains[idx]:2d} | {d_gains[idx]:2d} | {ratios[idx]:.3f} | {zero_crossings[idx]:5d} | {damping_states[idx]:6d} | {rise_time:14.1f} | {overshoot:13.1f} | {notes}')

print('\n\nNOTE: Algorithm stopped at iteration 10 without finding OVERDAMPED')
print('All iterations classified as UNDERDAMPED with 2-5 zero crossings')
print('Need to check why overdamped condition was never detected despite D reaching 1.7x P')
