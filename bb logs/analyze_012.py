import csv

print("=== Milestone 3 Alpha Autotune Analysis - btfl_012.bbl.csv ===\n")

STATE_NAMES = {
    '0': 'IDLE',
    '1': 'SETUP', 
    '2': 'WAIT_STABLE',
    '3': 'EXCITE',
    '4': 'MEASURE',
    '5': 'ANALYZE',
    '6': 'ADJUST',
    '7': 'COMPLETE',
    '8': 'ABORTED'
}

PHASE_NAMES = {
    '0': 'Phase 1: Establish Ratio',
    '1': 'Phase 2: Scale Up',
    '2': 'Phase 3: Fine-tune',
    '3': 'Complete'
}

with open('btfl_012.bbl.csv') as f:
    # Skip header lines (148 lines based on other scripts)
    for _ in range(148):
        next(f)
    
    reader = csv.reader(f)
    
    prev_state = None
    iteration = 0
    state_5_count = 0
    state_6_count = 0
    
    # Track state durations
    state_entries = {}
    state_durations = {str(i): [] for i in range(9)}
    
    # Track phase progression
    phases_seen = set()
    
    for row in reader:
        if len(row) <= 40:
            continue
        
        time_us = int(row[1])
        state = row[33]      # debug[0] = state
        debug1 = row[34]     # debug[1]
        debug2 = row[35]     # debug[2]
        debug3 = row[36]     # debug[3]
        debug4 = row[37]     # debug[4]
        debug5 = row[38]     # debug[5]
        debug6 = row[39]     # debug[6]
        debug7 = row[40]     # debug[7]
        
        # Track state entry times for duration calculation
        if state != prev_state and state in STATE_NAMES:
            if prev_state in state_entries:
                duration_us = time_us - state_entries[prev_state]
                state_durations[prev_state].append(duration_us)
            state_entries[state] = time_us
        
        # Detect new iteration (entering WAIT_STABLE from ADJUST or IDLE)
        if state == '2' and prev_state in ['6', '0', '1', None]:
            iteration += 1
            if iteration > 1:
                print()
        
        # Print state transitions with debug data
        if state != prev_state:
            state_name = STATE_NAMES.get(state, f'Unknown({state})')
            
            if state == '5':  # ANALYZE
                state_5_count += 1
                phase = PHASE_NAMES.get(debug6, f'Phase {debug6}')
                print(f"Iter {iteration} ANALYZE ({phase}, phase iter {debug7}):")
                print(f"  Rise Time:   {debug2} ms")
                print(f"  Overshoot:   {debug3} %")
                print(f"  D-term Osc:  {debug4} Hz")
                print(f"  Gyro Osc:    {debug5} Hz")
                phases_seen.add(debug6)
                
            elif state == '6':  # ADJUST
                state_6_count += 1
                phase = PHASE_NAMES.get(debug6, f'Phase {debug6}')
                print(f"Iter {iteration} ADJUST ({phase}, phase iter {debug7}):")
                print(f"  Original P:  {debug1}")
                print(f"  New P:       {debug2}")
                print(f"  Original D:  {debug3}")
                print(f"  New D:       {debug4}")
                print(f"  Ratio×100:   {debug5}")
                phases_seen.add(debug6)
                
            elif state in ['0', '1', '7', '8']:
                # Print major state transitions
                if state == '1':
                    print(f"\n=== SETUP - Starting Autotune ===")
                elif state == '7':
                    print(f"\n=== COMPLETE - Autotune Finished ===")
                elif state == '8':
                    print(f"\n=== ABORTED - Autotune Stopped ===")
        
        prev_state = state
    
    # Calculate final state entry duration
    if prev_state in state_entries and prev_state in STATE_NAMES:
        duration_us = time_us - state_entries[prev_state]
        state_durations[prev_state].append(duration_us)
    
    print(f"\n{'='*60}")
    print(f"SUMMARY")
    print(f"{'='*60}")
    print(f"Total iterations:      {iteration}")
    print(f"ANALYZE states seen:   {state_5_count}")
    print(f"ADJUST states seen:    {state_6_count}")
    print(f"Phases encountered:    {', '.join(sorted(phases_seen))}")
    
    print(f"\n{'='*60}")
    print(f"STATE DURATIONS (average)")
    print(f"{'='*60}")
    for state_num, durations in state_durations.items():
        if durations:
            avg_ms = sum(durations) / len(durations) / 1000
            state_name = STATE_NAMES.get(state_num, f'State {state_num}')
            print(f"{state_name:15} {avg_ms:6.1f} ms (n={len(durations)})")
    
    if state_5_count == 0:
        print(f"\n⚠️  WARNING: No ANALYZE states detected!")
        print(f"   State 5 is executing too fast for blackbox to capture.")
    
    if state_6_count == 0:
        print(f"\n⚠️  WARNING: No ADJUST states detected!")
        print(f"   State 6 is executing too fast for blackbox to capture.")
    
    if '0' not in phases_seen:
        print(f"\n⚠️  WARNING: Phase 0 never seen!")
        print(f"   Phase state machine may not be initializing correctly.")
