import csv

with open('btfl_011.bbl.csv') as f:
    # Skip to data
    for _ in range(148):
        next(f)
    
    reader = csv.reader(f)
    
    # Find all state transitions
    prev_state = None
    iteration = 0
    
    print("=== Autotune Analysis - btfl_011.bbl.csv ===\n")
    
    for row in reader:
        if len(row) <= 40:
            continue
            
        state = row[33]
        
        # Detect new iteration (state 2)
        if state == '2' and prev_state in ['7', None]:
            iteration += 1
            if iteration > 1:
                print()
        
        # Print key state transitions with full debug data
        if state != prev_state:
            if state == '5':  # ANALYZE
                print(f"Iter {iteration} ANALYZE:")
                print(f"  Rise Time:  {row[35]} ms")
                print(f"  Overshoot:  {row[36]} %")
                print(f"  D-term Osc: {row[37]} Hz")
                print(f"  Gyro Osc:   {row[38]} Hz")
                
            elif state == '6':  # ADJUST  
                print(f"Iter {iteration} ADJUST:")
                orig_d = row[35]  # debug[2]
                new_d = row[36]   # debug[3]
                orig_p = row[38]  # debug[5]
                new_p = row[40]   # debug[7]
                orig_i = row[39]  # debug[6]
                
                print(f"  P: {orig_p:>3} -> {new_p:>3}")
                print(f"  D: {orig_d:>3} -> {new_d:>3}")
                print(f"  I: {orig_i:>3}")
                
        prev_state = state
        
    print(f"\n=== Completed {iteration} iterations ===")
