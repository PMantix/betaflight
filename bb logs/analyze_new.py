import csv

with open('btfl_010.bbl.csv') as f:
    # Skip to data
    for _ in range(148):
        next(f)
    
    reader = csv.reader(f)
    
    # Find all state transitions
    prev_state = None
    iteration = 0
    
    print("=== Autotune Analysis - btfl_010.bbl.csv ===\n")
    
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
                print(f"  Rise Time: {row[35]} ms")
                print(f"  Overshoot: {row[36]} %")
                print(f"  Osc Freq:  {row[37]} Hz")
                
            elif state == '6':  # ADJUST  
                print(f"Iter {iteration} ADJUST:")
                print(f"  Orig P: {row[38]:>3}  ->  New P: {row[40]:>3}")
                print(f"  Orig I: {row[39]:>3}")
                # Check if D is now logged in debug[2] and debug[3]
                if len(row) > 35:
                    orig_d = row[35]  # debug[2]
                    new_d = row[36]   # debug[3]
                    if orig_d and new_d:
                        print(f"  Orig D: {orig_d:>3}  ->  New D: {new_d:>3}")
                
        prev_state = state
        
    print(f"\n=== Completed {iteration} iterations ===")
