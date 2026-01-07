import csv

with open('btfl_009.bbl.csv') as f:
    # Skip to data
    for _ in range(148):
        next(f)
    
    reader = csv.reader(f)
    
    # Find all state transitions
    prev_state = None
    prev_time = 0
    iteration = 0
    
    for row in reader:
        if len(row) <= 40:
            continue
            
        state = row[33]
        t = int(row[1])
        
        # Detect new iteration (state 2 after state 7)
        if state == '2' and prev_state == '7' and t - prev_time > 500000:
            iteration += 1
            print(f"\n=== Iteration {iteration} ===")
        
        # Print state transitions with key debug values
        if state != prev_state:
            if state == '5':  # ANALYZE
                print(f"  STATE ANALYZE: OscFreq={row[37]}Hz, RiseTime={row[35]}ms, Overshoot={row[36]}%")
            elif state == '6':  # ADJUST  
                print(f"  STATE ADJUST: OrigP={row[38]}, OrigI={row[39]}, NewP={row[40]}")
                
        prev_state = state
        prev_time = t
