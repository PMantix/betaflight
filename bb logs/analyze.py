import csv

with open('btfl_009.bbl.csv') as f:
    # Skip to data
    for _ in range(148):
        next(f)
    
    reader = csv.reader(f)
    
    # Find state 6 rows
    state6_rows = [row for row in reader if len(row) > 40 and row[33] == '6']
    
    print(f'Found {len(state6_rows)} state 6 samples')
    
    # Extract unique iterations (when time jumps > 1s)
    iterations = []
    prev_time = 0
    for row in state6_rows:
        t = int(row[1])
        if t - prev_time > 1000000:  # 1 second gap
            iterations.append({
                'orig_P': row[38],
                'orig_I': row[39], 
                'new_P': row[40]
            })
            prev_time = t
    
    print(f'\nFound {len(iterations)} iterations:')
    for i, it in enumerate(iterations):
        print(f"Iter {i+1}: OrigP={it['orig_P']:>3}, OrigI={it['orig_I']:>3}, NewP={it['new_P']:>3}")
