#!/usr/bin/env python3
"""
Analyze filter tuning blackbox logs.

Debug channels for filter mode:
[0] = State (0=IDLE, 1=ARMED, 2=DETECTING, 3=COLLECTING, 4=SETTLING, 5=ANALYZING, 6=ADJUSTING, 7=SIGNALING)
[1] = mode (lower 4 bits) | iteration (upper 4 bits)  -> mode 2 = FILTER
[2] = dterm_lpf1 Hz (in filter mode) or P gain (in roll/pitch mode)
[3] = gyro_lpf1 Hz (in filter mode) or D gain (in roll/pitch mode)  
[4] = status code
[5] = noise ratio × 10 (in filter mode) or overshoot (in roll/pitch mode)
[6] = filter mode indicator (100 = filter, 0 = not filter) or noise × 10
[7] = filter score × 100 (in filter mode) or gain score × 100

States: IDLE=0, ARMED=1, DETECTING=2, COLLECTING=3, SETTLING=4, ANALYZING=5, ADJUSTING=6, SIGNALING=7
"""

import pandas as pd
import sys

STATE_NAMES = {
    0: "IDLE",
    1: "ARMED", 
    2: "DETECTING",
    3: "COLLECTING",
    4: "SETTLING",
    5: "ANALYZING",
    6: "ADJUSTING",
    7: "SIGNALING"
}

def analyze_filter_log(filename):
    print(f"\n=== Filter Tuning Analysis - {filename} ===\n")
    
    # Read CSV, skip header metadata (first 147 rows, data starts at 148)
    df = pd.read_csv(filename, skiprows=147)
    
    print(f"Total samples: {len(df)}")
    print(f"Duration: {(df['time'].iloc[-1] - df['time'].iloc[0]) / 1e6:.2f} seconds\n")
    
    # Show state distribution
    print("=== State Distribution ===")
    state_counts = df['debug[0]'].value_counts().sort_index()
    for state, count in state_counts.items():
        pct = count / len(df) * 100
        name = STATE_NAMES.get(int(state), "???")
        print(f"  {int(state)} ({name:10}): {count:6} samples ({pct:5.1f}%)")
    print()
    
    # Find filter mode periods (debug[6] == 100 or mode bits == 2)
    # Mode is in lower 4 bits of debug[1]
    debug1_int = df['debug[1]'].astype(int)
    df['mode'] = debug1_int.mod(16)  # lower 4 bits
    df['iteration'] = debug1_int.floordiv(16).mod(16)  # next 4 bits
    
    filter_mode = df[df['debug[6]'] == 100]
    if len(filter_mode) > 0:
        print(f"=== Filter Mode Active ===")
        print(f"Samples in filter mode: {len(filter_mode)}")
        print()
    
    # Find DETECTING state periods (state == 2)
    detecting = df[df['debug[0]'] == 2]
    if len(detecting) > 0:
        print(f"=== DETECTING State Analysis ===")
        print(f"Samples: {len(detecting)}")
        
        # Check throttle during detecting
        if 'rcCommand[3]' in df.columns:
            throttle_vals = detecting['rcCommand[3]']
            print(f"Throttle: min={throttle_vals.min()}, max={throttle_vals.max()}, mean={throttle_vals.mean():.0f}")
        print()
    
    # Find COLLECTING state periods (state == 3)
    collecting = df[df['debug[0]'] == 3]
    if len(collecting) > 0:
        print(f"=== COLLECTING State Analysis ===")
        print(f"Samples: {len(collecting)}")
        print()
    
    # Find ANALYZING periods (state == 5)
    analyzing = df[df['debug[0]'] == 5]
    if len(analyzing) > 0:
        print(f"=== ANALYZING State ===")
        print(f"Samples: {len(analyzing)}")
        
        # Show filter values during analysis
        if len(analyzing) > 0:
            sample = analyzing.iloc[0]
            dterm_lpf1 = sample['debug[2]']
            gyro_lpf1 = sample['debug[3]']
            noise_ratio = sample['debug[5]'] / 10.0
            filter_score = sample['debug[7]'] / 100.0
            print(f"  D-term LPF1: {dterm_lpf1} Hz")
            print(f"  Gyro LPF1: {gyro_lpf1} Hz")
            print(f"  Noise ratio: {noise_ratio:.1f}")
            print(f"  Filter score: {filter_score:.2f}")
        print()
    
    # Find ADJUSTING periods (state == 6)
    adjusting = df[df['debug[0]'] == 6]
    if len(adjusting) > 0:
        print(f"=== ADJUSTING State ===")
        print(f"Samples: {len(adjusting)}")
        
        if len(adjusting) > 0:
            sample = adjusting.iloc[0]
            dterm_lpf1 = sample['debug[2]']
            gyro_lpf1 = sample['debug[3]']
            noise_ratio = sample['debug[5]'] / 10.0
            filter_score = sample['debug[7]'] / 100.0
            print(f"  D-term LPF1: {dterm_lpf1} Hz")
            print(f"  Gyro LPF1: {gyro_lpf1} Hz")
            print(f"  Noise ratio: {noise_ratio:.1f}")
            print(f"  Filter score: {filter_score:.2f}")
        print()
    
    # Find SIGNALING periods (state == 7)
    signaling = df[df['debug[0]'] == 7]
    if len(signaling) > 0:
        print(f"=== SIGNALING State (Wiggle) ===")
        print(f"Samples: {len(signaling)}")
        print()
    
    # Show state transitions over time
    print("=== State Transitions ===")
    prev_state = None
    transitions = []
    for idx, row in df.iterrows():
        state = int(row['debug[0]'])
        if state != prev_state:
            time_ms = row['time'] / 1000
            name = STATE_NAMES.get(state, "???")
            mode = int(row['debug[1]']) & 0x0F
            mode_name = {0: "NONE", 1: "ROLL", 2: "PITCH", 3: "FILTER"}.get(mode, "???")
            transitions.append((time_ms, state, name, mode, mode_name))
            prev_state = state
    
    # Print first 30 and last 10 transitions
    print(f"Total transitions: {len(transitions)}")
    print("\nFirst transitions:")
    for t in transitions[:30]:
        print(f"  {t[0]:8.1f}ms: {t[1]} ({t[2]:10}) mode={t[3]} ({t[4]})")
    
    if len(transitions) > 40:
        print(f"\n... ({len(transitions) - 40} more) ...\n")
        print("Last transitions:")
        for t in transitions[-10:]:
            print(f"  {t[0]:8.1f}ms: {t[1]} ({t[2]:10}) mode={t[3]} ({t[4]})")
    print()
    
    # Analyze throttle patterns
    print("=== Throttle Analysis ===")
    if 'rcCommand[3]' in df.columns:
        throttle = df['rcCommand[3]']
        print(f"Min: {throttle.min()}, Max: {throttle.max()}, Mean: {throttle.mean():.0f}")
        
        # Find high throttle periods (>1500)
        high_throttle = df[df['rcCommand[3]'] > 1500]
        if len(high_throttle) > 0:
            print(f"High throttle (>1500) samples: {len(high_throttle)}")
    print()
    
    # Look for gyro noise patterns
    print("=== Gyro Noise Analysis ===")
    if 'gyroADC[0]' in df.columns:
        gyro = df[['gyroADC[0]', 'gyroADC[1]', 'gyroADC[2]']]
        print(f"Roll  gyro: std={gyro['gyroADC[0]'].std():.1f}")
        print(f"Pitch gyro: std={gyro['gyroADC[1]'].std():.1f}")
        print(f"Yaw   gyro: std={gyro['gyroADC[2]'].std():.1f}")
    print()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "btfl_filter_1.bbl.csv"
    
    analyze_filter_log(filename)
