#!/usr/bin/env python3
"""
Analyze autotune status codes from blackbox log
Shows which code paths were triggered during tuning
"""

import pandas as pd
import sys

# Status code definitions (from autotune_types.h)
STATUS_CODES = {
    # Phase 1 sweep progress (0-15 = iteration number)
    1: "P1: Sweep starting",
    17: "P1: Early stop (scores degrading)",
    18: "P1: Max iterations, applying best",
    19: "P1: Sweep failed (D > 2.5P)",
    
    # Phase 2: Scale Up (20-29)
    20: "P2: Aggressive +25% (no limits)",
    21: "P2: Cautious +10% (approaching limits)",
    22: "P2: Noise limit hit, backing off",
    23: "P2: Overshoot limit",
    24: "P2: Max iterations",
    
    # Phase 3: Fine Tune (30-39)
    30: "P3: Reducing D 15% (high noise >15Hz)",
    31: "P3: Reducing D 5% (moderate noise)",
    32: "P3: Clean after noise - DONE!",
    33: "P3: Final +5% push",
    34: "P3: Max iterations",
    
    # Completion (50-59)
    50: "COMPLETE",
    
    # Errors (90-99)
    99: "ABORTED",
    
    # Safety failures (100-109)
    100: "Safety check failed",
    101: "Attitude check failed",
    102: "Gyro rate too high",
    103: "Throttle out of range",
    104: "Stick deflection",
    
    # Analysis warnings (110-119)
    110: "Rise time warning",
    111: "Overshoot warning",
    112: "Noise warning",
    113: "Stick interference",
    
    # Analysis errors (200+)
    200: "Analysis invalid",
    201: "Analysis rejected",
}

def analyze_status(filename):
    print(f"\n=== Autotune Status Code Analysis - {filename} ===\n")
    
    # Read CSV, skipping header metadata
    df = pd.read_csv(filename, skiprows=147)
    
    # Look at ANALYZE state (debug[0] == 4) to find when decisions were made
    # Status code should be visible in one of the debug channels
    
    # Group by iteration (debug[1])
    iterations = df.groupby('debug[1]')
    
    print("=" * 90)
    print(f"{'Iter':>4} {'State':>6} {'Phase':>5} {'P':>5} {'D':>5} {'Score':>6} {'Status':>6} {'Meaning'}")
    print("=" * 90)
    
    last_status = -1
    for iter_num, group in iterations:
        # Get first sample from this iteration
        sample = group.iloc[0]
        
        state = int(sample['debug[0]'])
        phase = int(sample['debug[2]'] if 'debug[2]' in sample else 0)
        p_gain = int(sample['debug[2]'] if state == 4 else 0)  # In ANALYZE, debug[2] = P
        d_gain = int(sample['debug[3]'] if state == 4 else 0)  # In ANALYZE, debug[3] = D
        score = sample['debug[6]'] if state == 4 else 0
        
        # For status code, we need to look at when state changes
        # The status is set during ADJUST phase, which comes after ANALYZE
        
        # Let's just track unique iterations and their gains
        if iter_num > 0 and state == 4:  # ANALYZE state
            meaning = STATUS_CODES.get(int(score), f"Phase iter {int(score)}" if score < 16 else f"Unknown ({int(score)})")
            print(f"{iter_num:4d} {state:6d} {phase:5d} {p_gain:5d} {d_gain:5d} {score:6.0f} {'':>6} {meaning}")
    
    print("=" * 90)
    
    # Also show phase transitions by tracking P changes
    print("\n=== Gain Progression ===")
    prev_p = 0
    for iter_num, group in iterations:
        sample = group.iloc[0]
        state = int(sample['debug[0]'])
        if state == 4 and iter_num > 0:  # ANALYZE state
            p_gain = int(sample['debug[2]'])
            d_gain = int(sample['debug[3]'])
            
            if p_gain != prev_p and prev_p > 0:
                print(f"  ⚡ Phase transition at iter {iter_num}: P changed {prev_p} → {p_gain}")
            prev_p = p_gain
    
    print()

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_status.py <blackbox_csv_file>")
        sys.exit(1)
    
    filename = sys.argv[1]
    analyze_status(filename)

if __name__ == "__main__":
    main()
