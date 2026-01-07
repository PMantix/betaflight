#!/usr/bin/env python3
"""
Analyze btfl_0019 - Phase 0 sweep with new metrics
Debug channels:
[0] = State (0-8)
[1] = Global iteration
[2] = Phase (0-3)
[3] = Phase iteration
[4] = P gain
[5] = D gain
[6] = Status code
[7] = (dampingState × 100) + zeroCrossings
"""

import pandas as pd
import sys

def main(filename):
    print(f"\n=== Autotune Phase 0 Sweep Analysis - {filename} ===\n")
    
    # Read CSV
    df = pd.read_csv(filename, skiprows=148)  # Skip header metadata
    
    # Find ANALYZE state periods (debug[0] == 4)
    analyze_mask = df['debug[0]'] == 4
    analyze_periods = df[analyze_mask].copy()
    
    if len(analyze_periods) == 0:
        print("No ANALYZE state found!")
        return
    
    # Group by iteration (debug[1])
    iterations = analyze_periods.groupby('debug[1]')
    
    print("=" * 100)
    print(f"{'Iter':>4} {'Phase':>5} {'PhItr':>5} {'P':>5} {'D':>5} {'Status':>6} "
          f"{'Damp':>6} {'ZC':>3} {'RiseT':>7} {'Over%':>6} {'SettleT':>8} {'DtAmp':>7}")
    print("=" * 100)
    
    sweep_data = []
    
    for iter_num, group in iterations:
        # Get the values from first sample of this iteration (they should be constant during ANALYZE)
        first_sample = group.iloc[0]
        
        phase = int(first_sample['debug[2]'])
        phase_iter = int(first_sample['debug[3]'])
        p_gain = int(first_sample['debug[4]'])
        d_gain = int(first_sample['debug[5]'])
        status = int(first_sample['debug[6]'])
        damp_and_zc = int(first_sample['debug[7]'])
        
        # Decode damping state and zero crossings
        damp_state = damp_and_zc // 100
        zero_crossings = damp_and_zc % 100
        
        damp_names = {0: "UNK", 1: "UNDER", 2: "CRIT", 3: "OVER"}
        damp_str = damp_names.get(damp_state, "???")
        
        # Extract metrics from axisP/D/I (these are the analyzed values)
        # Note: The actual metric values are not in debug channels yet, 
        # they're in the analysis struct. We'll need to look at gyro/setpoint data.
        
        # For now, show what we have
        print(f"{iter_num:4d} {phase:5d} {phase_iter:5d} {p_gain:5d} {d_gain:5d} {status:6d} "
              f"{damp_str:>6} {zero_crossings:3d} {'---':>7} {'---':>6} {'---':>8} {'---':>7}")
        
        sweep_data.append({
            'iter': iter_num,
            'phase': phase,
            'phase_iter': phase_iter,
            'P': p_gain,
            'D': d_gain,
            'D/P': d_gain / p_gain if p_gain > 0 else 0,
            'status': status,
            'damping': damp_str,
            'zero_crossings': zero_crossings
        })
    
    print("=" * 100)
    print()
    
    # Show sweep progression
    print("=== Sweep Progression ===")
    print(f"{'Iter':>4} {'P':>5} {'D':>5} {'D/P':>6} {'Status':>8} {'Damping':>8} {'ZC':>3}")
    print("-" * 50)
    for data in sweep_data:
        status_desc = {
            0: "normal",
            1: "under",
            2: "crit",
            3: "over",
            100: "D>2.5P",
            101: "maxiter"
        }.get(data['status'], f"code{data['status']}")
        
        print(f"{data['iter']:4d} {data['P']:5d} {data['D']:5d} {data['D/P']:6.2f} "
              f"{status_desc:>8} {data['damping']:>8} {data['ZC']:3d}")
    
    print()
    
    # Find when sweep ended
    last_iter = sweep_data[-1]
    if last_iter['status'] == 3:
        print(f"✅ Sweep completed successfully at iteration {last_iter['iter']}")
        print(f"   Found OVERDAMPED with P={last_iter['P']}, D={last_iter['D']} (D/P={last_iter['D/P']:.2f})")
    elif last_iter['status'] == 100:
        print(f"❌ Sweep aborted: D exceeded 2.5×P at iteration {last_iter['iter']}")
    elif last_iter['status'] == 101:
        print(f"❌ Sweep aborted: Maximum iterations reached at {last_iter['iter']}")
    else:
        print(f"⚠️  Sweep ended with status {last_iter['status']} at iteration {last_iter['iter']}")
    
    print()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_019.py <logfile.csv>")
        sys.exit(1)
    
    main(sys.argv[1])
