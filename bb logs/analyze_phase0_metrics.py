#!/usr/bin/env python3
"""
Phase 0 Sweep Metrics Analyzer
Extracts and visualizes all step response metrics for manual scoring
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def analyze_sweep(filename):
    """Extract and visualize Phase 0 sweep metrics from blackbox CSV"""
    
    print(f"Loading {filename}...")
    df = pd.read_csv(filename, skiprows=147)
    
    # New debug layout for DEBUG_AUTOTUNE_METRICS (mode 100):
    # debug[0] = State (4 = ANALYZE)
    # debug[1] = Iteration number
    # debug[2] = P gain
    # debug[3] = D gain
    # debug[4] = Overshoot × 10
    # debug[5] = D-term oscillation amplitude × 10
    # debug[6] = Settling time (ms)
    # debug[7] = Status code
    
    # Extract ANALYZE state periods (state 4)
    analyze_mask = df['debug[0]'] == 4
    analyze_df = df[analyze_mask]
    
    if len(analyze_df) == 0:
        print("ERROR: No ANALYZE state data found (debug[0] == 4)")
        print(f"Unique debug[0] values: {df['debug[0]'].unique()}")
        return None
    
    # Group by iteration number
    iterations = analyze_df.groupby('debug[1]')
    
    print(f"Found {len(iterations)} iterations")
    
    metrics = []
    for iter_num, group in iterations:
        # Get first sample from each iteration
        sample = group.iloc[0]
        
        # Extract from hybrid DEBUG_AUTOTUNE_METRICS layout
        state = sample['debug[0]']
        iteration = sample['debug[1]']
        p_gain = sample['debug[2]']
        d_gain = sample['debug[3]']
        overshoot = sample['debug[4]'] / 10.0  # Decode from × 10
        dterm_osc = sample['debug[5]'] / 10.0  # Decode from × 10
        rise_time_sp = sample['debug[6]'] / 10.0  # ms × 10
        tracking_err = sample['debug[7]'] / 10.0  # deg/s × 10
        
        metrics.append({
            'iteration': int(iteration),
            'P': p_gain,
            'D': d_gain,
            'D/P': d_gain / p_gain if p_gain > 0 else 0,
            'overshoot_pct': overshoot,
            'dterm_osc': dterm_osc,
            'rise_ms': rise_time_sp,
            'tracking_err': tracking_err
        })
    
    df_metrics = pd.DataFrame(metrics)
    df_metrics = df_metrics.sort_values('iteration')
    
    # Print summary table
    print("\n" + "="*80)
    print("Phase 0 Sweep Summary")
    print("="*80)
    print(df_metrics[['iteration', 'P', 'D', 'D/P', 'overshoot_pct', 'dterm_osc', 
                      'rise_ms', 'tracking_err']].to_string(index=False))
    print("\nStatus Codes: 0=normal, 100=D limit, 101=max iter, 102=sweep complete")
    print("="*80)
    
    
    # Visualization - expanded to 3x2 grid for all metrics
    fig, axes = plt.subplots(3, 2, figsize=(14, 14))
    fig.suptitle('Phase 0 Sweep: Metrics Analysis', fontsize=14, fontweight='bold')
    
    x = df_metrics['D/P']
    iterations_x = df_metrics['iteration']
    
    # Plot 1: D/P Ratio progression
    ax1 = axes[0, 0]
    ax1.plot(iterations_x, x, 'b-o', linewidth=2, markersize=6)
    ax1.set_ylabel('D/P Ratio', fontsize=11)
    ax1.set_xlabel('Iteration', fontsize=11)
    ax1.set_title('D/P Ratio Progression', fontsize=12, fontweight='bold')
    ax1.axhline(y=0.67, color='purple', linestyle='--', alpha=0.5, label='Common ratio (0.67)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Overshoot
    ax2 = axes[0, 1]
    ax2.plot(iterations_x, df_metrics['overshoot_pct'], 'r-o', linewidth=2, markersize=6)
    ax2.set_ylabel('Overshoot (%)', fontsize=11)
    ax2.set_xlabel('Iteration', fontsize=11)
    ax2.set_title('Overshoot vs Iteration', fontsize=12, fontweight='bold')
    ax2.axhspan(5, 15, alpha=0.2, color='green', label='Target range')
    ax2.axhline(y=20, color='orange', linestyle='--', alpha=0.5, label='Warning (20%)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: D-term Oscillation (KEY METRIC)
    ax3 = axes[1, 0]
    ax3.plot(iterations_x, df_metrics['dterm_osc'], 'm-o', linewidth=2, markersize=6)
    ax3.set_ylabel('D-term Oscillation Amplitude', fontsize=11)
    ax3.set_xlabel('Iteration', fontsize=11)
    ax3.set_title('D-term Oscillation (Key Overdamping Indicator)', fontsize=12, fontweight='bold')
    ax3.axhspan(15, 40, alpha=0.2, color='green', label='Optimal range')
    ax3.axhline(y=50, color='red', linestyle='--', alpha=0.7, label='Excessive (>50)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Rise Time to Setpoint
    ax4 = axes[1, 1]
    ax4.plot(iterations_x, df_metrics['rise_ms'], 'c-o', linewidth=2, markersize=6)
    ax4.set_ylabel('Rise Time to Setpoint (ms)', fontsize=11)
    ax4.set_xlabel('Iteration', fontsize=11)
    ax4.set_title('Rise Time vs Iteration (Responsiveness)', fontsize=12, fontweight='bold')
    ax4.axhline(y=50, color='green', linestyle='--', alpha=0.5, label='Fast (<50ms)')
    ax4.axhline(y=100, color='orange', linestyle='--', alpha=0.5, label='Slow (>100ms)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Tracking Error (NEW)
    ax5 = axes[2, 0]
    ax5.plot(iterations_x, df_metrics['tracking_err'], 'g-o', linewidth=2, markersize=6)
    ax5.set_ylabel('Tracking Error (deg/s)', fontsize=11)
    ax5.set_xlabel('Iteration', fontsize=11)
    ax5.set_title('Tracking Error vs Iteration (Stability)', fontsize=12, fontweight='bold')
    ax5.axhline(y=100, color='green', linestyle='--', alpha=0.5, label='Good (<100)')
    ax5.axhline(y=150, color='orange', linestyle='--', alpha=0.5, label='Poor (>150)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # Plot 6: Composite view - Rise Time vs Tracking Error
    ax6 = axes[2, 1]
    scatter = ax6.scatter(df_metrics['rise_ms'], df_metrics['tracking_err'], 
                         c=df_metrics['dterm_osc'], s=100, cmap='RdYlGn_r', alpha=0.7)
    ax6.set_xlabel('Rise Time (ms)', fontsize=11)
    ax6.set_ylabel('Tracking Error (deg/s)', fontsize=11)
    ax6.set_title('Rise Time vs Tracking Error (colored by D-term osc)', fontsize=12, fontweight='bold')
    cbar = plt.colorbar(scatter, ax=ax6)
    cbar.set_label('D-term Oscillation', fontsize=10)
    ax6.grid(True, alpha=0.3)
    # Annotate each point with iteration number
    for idx, row in df_metrics.iterrows():
        ax6.annotate(f"{int(row['iteration'])}", 
                    (row['rise_ms'], row['tracking_err']),
                    fontsize=8, ha='center')
    
    plt.tight_layout()
    plt.savefig(filename.replace('.csv', '_metrics.png'), dpi=150)
    plt.show()
    
    # Print summary table
    print("\n" + "="*90)
    print("Phase 0 Sweep Summary")
    print("="*90)
    print(df_metrics[['iteration', 'P', 'D', 'D/P', 'overshoot_pct', 'dterm_osc', 
                      'settling_ms', 'status']].to_string(index=False))
    print("\n" + "="*90)
    print("MANUAL ASSESSMENT GUIDE:")
    print("-" * 90)
    print("Look for iteration with:")
    print("  - Moderate overshoot (5-15%)")
    print("  - Minimal D-term oscillation (<40) - KEY METRIC")
    print("  - Fast rise time (<60ms)")
    print("  - Low tracking error (<100 deg/s)")
    print("  - D/P ratio typically 0.6-1.5")
    print("\nRED FLAGS:")
    print("  - D-term osc > 50: Excessive D (overdamped)")
    print("  - Overshoot > 25%: Insufficient D (underdamped)")
    print("  - Settling time increasing: Getting sluggish")
    print("="*90)
    
    return df_metrics
    
    plt.tight_layout()
    output_file = filename.replace('.csv', '_metrics.png')
    plt.savefig(output_file, dpi=150)
    print(f"\nPlot saved to: {output_file}")
    plt.show()
    
    # Save metrics to CSV for further analysis
    csv_output = filename.replace('.csv', '_metrics_summary.csv')
    phase0_df.to_csv(csv_output, index=False)
    print(f"Metrics summary saved to: {csv_output}")
    
    return df_metrics

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_phase0_metrics.py <blackbox_csv_file>")
        print("Example: python analyze_phase0_metrics.py btfl_0020.bbl.csv")
        sys.exit(1)
    
    filename = sys.argv[1]

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_phase0_metrics.py <blackbox_csv_file>")
        print("Example: python analyze_phase0_metrics.py btfl_0020.bbl.csv")
        sys.exit(1)
    
    filename = sys.argv[1]
    result = analyze_sweep(filename)
    
    if result is not None:
        print("\n" + "="*90)
        print("NEXT STEPS:")
        print("="*90)
        print("1. Review the 4 plots to understand metric trends")
        print("2. Identify iteration where D-term osc starts climbing rapidly (overdamped)")
        print("3. Ideal iteration is typically 1-2 steps before D-term osc spike")
        print("4. Record best iteration number and use for automated scoring calibration")
        print("="*90)

