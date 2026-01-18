#!/usr/bin/env python3
"""
Detailed response analysis - look at actual oscillation/settling behavior
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

def find_header_row(filename):
    with open(filename, 'r') as f:
        for i, line in enumerate(f):
            if 'loopIteration' in line:
                return i
    return 0

def main():
    filename = sys.argv[1] if len(sys.argv) > 1 else 'btfl__mk4___subagent_2.bbl.csv'
    
    header_row = find_header_row(filename)
    df = pd.read_csv(filename, skiprows=header_row)
    
    print(f"Loaded {len(df)} samples from {filename}")
    
    # Get relevant columns
    time_us = df['time'].values
    rel_time = (time_us - time_us[0]) / 1e6
    
    # Gyro and setpoint
    gyro_roll = df['gyroADC[0]'].values if 'gyroADC[0]' in df.columns else np.zeros(len(df))
    setpoint_roll = df['setpoint[0]'].values if 'setpoint[0]' in df.columns else np.zeros(len(df))
    
    # Sample rate
    avg_dt = np.mean(np.diff(time_us[:1000]))
    sample_rate = 1e6 / avg_dt
    print(f"Sample rate: {sample_rate:.0f} Hz")
    
    # Find a representative maneuver - look for when setpoint goes high then returns to ~0
    # This is the settling phase we want to analyze
    
    maneuver_threshold = 150  # deg/s for significant maneuver
    settle_threshold = 20     # deg/s for "stick released"
    
    # Find maneuver end points (where setpoint drops from high to low)
    maneuver_ends = []
    for i in range(100, len(setpoint_roll) - 1):
        if abs(setpoint_roll[i-1]) > maneuver_threshold and abs(setpoint_roll[i]) < settle_threshold:
            maneuver_ends.append(i)
    
    print(f"\nFound {len(maneuver_ends)} maneuver end points (stick release)")
    
    # Analyze settling behavior after each maneuver
    print("\n" + "=" * 80)
    print("SETTLING ANALYSIS (what happens AFTER stick is released)")
    print("=" * 80)
    
    oscillation_data = []
    
    for m_idx, end_idx in enumerate(maneuver_ends[:10]):  # First 10 maneuvers
        # Look at 500ms window after stick release
        window_samples = int(0.5 * sample_rate)
        window_end = min(end_idx + window_samples, len(gyro_roll))
        
        t_window = rel_time[end_idx:window_end] - rel_time[end_idx]
        gyro_window = gyro_roll[end_idx:window_end]
        sp_window = setpoint_roll[end_idx:window_end]
        
        # With stick released, setpoint should be ~0
        # Gyro should settle to ~0 if well damped
        # If underdamped, gyro will oscillate around 0
        
        # Find zero crossings in gyro (oscillation indicator)
        zero_crossings = 0
        for i in range(1, len(gyro_window)):
            if gyro_window[i-1] * gyro_window[i] < 0:
                zero_crossings += 1
        
        # Peak-to-peak oscillation amplitude
        gyro_max = np.max(gyro_window[:min(100, len(gyro_window))])  # First 100ms
        gyro_min = np.min(gyro_window[:min(100, len(gyro_window))])
        oscillation_amplitude = gyro_max - gyro_min
        
        # Mean absolute gyro (should be low if well damped)
        mean_abs_gyro = np.mean(np.abs(gyro_window[:min(200, len(gyro_window))]))
        
        # Settling time (time to get within ±10 deg/s and stay there)
        settled_idx = None
        for i in range(len(gyro_window)):
            if i > 10:  # Need at least 10 samples to confirm settled
                if np.all(np.abs(gyro_window[i-10:i]) < 10.0):
                    settled_idx = i
                    break
        
        settling_time_ms = (settled_idx / sample_rate * 1000) if settled_idx else float('inf')
        
        print(f"\nManeuver {m_idx+1} at t={rel_time[end_idx]:.2f}s:")
        print(f"  Setpoint at release: {setpoint_roll[end_idx-1]:.1f} deg/s -> {setpoint_roll[end_idx]:.1f} deg/s")
        print(f"  Gyro at release:     {gyro_roll[end_idx]:.1f} deg/s")
        print(f"  Zero crossings:      {zero_crossings} (more = oscillating)")
        print(f"  Oscillation P-P:     {oscillation_amplitude:.1f} deg/s")
        print(f"  Mean |gyro| (200ms): {mean_abs_gyro:.1f} deg/s")
        print(f"  Settling time:       {settling_time_ms:.0f} ms")
        
        # Classification based on settling behavior
        if zero_crossings >= 3 and oscillation_amplitude > 40:
            behavior = "UNDERDAMPED (oscillating)"
        elif settling_time_ms > 200:
            behavior = "OVERDAMPED (slow to settle)"
        elif settling_time_ms < 100 and oscillation_amplitude < 30:
            behavior = "CRITICALLY DAMPED (good)"
        else:
            behavior = "NEEDS TUNING"
        
        print(f"  --> {behavior}")
        
        oscillation_data.append({
            'time': rel_time[end_idx],
            'zero_crossings': zero_crossings,
            'oscillation_pp': oscillation_amplitude,
            'mean_abs_gyro': mean_abs_gyro,
            'settling_time_ms': settling_time_ms
        })
    
    # Summary statistics
    print("\n" + "=" * 80)
    print("OVERALL SETTLING STATISTICS")
    print("=" * 80)
    
    if oscillation_data:
        avg_zero_crossings = np.mean([d['zero_crossings'] for d in oscillation_data])
        avg_oscillation = np.mean([d['oscillation_pp'] for d in oscillation_data])
        avg_settling = np.mean([d['settling_time_ms'] for d in oscillation_data if d['settling_time_ms'] < 1000])
        
        print(f"Avg zero crossings:    {avg_zero_crossings:.1f}")
        print(f"Avg oscillation P-P:   {avg_oscillation:.1f} deg/s")
        print(f"Avg settling time:     {avg_settling:.0f} ms")
        
        if avg_zero_crossings > 2:
            print("\n⚠️  UNDERDAMPED: Multiple oscillations detected after maneuvers")
            print("   The current classification is WRONG - this should trigger P_DOWN or D_UP")
        elif avg_settling > 200:
            print("\n⚠️  OVERDAMPED: Slow settling detected")
    
    # Also look at DURING maneuver tracking
    print("\n" + "=" * 80)
    print("DURING-MANEUVER ANALYSIS")
    print("=" * 80)
    
    # Find high setpoint periods
    high_sp_mask = np.abs(setpoint_roll) > 100
    
    if np.any(high_sp_mask):
        gyro_during = gyro_roll[high_sp_mask]
        sp_during = setpoint_roll[high_sp_mask]
        
        # Error during tracking
        tracking_error = gyro_during - sp_during
        
        print(f"During high-rate maneuvers ({np.sum(high_sp_mask)} samples):")
        print(f"  Mean setpoint:      {np.mean(np.abs(sp_during)):.1f} deg/s")
        print(f"  Mean gyro:          {np.mean(np.abs(gyro_during)):.1f} deg/s")
        print(f"  Mean tracking error: {np.mean(tracking_error):.1f} deg/s")
        print(f"  RMS tracking error:  {np.sqrt(np.mean(tracking_error**2)):.1f} deg/s")
        
        # Gyro overshooting or undershooting?
        overshoot_samples = np.sum(np.abs(gyro_during) > np.abs(sp_during) * 1.1)
        undershoot_samples = np.sum(np.abs(gyro_during) < np.abs(sp_during) * 0.9)
        
        print(f"  Overshoot samples:   {overshoot_samples} ({100*overshoot_samples/len(gyro_during):.1f}%)")
        print(f"  Undershoot samples:  {undershoot_samples} ({100*undershoot_samples/len(gyro_during):.1f}%)")
    
    # Plot first few maneuvers if matplotlib available
    try:
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        
        # Plot 3 representative settling responses
        for ax_idx, end_idx in enumerate(maneuver_ends[:3]):
            ax = axes[ax_idx]
            
            # Show 100ms before and 500ms after
            pre_samples = int(0.1 * sample_rate)
            post_samples = int(0.5 * sample_rate)
            start = max(0, end_idx - pre_samples)
            end = min(len(gyro_roll), end_idx + post_samples)
            
            t = (rel_time[start:end] - rel_time[end_idx]) * 1000  # ms relative to stick release
            
            ax.plot(t, setpoint_roll[start:end], 'b-', label='Setpoint', alpha=0.7)
            ax.plot(t, gyro_roll[start:end], 'r-', label='Gyro', alpha=0.7)
            ax.axvline(0, color='k', linestyle='--', alpha=0.5, label='Stick release')
            ax.axhline(0, color='gray', linestyle='-', alpha=0.3)
            ax.set_xlabel('Time relative to stick release (ms)')
            ax.set_ylabel('Rate (deg/s)')
            ax.set_title(f'Maneuver {ax_idx+1} settling response')
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.set_xlim(-100, 500)
        
        plt.tight_layout()
        plt.savefig('settling_response.png', dpi=150)
        print("\n✓ Saved plot to settling_response.png")
        plt.close()
    except Exception as e:
        print(f"\n(Could not generate plot: {e})")

if __name__ == "__main__":
    main()
