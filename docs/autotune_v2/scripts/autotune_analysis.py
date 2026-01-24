#!/usr/bin/env python3
"""
Autotune V2 Log Analysis Suite
==============================

Comprehensive analysis tools for validating autotune flight test results.

Usage:
    python autotune_analysis.py <csv_file> [options]

Options:
    --timeline      Generate state timeline plot
    --gains         Generate gain progression plot
    --metrics       Generate metrics trace plot
    --crossaxis     Generate cross-axis regression check
    --summary       Print summary report
    --all           Generate all outputs
    --output-dir    Directory for output files (default: same as input)

Example:
    python autotune_analysis.py btfl_test1.csv --all --output-dir plots/

IMPORTANT: Blackbox CSV files have a 147-row header. This script handles that.
"""

import argparse
import os
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple
import json

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# =============================================================================
# Constants (must match autotune_debug.h and autotune_types.h)
# =============================================================================

BLACKBOX_HEADER_ROWS = 147

# State enum values
STATE_NAMES = {
    0: "IDLE",
    1: "HOVER_LOCK",
    2: "THROTTLE_SWEEP",
    3: "NOISE_CONFIRM",
    4: "PD_RATIO_SEEK",
    5: "PD_SCALE_UP",
    6: "F_TUNE",
    7: "PD_RETUNE_AFTER_F",
    8: "COMPLETE",
}

# Axis enum values
AXIS_NAMES = {
    0: "ROLL",
    1: "PITCH",
    2: "YAW",
}

# Reason code enum values
REASON_NAMES = {
    # Normal progression (0-10)
    0: "NONE",
    1: "EVENT_DETECTED",
    2: "QUALITY_OK",
    3: "DECISION_INCREASE",
    4: "DECISION_DECREASE",
    5: "DECISION_HOLD",
    6: "AXIS_COMPLETE",
    7: "PHASE_COMPLETE",
    8: "HOVER_LOCKED",
    9: "FILTERS_SET",
    10: "TARGET_REACHED",
    # Quality gate failures (11-19)
    11: "INSUFFICIENT_DEFLECTION",
    12: "CROSS_AXIS_CONTAMINATION",
    13: "THROTTLE_OUT_OF_BAND",
    14: "ABNORMAL_DURATION",
    15: "INVALID_METRICS",
    16: "EVENT_TIMEOUT",
    17: "STICK_NOT_CENTERED",
    # Safety events (20-29)
    20: "ROLLBACK_OSCILLATION",
    21: "ROLLBACK_OVERSHOOT",
    22: "GAIN_LIMIT_MIN",
    23: "GAIN_LIMIT_MAX",
    24: "EVENT_LIMIT",
    25: "NOISE_TOO_HIGH",
    26: "TRUST_DEPLETED",
    # Abort conditions (30-35)
    30: "ABORT_SWITCH",
    31: "ABORT_DISARM",
    32: "ABORT_FAILSAFE",
    33: "ABORT_TIMEOUT",
    34: "ABORT_USER",
    35: "ABORT_ERROR",
}

# Debug channel indices
DEBUG_STATE = 0
DEBUG_AXIS = 1
DEBUG_REASON = 2
DEBUG_DECISION = 3
DEBUG_OVERSHOOT = 4
DEBUG_GAIN_P = 5
DEBUG_GAIN_D = 6
DEBUG_GAIN_F = 7

# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class Transition:
    """Represents a state or reason code transition."""
    time_s: float
    from_value: int
    to_value: int
    state: int
    axis: int
    reason: int
    p_gain: int
    d_gain: int
    f_gain: int

@dataclass
class AxisTuneResult:
    """Results for a single axis tuning."""
    axis: int
    initial_p: int
    initial_d: int
    initial_f: int
    final_p: int
    final_d: int
    final_f: int
    event_count: int
    duration_s: float
    converged: bool
    overshoot_initial: float
    overshoot_final: float

@dataclass
class TuneSummary:
    """Overall tuning session summary."""
    log_file: str
    duration_s: float
    final_state: int
    axes_completed: List[int]
    axis_results: Dict[int, AxisTuneResult]
    total_events: int
    abort_reason: Optional[int]
    warnings: List[str]

# =============================================================================
# Data Loading
# =============================================================================

def load_log(csv_path: str) -> pd.DataFrame:
    """
    Load a Blackbox CSV file with proper header handling.
    
    Args:
        csv_path: Path to the CSV file
        
    Returns:
        DataFrame with time converted to seconds and relative time computed
    """
    print(f"Loading: {csv_path}")
    
    df = pd.read_csv(csv_path, skiprows=BLACKBOX_HEADER_ROWS, low_memory=False)
    
    # Verify header skip was correct
    if 'time' not in df.columns:
        raise ValueError("Column 'time' not found - check header skip")
    
    first_time = pd.to_numeric(df['time'].iloc[0], errors='coerce')
    if pd.isna(first_time) or first_time > 1e9:
        raise ValueError(f"First time value {first_time} looks wrong - check header skip")
    
    # Convert time to seconds
    df['time_us'] = pd.to_numeric(df['time'], errors='coerce')
    df['time_s'] = df['time_us'] / 1e6
    df['time_rel'] = df['time_s'] - df['time_s'].min()
    
    # Ensure debug columns are numeric
    for i in range(8):
        col = f'debug[{i}]'
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce').fillna(0).astype(int)
    
    print(f"  Loaded {len(df)} samples, {df['time_rel'].max():.1f}s duration")
    
    return df

# =============================================================================
# Analysis Functions
# =============================================================================

def find_state_transitions(df: pd.DataFrame) -> List[Transition]:
    """Find all state transitions with associated metadata."""
    transitions = []
    
    df['state_prev'] = df['debug[0]'].shift(1)
    mask = df['debug[0]'] != df['state_prev']
    
    for idx in df[mask].index[1:]:  # Skip first row
        row = df.loc[idx]
        trans = Transition(
            time_s=row['time_rel'],
            from_value=int(df.loc[idx-1, 'debug[0]'] if idx > 0 else 0),
            to_value=int(row['debug[0]']),
            state=int(row['debug[0]']),
            axis=int(row['debug[1]']),
            reason=int(row['debug[2]']),
            p_gain=int(row['debug[5]']),
            d_gain=int(row['debug[6]']),
            f_gain=int(row['debug[7]']),
        )
        transitions.append(trans)
    
    return transitions

def find_gain_changes(df: pd.DataFrame) -> Dict[str, List[Transition]]:
    """Find all P, D, F gain changes."""
    changes = {'P': [], 'D': [], 'F': []}
    
    for gain_name, debug_idx in [('P', 5), ('D', 6), ('F', 7)]:
        col = f'debug[{debug_idx}]'
        df['gain_prev'] = df[col].shift(1)
        mask = df[col] != df['gain_prev']
        
        for idx in df[mask].index[1:]:
            row = df.loc[idx]
            trans = Transition(
                time_s=row['time_rel'],
                from_value=int(df.loc[idx-1, col]),
                to_value=int(row[col]),
                state=int(row['debug[0]']),
                axis=int(row['debug[1]']),
                reason=int(row['debug[2]']),
                p_gain=int(row['debug[5]']),
                d_gain=int(row['debug[6]']),
                f_gain=int(row['debug[7]']),
            )
            changes[gain_name].append(trans)
    
    return changes

def compute_axis_results(df: pd.DataFrame, transitions: List[Transition]) -> Dict[int, AxisTuneResult]:
    """Compute tuning results per axis."""
    results = {}
    
    for axis in range(3):  # Roll, Pitch, Yaw
        # Find when this axis was being tuned (states 4-7)
        axis_df = df[(df['debug[0]'].isin([4, 5, 6, 7])) & (df['debug[1]'] == axis)]
        
        if len(axis_df) == 0:
            continue
        
        # Initial and final gains
        initial_p = int(axis_df.iloc[0]['debug[5]'])
        initial_d = int(axis_df.iloc[0]['debug[6]'])
        initial_f = int(axis_df.iloc[0]['debug[7]'])
        final_p = int(axis_df.iloc[-1]['debug[5]'])
        final_d = int(axis_df.iloc[-1]['debug[6]'])
        final_f = int(axis_df.iloc[-1]['debug[7]'])
        
        # Event count (reason code 1 = EVENT_DETECTED)
        event_count = len(axis_df[axis_df['debug[2]'] == 1])
        
        # Duration
        duration_s = axis_df['time_rel'].max() - axis_df['time_rel'].min()
        
        # Check for convergence (reached AXIS_COMPLETE reason)
        converged = 6 in axis_df['debug[2]'].values
        
        # Overshoot (debug[4] / 10 = percentage)
        overshoot_vals = axis_df['debug[4]'] / 10.0
        overshoot_initial = overshoot_vals.iloc[:min(10, len(overshoot_vals))].mean()
        overshoot_final = overshoot_vals.iloc[-min(10, len(overshoot_vals)):].mean()
        
        results[axis] = AxisTuneResult(
            axis=axis,
            initial_p=initial_p,
            initial_d=initial_d,
            initial_f=initial_f,
            final_p=final_p,
            final_d=final_d,
            final_f=final_f,
            event_count=event_count,
            duration_s=duration_s,
            converged=converged,
            overshoot_initial=overshoot_initial,
            overshoot_final=overshoot_final,
        )
    
    return results

def analyze_log(df: pd.DataFrame, csv_path: str) -> TuneSummary:
    """Perform complete log analysis."""
    transitions = find_state_transitions(df)
    axis_results = compute_axis_results(df, transitions)
    
    # Final state
    final_state = int(df['debug[0]'].iloc[-1])
    
    # Check for abort
    abort_reason = None
    for trans in transitions:
        if trans.reason >= 30 and trans.reason <= 35:
            abort_reason = trans.reason
            break
    
    # Total events
    total_events = len(df[df['debug[2]'] == 1])
    
    # Warnings
    warnings = []
    
    # Check for stuck states
    for state in range(9):
        state_df = df[df['debug[0]'] == state]
        if len(state_df) > 0:
            duration = state_df['time_rel'].max() - state_df['time_rel'].min()
            if duration > 60 and state not in [0, 8]:  # Not IDLE or COMPLETE
                warnings.append(f"State {STATE_NAMES.get(state, state)} lasted {duration:.1f}s (>60s)")
    
    # Check for excessive events
    if total_events > 30:
        warnings.append(f"Total events {total_events} exceeds target of 30")
    
    # Check for quality gate failures
    quality_failures = df[df['debug[2]'].isin(range(11, 18))]
    if len(quality_failures) > total_events * 0.5:
        warnings.append(f"High quality gate failure rate: {len(quality_failures)} failures")
    
    return TuneSummary(
        log_file=csv_path,
        duration_s=df['time_rel'].max(),
        final_state=final_state,
        axes_completed=list(axis_results.keys()),
        axis_results=axis_results,
        total_events=total_events,
        abort_reason=abort_reason,
        warnings=warnings,
    )

# =============================================================================
# Plotting Functions
# =============================================================================

def plot_state_timeline(df: pd.DataFrame, output_path: str):
    """Generate state timeline overlay plot."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    # Plot 1: State over time
    ax1 = axes[0]
    ax1.plot(df['time_rel'], df['debug[0]'], 'b-', linewidth=0.5, alpha=0.7)
    ax1.set_ylabel('State')
    ax1.set_yticks(range(9))
    ax1.set_yticklabels([STATE_NAMES.get(i, str(i)) for i in range(9)], fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Autotune V2 State Timeline')
    
    # Mark transitions
    transitions = find_state_transitions(df)
    for trans in transitions:
        ax1.axvline(x=trans.time_s, color='gray', linestyle=':', alpha=0.3)
    
    # Plot 2: Axis over time
    ax2 = axes[1]
    ax2.plot(df['time_rel'], df['debug[1]'], 'g-', linewidth=0.5, alpha=0.7)
    ax2.set_ylabel('Axis')
    ax2.set_yticks(range(3))
    ax2.set_yticklabels([AXIS_NAMES.get(i, str(i)) for i in range(3)])
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Reason code
    ax3 = axes[2]
    ax3.plot(df['time_rel'], df['debug[2]'], 'r-', linewidth=0.5, alpha=0.7)
    ax3.set_ylabel('Reason Code')
    ax3.set_xlabel('Time (s)')
    ax3.grid(True, alpha=0.3)
    
    # Add reason code color bands
    ax3.axhspan(0, 10, alpha=0.1, color='green', label='Normal')
    ax3.axhspan(11, 19, alpha=0.1, color='yellow', label='Quality Fail')
    ax3.axhspan(20, 29, alpha=0.1, color='orange', label='Safety')
    ax3.axhspan(30, 35, alpha=0.1, color='red', label='Abort')
    ax3.legend(loc='upper right', fontsize=8)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"  Saved: {output_path}")
    plt.close()

def plot_gains_progression(df: pd.DataFrame, output_path: str):
    """Generate P/D/F gain progression plot."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    # Plot 1: P Gain
    ax1 = axes[0]
    ax1.plot(df['time_rel'], df['debug[5]'], 'b-', linewidth=1, label='P Gain')
    ax1.set_ylabel('P Gain')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right')
    ax1.set_title('Autotune V2 Gain Progression')
    
    # Plot 2: D Gain
    ax2 = axes[1]
    ax2.plot(df['time_rel'], df['debug[6]'], 'r-', linewidth=1, label='D Gain')
    ax2.set_ylabel('D Gain')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right')
    
    # Plot 3: F Gain
    ax3 = axes[2]
    ax3.plot(df['time_rel'], df['debug[7]'], 'g-', linewidth=1, label='F Gain')
    ax3.set_ylabel('F Gain')
    ax3.set_xlabel('Time (s)')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right')
    
    # Color background by axis
    for ax in axes:
        for axis, color in [(0, 'blue'), (1, 'orange'), (2, 'green')]:
            axis_mask = df['debug[1]'] == axis
            if axis_mask.any():
                for start, end in _get_contiguous_regions(df, axis_mask):
                    ax.axvspan(start, end, alpha=0.05, color=color)
    
    # Add legend for axis colors
    legend_elements = [
        Patch(facecolor='blue', alpha=0.2, label='Roll'),
        Patch(facecolor='orange', alpha=0.2, label='Pitch'),
        Patch(facecolor='green', alpha=0.2, label='Yaw'),
    ]
    ax1.legend(handles=legend_elements, loc='upper left', fontsize=8)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"  Saved: {output_path}")
    plt.close()

def _get_contiguous_regions(df: pd.DataFrame, mask: pd.Series) -> List[Tuple[float, float]]:
    """Get contiguous time regions where mask is True."""
    regions = []
    in_region = False
    start = 0
    
    for idx in mask.index:
        if mask.loc[idx] and not in_region:
            start = df.loc[idx, 'time_rel']
            in_region = True
        elif not mask.loc[idx] and in_region:
            end = df.loc[idx, 'time_rel']
            regions.append((start, end))
            in_region = False
    
    if in_region:
        regions.append((start, df['time_rel'].iloc[-1]))
    
    return regions

def plot_metrics_trace(df: pd.DataFrame, output_path: str):
    """Generate overshoot and metrics trace plot."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    
    # Plot 1: Overshoot
    ax1 = axes[0]
    overshoot = df['debug[4]'] / 10.0  # Scale from ×10
    ax1.plot(df['time_rel'], overshoot, 'b-', linewidth=0.5, alpha=0.7, label='Overshoot %')
    ax1.axhline(y=5, color='g', linestyle='--', linewidth=1, label='Target Low (5%)')
    ax1.axhline(y=10, color='r', linestyle='--', linewidth=1, label='Target High (10%)')
    ax1.axhspan(5, 10, alpha=0.1, color='green')
    ax1.set_ylabel('Overshoot (%)')
    ax1.set_ylim(0, max(30, overshoot.max() * 1.1))
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right')
    ax1.set_title('Autotune V2 Metrics Trace')
    
    # Plot 2: Decision (delta)
    ax2 = axes[1]
    ax2.plot(df['time_rel'], df['debug[3]'], 'purple', linewidth=0.5, alpha=0.7, label='Decision/Delta')
    ax2.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
    ax2.set_ylabel('Decision')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"  Saved: {output_path}")
    plt.close()

def plot_cross_axis_check(df: pd.DataFrame, output_path: str):
    """Generate cross-axis regression check plot."""
    # Check if we have rate data
    rate_cols = ['gyroADC[0]', 'gyroADC[1]', 'gyroADC[2]']
    sp_cols = ['setpoint[0]', 'setpoint[1]', 'setpoint[2]']
    
    has_rates = all(col in df.columns for col in rate_cols)
    has_setpoints = all(col in df.columns for col in sp_cols)
    
    if not has_rates:
        print(f"  Warning: Rate columns not found, using debug channels only")
        # Fallback: just show axis state over time
        fig, ax = plt.subplots(figsize=(14, 6))
        ax.plot(df['time_rel'], df['debug[1]'], 'b-', linewidth=0.5)
        ax.set_ylabel('Active Axis')
        ax.set_xlabel('Time (s)')
        ax.set_title('Cross-Axis Check (rate data not available)')
        plt.tight_layout()
        plt.savefig(output_path, dpi=150)
        print(f"  Saved: {output_path}")
        plt.close()
        return
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    for i, (ax, axis_name) in enumerate(zip(axes, ['Roll', 'Pitch', 'Yaw'])):
        rate = pd.to_numeric(df[rate_cols[i]], errors='coerce')
        if has_setpoints:
            setpoint = pd.to_numeric(df[sp_cols[i]], errors='coerce')
            error = (rate - setpoint).abs()
        else:
            error = rate.abs()
        
        ax.plot(df['time_rel'], error.rolling(100).mean(), linewidth=0.5, alpha=0.7, label=f'{axis_name} Error')
        ax.set_ylabel(f'{axis_name} Error')
        ax.grid(True, alpha=0.3)
        
        # Highlight when OTHER axes are being tuned
        for other_axis in range(3):
            if other_axis != i:
                mask = (df['debug[0]'].isin([4, 5, 6, 7])) & (df['debug[1]'] == other_axis)
                color = ['blue', 'orange', 'green'][other_axis]
                for start, end in _get_contiguous_regions(df, mask):
                    ax.axvspan(start, end, alpha=0.1, color=color)
    
    axes[0].set_title('Cross-Axis Regression Check')
    axes[2].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"  Saved: {output_path}")
    plt.close()

# =============================================================================
# Summary Output
# =============================================================================

def print_summary(summary: TuneSummary):
    """Print formatted analysis summary."""
    print("\n" + "=" * 70)
    print("AUTOTUNE V2 LOG ANALYSIS REPORT")
    print("=" * 70)
    
    print(f"\nLog: {summary.log_file}")
    print(f"Duration: {summary.duration_s:.1f} seconds")
    print(f"Final State: {STATE_NAMES.get(summary.final_state, summary.final_state)}")
    
    if summary.abort_reason:
        print(f"⚠️  ABORTED: {REASON_NAMES.get(summary.abort_reason, summary.abort_reason)}")
    
    print(f"\nTotal Events: {summary.total_events}")
    print(f"Axes Completed: {[AXIS_NAMES.get(a, a) for a in summary.axes_completed]}")
    
    print("\n" + "-" * 70)
    print("PER-AXIS RESULTS")
    print("-" * 70)
    
    for axis, result in summary.axis_results.items():
        status = "✅ CONVERGED" if result.converged else "❌ NOT CONVERGED"
        print(f"\n{AXIS_NAMES.get(axis, axis)} Axis: {status}")
        print(f"  Initial: P={result.initial_p}, D={result.initial_d}, F={result.initial_f}")
        print(f"  Final:   P={result.final_p}, D={result.final_d}, F={result.final_f}")
        print(f"  Events: {result.event_count}, Duration: {result.duration_s:.1f}s")
        print(f"  Overshoot: {result.overshoot_initial:.1f}% → {result.overshoot_final:.1f}%")
    
    if summary.warnings:
        print("\n" + "-" * 70)
        print("⚠️  WARNINGS")
        print("-" * 70)
        for warning in summary.warnings:
            print(f"  • {warning}")
    
    # Verdict
    print("\n" + "=" * 70)
    if summary.final_state == 8 and not summary.abort_reason:
        print("✅ OVERALL: SUCCESS")
    elif summary.abort_reason:
        print(f"❌ OVERALL: ABORTED ({REASON_NAMES.get(summary.abort_reason, summary.abort_reason)})")
    else:
        print("⚠️  OVERALL: INCOMPLETE")
    print("=" * 70)

def export_summary_json(summary: TuneSummary, output_path: str):
    """Export summary as JSON for programmatic use."""
    data = {
        'log_file': summary.log_file,
        'duration_s': summary.duration_s,
        'final_state': summary.final_state,
        'final_state_name': STATE_NAMES.get(summary.final_state, str(summary.final_state)),
        'axes_completed': summary.axes_completed,
        'total_events': summary.total_events,
        'abort_reason': summary.abort_reason,
        'abort_reason_name': REASON_NAMES.get(summary.abort_reason, '') if summary.abort_reason else None,
        'warnings': summary.warnings,
        'axis_results': {
            AXIS_NAMES.get(k, str(k)): {
                'initial_p': v.initial_p,
                'initial_d': v.initial_d,
                'initial_f': v.initial_f,
                'final_p': v.final_p,
                'final_d': v.final_d,
                'final_f': v.final_f,
                'event_count': v.event_count,
                'duration_s': v.duration_s,
                'converged': v.converged,
                'overshoot_initial': v.overshoot_initial,
                'overshoot_final': v.overshoot_final,
            }
            for k, v in summary.axis_results.items()
        },
    }
    
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"  Saved: {output_path}")

# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Autotune V2 Log Analysis Suite',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('csv_file', help='Path to exported Blackbox CSV file')
    parser.add_argument('--timeline', action='store_true', help='Generate state timeline plot')
    parser.add_argument('--gains', action='store_true', help='Generate gain progression plot')
    parser.add_argument('--metrics', action='store_true', help='Generate metrics trace plot')
    parser.add_argument('--crossaxis', action='store_true', help='Generate cross-axis check plot')
    parser.add_argument('--summary', action='store_true', help='Print summary report')
    parser.add_argument('--all', action='store_true', help='Generate all outputs')
    parser.add_argument('--output-dir', help='Output directory for plots')
    parser.add_argument('--json', action='store_true', help='Export summary as JSON')
    
    args = parser.parse_args()
    
    if not any([args.timeline, args.gains, args.metrics, args.crossaxis, args.summary, args.all, args.json]):
        args.all = True  # Default to all
    
    if args.all:
        args.timeline = args.gains = args.metrics = args.crossaxis = args.summary = args.json = True
    
    # Load data
    df = load_log(args.csv_file)
    
    # Determine output directory
    if args.output_dir:
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
    else:
        output_dir = Path(args.csv_file).parent
    
    base_name = Path(args.csv_file).stem
    
    # Run analysis
    summary = analyze_log(df, args.csv_file)
    
    # Generate outputs
    if args.summary:
        print_summary(summary)
    
    if args.timeline:
        plot_state_timeline(df, str(output_dir / f"{base_name}_state_timeline.png"))
    
    if args.gains:
        plot_gains_progression(df, str(output_dir / f"{base_name}_gains.png"))
    
    if args.metrics:
        plot_metrics_trace(df, str(output_dir / f"{base_name}_metrics.png"))
    
    if args.crossaxis:
        plot_cross_axis_check(df, str(output_dir / f"{base_name}_crossaxis.png"))
    
    if args.json:
        export_summary_json(summary, str(output_dir / f"{base_name}_summary.json"))
    
    print("\nAnalysis complete.")

if __name__ == '__main__':
    main()
