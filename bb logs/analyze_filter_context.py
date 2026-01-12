#!/usr/bin/env python3
"""Deep analysis of filter context around 9300"""
import pandas as pd
import numpy as np

# Read the log
df = pd.read_csv('btfl_netwon_2___12p4s.bbl.csv', skiprows=147)

# Calculate relative time from log start
log_start = df['time'].iloc[0]
df['rel_time'] = (df['time'] - log_start) / 1e6

# Find all transitions TO 9300
df['reason'] = df['debug[7]']
df['prev_reason'] = df['reason'].shift(1)

transitions_to_9300 = df[(df['reason'] == 9300) & (df['prev_reason'] != 9300)]

print('='*100)
print('TRANSITIONS INTO 9300 (AT_LIMIT)')
print('='*100)

for idx, row in transitions_to_9300.iterrows():
    print(f"\n--- Transition at {row['rel_time']:.3f}s ---")
    
    # Get context: 50ms before and after
    t = row['rel_time']
    context = df[(df['rel_time'] >= t - 0.2) & (df['rel_time'] <= t + 0.2)]
    
    print(f"{'Time':>10} {'State':>6} {'d1':>5} {'d2':>5} {'d3':>5} {'d4':>5} {'d5':>5} {'d6':>5} {'Reason':>7}")
    print('-'*80)
    
    prev_state = None
    prev_reason = None
    for _, r in context.iterrows():
        state = int(r['debug[0]'])
        reason = int(r['debug[7]'])
        
        # Only show if state or reason changed
        if state != prev_state or reason != prev_reason:
            marker = " <--" if reason == 9300 and prev_reason != 9300 else ""
            print(f"{r['rel_time']:10.3f} {state:6} {int(r['debug[1]']):5} {int(r['debug[2]']):5} {int(r['debug[3]']):5} {int(r['debug[4]']):5} {int(r['debug[5]']):5} {int(r['debug[6]']):5} {reason:7}{marker}")
            prev_state = state
            prev_reason = reason
    
    # Only show first transition
    break

# Now let's see what happens through the full filter tune cycle
print()
print('='*100)
print('FULL STATE SEQUENCE (showing all state/reason transitions)')
print('='*100)

df['state_change'] = df['debug[0]'].diff() != 0
df['reason_change'] = df['debug[7]'].diff() != 0

transitions = df[df['state_change'] | df['reason_change']]

STATE_NAMES = {0:'IDLE', 1:'ARMED', 2:'DETECT', 3:'COLLECT', 4:'SETTLE', 5:'ANALYZE', 6:'ADJUST', 7:'SIGNAL'}

print(f"{'Time':>8} {'State':>8} {'d1':>5} {'d2':>5} {'d3':>5} {'d4':>5} {'d5':>5} {'d6':>5} {'Reason':>7}")
print('-'*80)

for _, row in transitions.head(50).iterrows():
    state = int(row['debug[0]'])
    state_name = STATE_NAMES.get(state, f'{state}?')
    reason = int(row['debug[7]'])
    print(f"{row['rel_time']:8.3f} {state_name:>8} {int(row['debug[1]']):5} {int(row['debug[2]']):5} {int(row['debug[3]']):5} {int(row['debug[4]']):5} {int(row['debug[5]']):5} {int(row['debug[6]']):5} {reason:7}")

# Let's also check if there are any FILTER_COLLECT periods and what noise they measured
print()
print('='*100)
print('NOISE FLOOR DURING FILTER TUNE (d6 = noiseFloor * 10)')
print('='*100)

# Find periods in COLLECT state with reason 2001
collect_periods = df[(df['debug[0]'] == 3) & (df['debug[7]'] == 2001)]
if len(collect_periods) > 0:
    print(f"d[6] values during FILTER_COLLECT: min={collect_periods['debug[6]'].min()}, max={collect_periods['debug[6]'].max()}, mean={collect_periods['debug[6]'].mean():.1f}")
    print("This means noise floor ranges from {:.1f} to {:.1f} deg/s".format(
        collect_periods['debug[6]'].min()/10, collect_periods['debug[6]'].max()/10))
