#!/usr/bin/env python3
"""Analyze blackbox logs to understand typical noise profiles."""

import os
import statistics

# Analyze all CSV logs to understand typical noise profiles
logs = [f for f in os.listdir('.') if f.endswith('.bbl.csv')]
print(f'Found {len(logs)} log files\n')

for logfile in sorted(logs)[-15:]:  # Last 15 logs
    try:
        with open(logfile, 'r') as f:
            lines = f.readlines()
        
        # Find header line
        header_idx = None
        for i, line in enumerate(lines):
            if 'loopIteration' in line:
                header_idx = i
                break
        
        if header_idx is None:
            continue
            
        headers = lines[header_idx].strip().replace('"', '').split(',')
        
        # Find column indices
        gyro_idx = headers.index('gyroADC[0]') if 'gyroADC[0]' in headers else None
        dterm_idx = headers.index('axisD[0]') if 'axisD[0]' in headers else None
        debug0_idx = headers.index('debug[0]') if 'debug[0]' in headers else None
        debug4_idx = headers.index('debug[4]') if 'debug[4]' in headers else None
        debug5_idx = headers.index('debug[5]') if 'debug[5]' in headers else None
        debug7_idx = headers.index('debug[7]') if 'debug[7]' in headers else None
        
        if gyro_idx is None:
            continue
        
        # Collect data
        gyro_vals = []
        dterm_vals = []
        dterm_amps = []  # debug[5] / 10
        overshoots = []  # debug[4] / 10
        
        for line in lines[header_idx+1:]:
            cols = line.strip().split(',')
            if len(cols) > max(gyro_idx, debug5_idx or 0, dterm_idx or 0):
                try:
                    gyro = float(cols[gyro_idx])
                    gyro_vals.append(abs(gyro))
                    
                    if dterm_idx and cols[dterm_idx]:
                        dterm = float(cols[dterm_idx])
                        dterm_vals.append(abs(dterm))
                    
                    if debug5_idx and cols[debug5_idx]:
                        damp = float(cols[debug5_idx]) / 10.0
                        if damp > 0 and damp < 500:
                            dterm_amps.append(damp)
                    
                    if debug4_idx and cols[debug4_idx]:
                        ovs = float(cols[debug4_idx]) / 10.0
                        if ovs > -50 and ovs < 100:
                            overshoots.append(ovs)
                except:
                    pass
        
        if gyro_vals:
            print(f'{logfile}:')
            print(f'  Gyro: mean={statistics.mean(gyro_vals):.1f}, median={statistics.median(gyro_vals):.1f}, max={max(gyro_vals):.1f}, stdev={statistics.stdev(gyro_vals):.1f}')
            if dterm_vals:
                print(f'  D-term raw: mean={statistics.mean(dterm_vals):.1f}, median={statistics.median(dterm_vals):.1f}, max={max(dterm_vals):.1f}, p95={sorted(dterm_vals)[int(len(dterm_vals)*0.95)]:.1f}')
            if dterm_amps:
                print(f'  D-amp (debug5): mean={statistics.mean(dterm_amps):.1f}, median={statistics.median(dterm_amps):.1f}, max={max(dterm_amps):.1f}')
            if overshoots:
                print(f'  Overshoot (debug4): mean={statistics.mean(overshoots):.1f}, median={statistics.median(overshoots):.1f}')
            print()
    except Exception as e:
        print(f'{logfile}: Error - {e}')
