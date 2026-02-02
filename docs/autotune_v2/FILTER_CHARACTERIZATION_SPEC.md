# Filter Characterization Phase - Technical Specification

**Version:** 1.1  
**Date:** January 25, 2026  
**Status:** Phase 1 Implemented  
**Purpose:** Define requirements for THROTTLE_SWEEP filter analysis

---

## 1. Problem Statement

### 1.1 Current Implementation Gap

The current THROTTLE_SWEEP phase:
- ✅ Detects throttle sweep patterns (pilot readiness)
- ✅ Measures time-domain gyro noise (RMS)
- ❌ Does NOT perform frequency analysis
- ❌ Does NOT compare filtered vs unfiltered gyro
- ❌ Does NOT check motor output spectrum
- ❌ Does NOT actually apply any filter changes
- ❌ Cannot determine if filtering should be RELAXED

### 1.2 What We Actually Need

Per PRD Section 6.2, the purpose is to:
> "Characterize noise vs throttle, adjust gyro/D-term filters + notch, verify improvement"

The key insight from flight test FFT analysis:
- Unfiltered gyro shows motor harmonic peaks (e.g., 101Hz, 163Hz)
- Filtered gyro shows these peaks removed
- Motor output shows minimal peak propagation, healthy exponential decay
- **Conclusion: Current filtering may be SUFFICIENT or even OVER-AGGRESSIVE**

The algorithm must determine:
1. Are motor harmonics being filtered effectively?
2. Is noise propagating to motor outputs?
3. Could we RELAX filtering for better response?

---

## 2. Existing Dynamic Notch Infrastructure

### 2.1 Available Components

Betaflight already has sophisticated frequency analysis in `dyn_notch_filter.c`:

| Component | Purpose | Location |
|-----------|---------|----------|
| **SDFT** | Sliding DFT for real-time FFT | `common/sdft.c` |
| **Peak Detection** | Find N biggest peaks in spectrum | `dyn_notch_filter.c:STEP_DETECT_PEAKS` |
| **Noise Floor** | Calculate average power spectral density | `dyn_notch_filter.c:STEP_CALC_FREQUENCIES` |
| **Center Frequencies** | Track peak frequencies per axis | `dynNotch.centerFreq[axis][p]` |

### 2.2 SDFT Details

```c
#define SDFT_SAMPLE_SIZE 72    // Input buffer size
#define SDFT_BIN_COUNT   36    // Frequency bins (0-Nyquist)

// Resolution depends on sample rate:
// At 8kHz PID, with 600Hz max: sdftResolutionHz = 18.5Hz per bin
// Bin 1 = 18.5Hz, Bin 2 = 37Hz, ... Bin 35 = 648Hz
```

### 2.3 Exposed Data We Can Read

| Data | Access Method | Notes |
|------|---------------|-------|
| Peak frequencies | `getMaxFFT()` | Only returns max across all axes |
| Notch active | `isDynNotchActive()` | Boolean check |
| Debug FFT_FREQ | `DEBUG_SET(DEBUG_FFT_FREQ, p+1, freq)` | Up to 7 center frequencies |

### 2.4 Data We Need But Isn't Exposed

| Data Needed | Current State | Solution |
|-------------|---------------|----------|
| Per-axis peak frequencies | Internal to `dynNotch_t` | Add getter function |
| Per-axis peak amplitudes | Computed in `peaks[]` but not stored | Add storage + getter |
| Noise floor per axis | Computed but discarded | Store and expose |
| SDFT raw bins | Internal to `sdft[]` | Could add getter |

---

## 3. Proposed Algorithm

### 3.1 High-Level Flow

```
THROTTLE_SWEEP Phase:
┌─────────────────────────────────────────────────────────────────┐
│ 1. COLLECT (during throttle sweeps)                             │
│    - Read dynamic notch peak frequencies                        │
│    - Sample gyroUnfilt[] spectrum                               │
│    - Sample gyroADC[] spectrum (filtered)                       │
│    - Sample motor[] commands                                    │
│    - Track all vs throttle level                                │
│                                                                 │
│ 2. ANALYZE (after sweep completion)                             │
│    - Identify motor harmonic frequencies from peaks             │
│    - Compare unfiltered vs filtered: is LPF effective?          │
│    - Check motor output: is noise propagating?                  │
│    - Measure broadband noise floor                              │
│                                                                 │
│ 3. DECIDE                                                       │
│    - SUFFICIENT: peaks removed, motors clean → proceed          │
│    - TIGHTEN: peaks still in motors → lower LPF / add notch     │
│    - RELAX: very clean everywhere → raise LPF for response      │
│                                                                 │
│ 4. APPLY (if changes needed)                                    │
│    - Adjust gyro LPF cutoff                                     │
│    - Adjust D-term LPF cutoff                                   │
│    - Enable/configure notch filters                             │
│                                                                 │
│ 5. VERIFY (optional repeat sweep)                               │
│    - Confirm improvement or rollback                            │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 Leveraging Dynamic Notch

**Option A: Read Dynamic Notch Results (Minimal Changes)**

The dynamic notch is already running and tracking motor harmonics. We can:

```c
// In autotune_filter.c, during THROTTLE_SWEEP:

// 1. Check if dynamic notch is active
if (isDynNotchActive()) {
    // 2. Read the max peak frequency it's tracking
    int maxPeakHz = getMaxFFT();
    
    // 3. Compare to our LPF cutoff
    float gyroLpfCutoff = gyroConfig()->gyro_lpf1_static_hz;
    
    // If peak is well below LPF, filtering is effective
    if (maxPeakHz < gyroLpfCutoff * 0.7f) {
        // LPF is cutting above the noise - could potentially relax
    }
}
```

**Limitation:** Only gives us peak frequency, not amplitude or effectiveness.

**Option B: Add Getters to Dynamic Notch (Medium Changes)**

Expose more data from `dyn_notch_filter.c`:

```c
// New functions to add to dyn_notch_filter.h:

// Get center frequency for specific axis and peak index
float getDynNotchCenterFreq(int axis, int peakIndex);

// Get the noise floor threshold (indicates overall noise level)
float getDynNotchNoiseFloor(void);

// Get peak value/amplitude for specific axis and peak
float getDynNotchPeakValue(int axis, int peakIndex);
```

**Option C: Run Our Own SDFT (Maximum Flexibility)**

Use the existing SDFT infrastructure but run our own analysis:

```c
// In autotune_filter.c:
#include "common/sdft.h"

static sdft_t autotuneSDFT[XYZ_AXIS_COUNT];
static float sdftOutput[SDFT_BIN_COUNT];

// During throttle sweep, collect samples
void autotuneFilterSampleGyro(int axis, float unfilteredGyro, float filteredGyro) {
    sdftPush(&autotuneSDFT[axis], unfilteredGyro);
}

// After sweep, analyze spectrum
void autotuneFilterAnalyzeSpectrum(int axis) {
    sdftMagSq(&autotuneSDFT[axis], sdftOutput);
    
    // Find peaks, compare to filtered, make decision...
}
```

---

## 4. Decision Logic

### 4.1 Key Metrics

| Metric | Description | Source |
|--------|-------------|--------|
| **Peak Frequencies** | Motor harmonic locations | From SDFT or dynamic notch |
| **Unfiltered Peak Amplitude** | Raw noise at peak frequency | SDFT of gyroUnfilt |
| **Filtered Peak Amplitude** | Noise after LPF at same frequency | SDFT of gyroADC |
| **Motor Peak Amplitude** | Noise in motor commands at peak freq | SDFT of motor output |
| **Attenuation Ratio** | Filtered/Unfiltered at peak | Computed |
| **Motor Propagation** | Motor amplitude at peak frequency | Indicates PID noise amplification |

### 4.2 Decision Tree

```c
typedef enum {
    FILTER_DECISION_SUFFICIENT,   // No changes needed
    FILTER_DECISION_TIGHTEN,      // Need more filtering
    FILTER_DECISION_RELAX,        // Can reduce filtering for response
    FILTER_DECISION_ADD_NOTCH,    // Need notch at specific frequency
} filterDecision_e;

filterDecision_e analyzeFilterEffectiveness(void) {
    // Get peak data from dynamic notch or our own SDFT
    float peakFreqHz = getPrimaryPeakFrequency();
    float unfiltAmp = getUnfilteredAmplitudeAt(peakFreqHz);
    float filtAmp = getFilteredAmplitudeAt(peakFreqHz);
    float motorAmp = getMotorAmplitudeAt(peakFreqHz);
    
    float attenuation = filtAmp / unfiltAmp;  // < 1.0 means filtering helps
    float motorPropagation = motorAmp / filtAmp;  // < 1.0 means PID not amplifying
    
    // Decision logic
    if (attenuation < 0.3f && motorPropagation < 0.5f) {
        // Peaks well attenuated, motors clean
        // Check if we're over-filtering (LPF very low compared to peak)
        if (currentLpfHz < peakFreqHz * 0.5f) {
            return FILTER_DECISION_RELAX;  // Can raise LPF for better response
        }
        return FILTER_DECISION_SUFFICIENT;
    }
    
    if (attenuation > 0.7f) {
        // LPF not cutting enough at peak frequency
        if (peakFreqHz < 200.0f) {
            return FILTER_DECISION_ADD_NOTCH;  // Low frequency peak, use notch
        }
        return FILTER_DECISION_TIGHTEN;  // Lower LPF cutoff
    }
    
    if (motorPropagation > 1.0f) {
        // PID is amplifying noise into motors
        return FILTER_DECISION_TIGHTEN;
    }
    
    return FILTER_DECISION_SUFFICIENT;
}
```

### 4.3 Visual Interpretation (From Flight Test FFTs)

Your images showed:

| Observation | Interpretation | Decision |
|-------------|----------------|----------|
| Unfiltered gyro: peak at 150Hz | Motor harmonic present | - |
| Filtered gyro: peak nearly gone | LPF is effective | Sufficient or Relax |
| Motor output: small peak, exponential decay | PID not amplifying | Sufficient |
| GYRO LPF: 250-500Hz dynamic | Cutting well above 150Hz | Could potentially relax |

---

## 5. Implementation Phases

### Phase 1: Read-Only (Minimal Risk) ✅ IMPLEMENTED

1. ✅ Added getter functions to `dyn_notch_filter.c` to expose:
   - `dynNotchGetCenterFreq(axis, peakIndex)` - Peak center frequencies per axis
   - `dynNotchGetNoiseThreshold()` - Noise level estimate
   - `dynNotchHasValidPeak(axis, peakIndex)` - Check if peak is being tracked
   - `dynNotchGetResolutionHz()` - SDFT frequency resolution
   - `dynNotchGetMinHz()` / `dynNotchGetMaxHz()` - Frequency bounds
   - `dynNotchGetCount()` - Number of notches per axis
   
2. ✅ `readDynNotchPeaks()` reads peak data during throttle sweep

3. ✅ Peak frequency data stored in `filterCharState_t`

4. ✅ `autotuneFilterComputeRecommendations()` uses smart logic:
   - Peak-based: LPF = maxPeakFreq × 1.3 headroom
   - Noise-based: thresholds at 40/20 deg/s → 150/200/250 Hz
   - Takes minimum of both for safety

**Deliverables (Completed):**
- ✅ New functions in `dyn_notch_filter.h/c` (~77 lines added)
- ✅ Updated `autotuneFilterComputeRecommendations()` using real dynamic notch data
- ✅ Extended `filterCharState_t` with `peakFreq[][]`, `peakCount[]`, `maxPeakFreq`, `minPeakFreq`
- ✅ Added `autotuneFilterGetPrimaryPeakHz()`, `autotuneFilterGetMaxPeakHz()`, `autotuneFilterHasDynNotchData()`
- ⏳ Debug output showing analysis results (pending flight test verification)

### Phase 2: Apply Filter Changes

1. Implement `autotuneFilterApplyRecommendations()` to actually modify:
   - `gyroConfig()->gyro_lpf1_static_hz`
   - `gyroConfig()->gyro_lpf1_dyn_min_hz`
   - `gyroConfig()->gyro_lpf1_dyn_max_hz`
   - D-term LPF settings

2. Store previous values for rollback

3. Trigger filter re-initialization

### Phase 3: Verification Loop

1. After applying changes, wait for filters to settle

2. Re-run analysis to confirm improvement

3. If worse, rollback and try alternative

---

## 6. Data Structures

### 6.1 Filter Characterization State (Extended)

```c
typedef struct filterCharState_s {
    // Existing fields...
    float minThrottleSeen;
    float maxThrottleSeen;
    uint8_t sweepCount;
    
    // NEW: Frequency analysis results
    float peakFrequencies[XYZ_AXIS_COUNT][3];     // Top 3 peaks per axis
    float peakAmplitudes[XYZ_AXIS_COUNT][3];      // Amplitude of each peak
    float attenuationRatio[XYZ_AXIS_COUNT][3];    // Filtered/Unfiltered
    float motorPropagation[XYZ_AXIS_COUNT][3];    // Motor amp at peak freq
    float noiseFloor[XYZ_AXIS_COUNT];             // Broadband noise level
    
    // NEW: Analysis results
    filterDecision_e decision;
    float recommendedGyroLpf;
    float recommendedDtermLpf;
    float recommendedNotchFreq;    // 0 if no notch needed
    
    // Rollback state
    float previousGyroLpf;
    float previousDtermLpf;
    
} filterCharState_t;
```

### 6.2 New Functions to Add to dyn_notch_filter

```c
// In dyn_notch_filter.h:

// Get center frequency for specific axis and peak (0 = primary)
float dynNotchGetCenterFreq(int axis, int peakIndex);

// Get number of active peaks for axis
int dynNotchGetPeakCount(int axis);

// Get the current noise floor estimate
float dynNotchGetNoiseFloor(void);

// Get raw SDFT bin data for external analysis
const float* dynNotchGetSdftData(int axis);
```

---

## 7. Debug Output

### 7.1 During THROTTLE_SWEEP

| Channel | Content | Scale |
|---------|---------|-------|
| debug[0] | State (always 2) | - |
| debug[1] | Current axis being analyzed | 0/1/2 |
| debug[2] | Primary peak frequency Hz | Direct |
| debug[3] | Peak attenuation × 100 | % (lower = better) |
| debug[4] | Throttle range seen × 100 | % |
| debug[5] | Motor propagation × 100 | % (lower = better) |
| debug[6] | Sweep count | - |
| debug[7] | Decision code | 0=Sufficient, 1=Tighten, 2=Relax, 3=Notch |

### 7.2 Blackbox Analysis

With proper debug output, blackbox analysis should show:
- Peak frequencies varying with throttle (motor RPM changes)
- Attenuation consistently low if filtering effective
- Motor propagation low in healthy setup

---

## 8. Acceptance Criteria

| Criterion | Target |
|-----------|--------|
| Correctly identifies motor harmonic peaks | Match dynamic notch detection |
| Detects when filtering is effective | Attenuation < 0.3 at peaks |
| Detects when filtering is insufficient | Motor propagation > 1.0 |
| Can recommend RELAXING filters | When over-filtered and response suffers |
| Applies changes safely | With rollback on regression |
| Completes within throttle sweep duration | No additional flight time |

---

## 9. Flight Test Verification Plan

The following flight tests validate filter characterization functionality. See [VERIFICATION.md](VERIFICATION.md) for full test procedures.

### 9.1 Phase 1 Verification (Read-Only)

| Test ID | Description | Status |
|---------|-------------|--------|
| FLT-01  | Throttle Sweep Detection | ✅ Existing |
| FLT-01a | Dynamic Notch Peak Frequency Reading | 🆕 Added |
| FLT-01b | Smart LPF Recommendation Calculation | 🆕 Added |
| FLT-01c | Peak Frequency vs Throttle Band Correlation | 🆕 Added |
| FLT-01d | Filter Characterization Fallback (No Dyn Notch) | 🆕 Added |

### 9.2 Test Flight Procedure for Filter Characterization

**Recommended Flight Pattern:**

```
1. ARM and take off to stable hover (3-5s)
2. Activate autotune switch
3. Wait for HOVER_LOCK wiggle confirmation (2-3s)
4. Perform deliberate throttle blip sequence:
   
   Blip 1: Quick punch hover → 80% → hover (0.5s up, 1s settle)
   Blip 2: Medium punch hover → 70% → hover (0.5s up, 1s settle)
   Blip 3: Full punch hover → 90%+ → hover (0.5s up, 2s settle)
   
5. Wait for "Filters Set" wiggle (advancing to state 3 or 4)
6. Continue with PID tuning flicks...
```

### 9.3 Blackbox Analysis Checklist

After flight, verify filter characterization with blackbox data:

```python
import pandas as pd

def analyze_filter_characterization(df):
    """Analyze filter characterization from blackbox log."""
    
    # 1. Check throttle sweep occurred
    sweep_state = df[df['debug[0]'] == 2]  # THROTTLE_SWEEP
    assert len(sweep_state) > 0, "FAIL: Never entered THROTTLE_SWEEP state"
    
    sweep_duration = (sweep_state['time'].max() - sweep_state['time'].min()) / 1e6
    print(f"✅ THROTTLE_SWEEP duration: {sweep_duration:.1f}s")
    
    # 2. Check throttle range during sweep
    sweep_throttle = sweep_state['rcCommand[3]'] if 'rcCommand[3]' in df else None
    if sweep_throttle is not None:
        throttle_min = (sweep_throttle.min() - 1000) / 1000 * 100
        throttle_max = (sweep_throttle.max() - 1000) / 1000 * 100
        throttle_range = throttle_max - throttle_min
        print(f"✅ Throttle range seen: {throttle_min:.0f}% - {throttle_max:.0f}% (range: {throttle_range:.0f}%)")
        
        if throttle_range < 30:
            print("⚠️ WARNING: Throttle range < 30% - may not adequately characterize motor noise")
    
    # 3. Check state progression
    states_seen = df['debug[0]'].unique()
    expected_progression = [1, 2, 3, 4]  # HOVER_LOCK → THROTTLE_SWEEP → NOISE_CONFIRM → PD_RATIO_SEEK
    
    for state in expected_progression:
        if state in states_seen:
            print(f"✅ Reached state {state}")
        else:
            print(f"❌ Never reached state {state}")
            break
    
    # 4. Check for filter recommendations (when debug channels are available)
    # Future: Add debug channel for recommendedLpf, maxPeakFreq
    
    return True

# Usage: analyze_filter_characterization(df)
```

### 9.4 Expected Debug Channel Mapping

During THROTTLE_SWEEP (debug[0]=2), debug channels should show:

| Channel | Content | Expected Range | Notes |
|---------|---------|----------------|-------|
| debug[0] | State = 2 | 2 | THROTTLE_SWEEP |
| debug[1] | Current axis (0=Roll) | 0-2 | Usually 0 during filter char |
| debug[2] | Primary peak freq (Hz) | 80-300 | From dynamic notch |
| debug[3] | Max peak freq × 10 | 800-3000 | Highest detected |
| debug[4] | Throttle range × 100 | 0-100 | % of range seen |
| debug[5] | Sweep count | 0-5 | Number of complete sweeps |
| debug[6] | Noise level × 10 | 0-500 | deg/s × 10 |
| debug[7] | Recommended LPF | 100-400 | Hz |

*Note: Exact debug mapping depends on implementation - verify against source.*

---

## 10. Open Questions

1. **Dynamic Notch Dependency:** Should autotune require dynamic notch enabled? Or have its own SDFT?

2. **Motor Spectrum Access:** How to get frequency analysis of motor commands? May need to add SDFT for motor output.

3. **D-term vs Gyro:** Should we analyze D-term noise separately? D-term LPF has different requirements.

4. **Throttle Correlation:** How to handle that motor harmonic frequencies shift with throttle/RPM?

5. **Performance Impact:** Can we afford additional SDFT computation during flight?

---

## 11. References

- [dyn_notch_filter.c](../../src/main/flight/dyn_notch_filter.c) - Dynamic notch implementation
- [sdft.c](../../src/main/common/sdft.c) - Sliding DFT implementation
- [PRD.md](PRD.md) - Section 6.2 THROTTLE_SWEEP_FILTER_CHAR

---

*End of Specification*
