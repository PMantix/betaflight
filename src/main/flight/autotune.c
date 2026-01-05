/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_AUTOTUNE

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"

#include "io/beeper.h"

#include "pg/autotune.h"

#include "sensors/gyro.h"

#include "autotune.h"

#define AUTOTUNE_SAMPLE_COUNT 200       // Number of samples to collect
#define AUTOTUNE_TARGET_SAMPLE_RATE_HZ 200  // Target sample rate (Hz) - 5ms between samples

// Runtime state
typedef struct autotuneRuntime_s {
    autotuneState_e state;
    autotuneState_e prevState;
    timeUs_t stateEnteredAt;
    uint8_t currentAxis;
    bool active;
    bool stateJustEntered;          // Flag set on state change, cleared after first execution
    
    // Measurement data
    timeUs_t excitationStartTime;
    float gyroHistory[AUTOTUNE_SAMPLE_COUNT];      // Gyro response (deg/s)
    float setpointHistory[AUTOTUNE_SAMPLE_COUNT];  // Commanded rate (deg/s)
    uint16_t sampleIndex;
    uint16_t sampleCount;
    uint16_t sampleCounter;         // Counter for sample interval
    uint16_t sampleIntervalLoops;   // Calculated sample interval based on PID frequency
    
} autotuneRuntime_t;

static autotuneRuntime_t runtime;

// State names for debugging
static const char* const stateNames[] = {
    "IDLE",
    "SETUP",
    "WAIT_STABLE",
    "EXCITE",
    "MEASURE",
    "ANALYZE",
    "ADJUST",
    "COMPLETE",
    "ABORTED"
};

// Initialize autotune system
void autotuneInit(void)
{
    memset(&runtime, 0, sizeof(runtime));
    runtime.state = AUTOTUNE_STATE_IDLE;
    runtime.prevState = AUTOTUNE_STATE_IDLE;
    runtime.active = false;
    runtime.currentAxis = FD_ROLL;  // Start with roll
    
    // Calculate sample interval based on actual PID loop frequency
    // Target: collect samples at AUTOTUNE_TARGET_SAMPLE_RATE_HZ (e.g., 200Hz = 5ms)
    // sampleInterval = pidFrequency / targetSampleRate
    // e.g., 8000Hz / 200Hz = 40 loops
    //       4000Hz / 200Hz = 20 loops
    const uint16_t pidFreq = pidGetPidFrequency();
    runtime.sampleIntervalLoops = pidFreq / AUTOTUNE_TARGET_SAMPLE_RATE_HZ;
    
    // Sanity check - ensure at least 1 loop interval
    if (runtime.sampleIntervalLoops < 1) {
        runtime.sampleIntervalLoops = 1;
    }
}

// Check if it's safe to run autotune
static bool autotuneSafetyCheck(void)
{
    // Check attitude limits (in decidegrees, so 300 = 30 degrees)
    const int safetyMarginDeci = autotuneConfig()->safety_margin * 10;
    
    if (ABS(attitude.raw[FD_ROLL]) > safetyMarginDeci ||
        ABS(attitude.raw[FD_PITCH]) > safetyMarginDeci) {
        DEBUG_SET(DEBUG_AUTOTUNE, 4, 1); // Attitude check failed
        return false;
    }
    
    // Check gyro rates not excessive (deg/sec)
    for (int axis = 0; axis < 3; axis++) {
        if (fabsf(gyro.gyroADCf[axis]) > 500.0f) {
            DEBUG_SET(DEBUG_AUTOTUNE, 4, 2); // Gyro rate check failed
            return false;
        }
    }
    
    // Check throttle is in reasonable range (hover)
    // Use Betaflight's proper throttle percentage function (returns 0-100)
    const int8_t throttlePercent = calculateThrottlePercent();
    DEBUG_SET(DEBUG_AUTOTUNE, 5, throttlePercent); // Show throttle %
    if (throttlePercent < 20 || throttlePercent > 80) {
        DEBUG_SET(DEBUG_AUTOTUNE, 4, 3); // Throttle check failed
        return false;
    }
    
    // Check no stick input (pilot not commanding) - relaxed to 20%
    const float maxStickDeflection = MAX(fabsf(getRcDeflection(FD_ROLL)), 
                                         MAX(fabsf(getRcDeflection(FD_PITCH)), 
                                             fabsf(getRcDeflection(FD_YAW))));
    DEBUG_SET(DEBUG_AUTOTUNE, 6, (int16_t)(maxStickDeflection * 100)); // Max stick %
    
    if (maxStickDeflection > 0.2f) {
        DEBUG_SET(DEBUG_AUTOTUNE, 4, 4); // Stick deflection check failed
        return false;
    }
    
    DEBUG_SET(DEBUG_AUTOTUNE, 4, 0); // All checks passed
    return true;
}

// Collect sample data
static void autotuneSampleData(float gyro, float setpoint)
{
    if (runtime.sampleIndex >= AUTOTUNE_SAMPLE_COUNT) {
        return;  // Buffer full
    }
    
    runtime.gyroHistory[runtime.sampleIndex] = gyro;
    runtime.setpointHistory[runtime.sampleIndex] = setpoint;
    runtime.sampleIndex++;
    runtime.sampleCount = runtime.sampleIndex;
}

// Generate bidirectional 3-2-1-1 doublet excitation (zero net displacement)
// Returns additional setpoint to add to pilot's command
float autotuneGetExcitation(timeUs_t currentTimeUs)
{
    if (runtime.state != AUTOTUNE_STATE_EXCITE) {
        return 0.0f;
    }
    
    const timeUs_t elapsed = cmpTimeUs(currentTimeUs, runtime.excitationStartTime);
    const uint32_t stepDurationUs = autotuneConfig()->step_duration_ms * 1000;
    const float amplitude = autotuneConfig()->step_amplitude;
    
    // Bidirectional doublet (6 phases): +A, -2A, +2A, -2A, +2A, -A
    // Net displacement: A - 2A + 2A - 2A + 2A - A = 0
    // Creates two strong symmetric swings
    if (elapsed < stepDurationUs) {
        return amplitude;                    // Phase 1: +A
    } else if (elapsed < stepDurationUs * 2) {
        return -2.0f * amplitude;            // Phase 2: -2A
    } else if (elapsed < stepDurationUs * 3) {
        return 2.0f * amplitude;             // Phase 3: +2A (strong transition)
    } else if (elapsed < stepDurationUs * 4) {
        return -2.0f * amplitude;            // Phase 4: -2A (strong opposite)
    } else if (elapsed < stepDurationUs * 5) {
        return 2.0f * amplitude;             // Phase 5: +2A
    } else if (elapsed < stepDurationUs * 6) {
        return -amplitude;                   // Phase 6: -A (settle to zero)
    }
    
    return 0.0f;  // Done
}

// Change state with logging
static void changeState(autotuneState_e newState, timeUs_t currentTimeUs)
{
    if (runtime.state != newState) {
        runtime.prevState = runtime.state;
        runtime.state = newState;
        runtime.stateEnteredAt = currentTimeUs;
        runtime.stateJustEntered = true;  // Set flag for state initialization
        
        DEBUG_SET(DEBUG_AUTOTUNE, 0, newState);
    }
}

// Update autotune state machine
void autotuneUpdate(timeUs_t currentTimeUs)
{
    // Check if autotune mode is active
    const bool shouldBeActive = IS_RC_MODE_ACTIVE(BOXAUTOTUNE) && 
                                autotuneConfig()->autotune_enabled &&
                                ARMING_FLAG(ARMED);
    
    // Log current state to debug[0] every cycle
    DEBUG_SET(DEBUG_AUTOTUNE, 0, runtime.state);
    
    // Activation/deactivation logic
    if (shouldBeActive && !runtime.active) {
        // Just activated
        runtime.active = true;
        changeState(AUTOTUNE_STATE_SETUP, currentTimeUs);
        beeper(BEEPER_AUTOTUNE_START);
        DEBUG_SET(DEBUG_AUTOTUNE, 1, 1000); // Activation marker
    } else if (!shouldBeActive && runtime.active) {
        // Just deactivated
        runtime.active = false;
        changeState(AUTOTUNE_STATE_IDLE, currentTimeUs);
        DEBUG_SET(DEBUG_AUTOTUNE, 1, 2000); // Deactivation marker
    }
    
    if (!runtime.active) {
        return;
    }
    
    // Safety check first - abort if unsafe
    if (runtime.state != AUTOTUNE_STATE_IDLE && 
        runtime.state != AUTOTUNE_STATE_COMPLETE &&
        runtime.state != AUTOTUNE_STATE_ABORTED) {
        
        if (!autotuneSafetyCheck()) {
            changeState(AUTOTUNE_STATE_ABORTED, currentTimeUs);
            beeper(BEEPER_AUTOTUNE_FAIL);
            DEBUG_SET(DEBUG_AUTOTUNE, 2, 9999); // Safety abort marker
            return;
        }
    }
    
    const uint32_t stateTimeMs = (currentTimeUs - runtime.stateEnteredAt) / 1000;
    DEBUG_SET(DEBUG_AUTOTUNE, 3, stateTimeMs); // Time in current state (ms)
    
    // State machine
    switch (runtime.state) {
        case AUTOTUNE_STATE_IDLE:
            // Should not be here if active
            changeState(AUTOTUNE_STATE_SETUP, currentTimeUs);
            break;
            
        case AUTOTUNE_STATE_SETUP:
            // Wait 2 seconds before starting to allow stabilization
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 2000000) {
                changeState(AUTOTUNE_STATE_WAIT_STABLE, currentTimeUs);
            }
            break;
            
        case AUTOTUNE_STATE_WAIT_STABLE:
            // Wait for stable conditions for 1 second
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 1000000) {
                // Start excitation
                changeState(AUTOTUNE_STATE_EXCITE, currentTimeUs);
                DEBUG_SET(DEBUG_AUTOTUNE, 1, 3333); // Excitation start marker
            }
            break;
            
        case AUTOTUNE_STATE_EXCITE:
            // Initialize on entry
            if (runtime.stateJustEntered) {
                runtime.stateJustEntered = false;
                runtime.excitationStartTime = currentTimeUs;
                runtime.sampleIndex = 0;
                runtime.sampleCount = 0;
                runtime.sampleCounter = 0;
            }
            
            // Collect samples (subsampled to avoid filling buffer too fast)
            runtime.sampleCounter++;
            if (runtime.sampleCounter >= runtime.sampleIntervalLoops) {
                runtime.sampleCounter = 0;
                const float excitation = autotuneGetExcitation(currentTimeUs);
                autotuneSampleData(gyro.gyroADCf[runtime.currentAxis], excitation);
            }
            
            // Bidirectional doublet duration is 6x step duration
            const timeDelta_t doubletDurationUs = autotuneConfig()->step_duration_ms * 6000;
            if (cmpTimeUs(currentTimeUs, runtime.excitationStartTime) >= doubletDurationUs) {
                changeState(AUTOTUNE_STATE_MEASURE, currentTimeUs);
                DEBUG_SET(DEBUG_AUTOTUNE, 1, 4444); // Measurement start marker
            }
            
            // Debug output during excitation
            DEBUG_SET(DEBUG_AUTOTUNE, 1, runtime.sampleCount);
            DEBUG_SET(DEBUG_AUTOTUNE, 2, (int16_t)gyro.gyroADCf[runtime.currentAxis]);
            DEBUG_SET(DEBUG_AUTOTUNE, 3, (int16_t)autotuneGetExcitation(currentTimeUs));
            // Show time elapsed in doublet (ms)
            DEBUG_SET(DEBUG_AUTOTUNE, 7, (int16_t)(cmpTimeUs(currentTimeUs, runtime.excitationStartTime) / 1000));
            break;
            
        case AUTOTUNE_STATE_MEASURE:
            // Continue collecting response data after doublet
            runtime.sampleCounter++;
            if (runtime.sampleCounter >= runtime.sampleIntervalLoops) {
                runtime.sampleCounter = 0;
                autotuneSampleData(gyro.gyroADCf[runtime.currentAxis], 0.0f);
            }
            
            // Collect for 3x step duration to capture settling
            const timeDelta_t measureDurationUs = autotuneConfig()->step_duration_ms * 3000;
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) >= measureDurationUs || 
                runtime.sampleIndex >= AUTOTUNE_SAMPLE_COUNT) {
                changeState(AUTOTUNE_STATE_ANALYZE, currentTimeUs);
                DEBUG_SET(DEBUG_AUTOTUNE, 1, 5555); // Analysis start marker
            }
            
            // Debug output during measurement
            DEBUG_SET(DEBUG_AUTOTUNE, 1, runtime.sampleCount);
            DEBUG_SET(DEBUG_AUTOTUNE, 2, (int16_t)gyro.gyroADCf[runtime.currentAxis]);
            break;
            
        case AUTOTUNE_STATE_ANALYZE:
            // TODO: Implement in Milestone 3
            // Wait 500ms in analyze state
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 500000) {
                changeState(AUTOTUNE_STATE_ADJUST, currentTimeUs);
            }
            break;
            
        case AUTOTUNE_STATE_ADJUST:
            // TODO: Implement in Milestone 3
            // Wait 500ms in adjust state then complete
            if (cmpTimeUs(currentTimeUs, runtime.stateEnteredAt) > 500000) {
                changeState(AUTOTUNE_STATE_COMPLETE, currentTimeUs);
                beeper(BEEPER_AUTOTUNE_DONE);
            }
            break;
            
        case AUTOTUNE_STATE_COMPLETE:
            // Stay in complete state until switch turned off
            DEBUG_SET(DEBUG_AUTOTUNE, 2, 1111); // Completion marker
            break;
            
        case AUTOTUNE_STATE_ABORTED:
            // Stay in aborted state until switch turned off
            DEBUG_SET(DEBUG_AUTOTUNE, 2, 8888); // Abort marker
            break;
    }
}

// Query functions
bool autotuneIsActive(void)
{
    return runtime.active;
}

autotuneState_e autotuneGetState(void)
{
    return runtime.state;
}

const char* autotuneGetStateName(void)
{
    return stateNames[runtime.state];
}

// Modify setpoint to add excitation signal
// This is called from PID loop to inject the doublet
float autotuneModifySetpoint(uint8_t axis, float setpoint, timeUs_t currentTimeUs)
{
    if (!runtime.active || runtime.state != AUTOTUNE_STATE_EXCITE) {
        return setpoint;
    }
    
    // Only modify the axis being tested
    if (axis != runtime.currentAxis) {
        return setpoint;
    }
    
    // Add excitation signal to pilot's setpoint
    const float excitation = autotuneGetExcitation(currentTimeUs);
    return setpoint + excitation;
}

#endif // USE_AUTOTUNE
