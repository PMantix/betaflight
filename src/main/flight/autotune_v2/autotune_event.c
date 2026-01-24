/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_AUTOTUNE_V2

#include <string.h>
#include <math.h>

#include "common/maths.h"
#include "common/time.h"

#include "flight/autotune_v2/autotune_event.h"
#include "flight/autotune_v2/autotune_debug.h"

#include "pg/autotune.h"

// ============================================================================
// Static State
// ============================================================================

static eventDetector_t detector;
static eventData_t currentEvent;
static eventBuffer_t buffer;

// ============================================================================
// Initialization
// ============================================================================

void autotuneEventInit(void)
{
    autotuneEventReset();
    memset(&buffer, 0, sizeof(eventBuffer_t));
}

void autotuneEventReset(void)
{
    memset(&detector, 0, sizeof(eventDetector_t));
    memset(&currentEvent, 0, sizeof(eventData_t));
    detector.state = EVENT_STATE_IDLE;
}

// ============================================================================
// Event Detection
// ============================================================================

bool autotuneEventUpdate(timeUs_t currentTimeUs, uint8_t axis, float stickDeflection, float crossAxisPosition, float throttle)
{
    UNUSED(axis);
    
    const float absDeflection = fabsf(stickDeflection);
    
    switch (detector.state) {
        case EVENT_STATE_IDLE:
            // Wait for stick to move away from center
            if (absDeflection >= AUTOTUNE_STICK_DEFLECTION_START) {
                detector.state = EVENT_STATE_DEFLECTING;
                detector.deflectionStartUs = currentTimeUs;
                detector.peakDeflection = absDeflection;
                detector.stickSign = (stickDeflection > 0) ? 1 : -1;
                detector.throttleAtStart = throttle;
                detector.crossAxisMax = fabsf(crossAxisPosition);
                detector.hasReversed = false;
                
                currentEvent.startTimeUs = currentTimeUs;
                currentEvent.throttle = throttle;
            }
            break;
            
        case EVENT_STATE_DEFLECTING:
            // Track peak deflection
            if (absDeflection > detector.peakDeflection) {
                detector.peakDeflection = absDeflection;
            }
            
            // Track maximum cross-axis movement
            detector.crossAxisMax = fmaxf(detector.crossAxisMax, fabsf(crossAxisPosition));
            
            // Check for stick returning toward center
            if (absDeflection < detector.peakDeflection * 0.5f) {
                detector.state = EVENT_STATE_RETURNING;
            }
            
            // Timeout check
            if (currentTimeUs - detector.deflectionStartUs > AUTOTUNE_EVENT_MAX_DURATION_US) {
                autotuneEventReset();
            }
            break;
            
        case EVENT_STATE_RETURNING:
            // Track maximum cross-axis movement
            detector.crossAxisMax = fmaxf(detector.crossAxisMax, fabsf(crossAxisPosition));
            
            // Check for stick crossing center or reaching center threshold
            if ((stickDeflection * detector.stickSign < 0) || 
                (absDeflection < AUTOTUNE_STICK_CENTER_THRESHOLD)) {
                detector.hasReversed = true;
            }
            
            // Event complete when returned to center
            if (detector.hasReversed && absDeflection < AUTOTUNE_STICK_CENTER_THRESHOLD) {
                detector.state = EVENT_STATE_COMPLETE;
                detector.deflectionEndUs = currentTimeUs;
                
                // Fill event data
                currentEvent.stickDeflection = detector.peakDeflection;
                currentEvent.crossAxisMovement = detector.crossAxisMax;
                currentEvent.endTimeUs = currentTimeUs;
                currentEvent.qualityGatesPassed = autotuneEventCheckQuality(&currentEvent);
                
                return true;
            }
            
            // Timeout check
            if (currentTimeUs - detector.deflectionStartUs > AUTOTUNE_EVENT_MAX_DURATION_US) {
                autotuneEventReset();
            }
            break;
            
        case EVENT_STATE_COMPLETE:
            // Stay in complete until reset
            break;
            
        default:
            autotuneEventReset();
            break;
    }
    
    return false;
}

const eventData_t* autotuneEventGetData(void)
{
    return &currentEvent;
}

eventState_e autotuneEventGetState(void)
{
    return detector.state;
}

// ============================================================================
// Event Buffer
// ============================================================================

void autotuneEventBufferStart(void)
{
    memset(&buffer, 0, sizeof(eventBuffer_t));
    buffer.capturing = true;
}

void autotuneEventBufferAddSample(float gyro, float setpoint)
{
    if (!buffer.capturing || buffer.count >= EVENT_BUFFER_SIZE) {
        return;
    }
    
    buffer.gyro[buffer.count] = gyro;
    buffer.setpoint[buffer.count] = setpoint;
    buffer.count++;
}

void autotuneEventBufferStop(void)
{
    buffer.capturing = false;
}

const eventBuffer_t* autotuneEventBufferGet(void)
{
    return &buffer;
}

// ============================================================================
// Quality Gates
// ============================================================================

bool autotuneEventCheckDeflection(float deflection)
{
    // deflection is normalized 0.0-1.0, threshold is also normalized
    return deflection >= AUTOTUNE_STICK_DEFLECTION_MIN;
}

bool autotuneEventCheckCrossAxis(float crossAxis)
{
    // crossAxis is normalized 0.0-1.0, threshold is also normalized
    return crossAxis <= AUTOTUNE_CROSS_AXIS_MAX;
}

bool autotuneEventCheckThrottle(float throttle)
{
    return throttle >= AUTOTUNE_THROTTLE_LOW_LIMIT && 
           throttle <= AUTOTUNE_THROTTLE_HIGH_LIMIT;
}

bool autotuneEventCheckQuality(const eventData_t *event)
{
    if (!autotuneEventCheckDeflection(event->stickDeflection)) {
        return false;
    }
    
    if (!autotuneEventCheckCrossAxis(event->crossAxisMovement)) {
        return false;
    }
    
    if (!autotuneEventCheckThrottle(event->throttle)) {
        return false;
    }
    
    // Check duration
    const timeUs_t duration = event->endTimeUs - event->startTimeUs;
    if (duration < AUTOTUNE_EVENT_MIN_DURATION_US || 
        duration > AUTOTUNE_EVENT_MAX_DURATION_US) {
        return false;
    }
    
    return true;
}

#endif // USE_AUTOTUNE_V2
