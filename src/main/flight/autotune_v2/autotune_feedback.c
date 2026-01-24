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

#include <math.h>

#include "common/maths.h"
#include "common/time.h"

#include "drivers/time.h"

#include "flight/autotune_v2/autotune_feedback.h"
#include "flight/autotune_v2/autotune_debug.h"

// ============================================================================
// Constants
// ============================================================================

// Two basic element types: BUMP (half-sine, quick) and WIGGLE (full sine, slower)
#define BUMP_DURATION_US        100000  // 100ms half-sine (nudge)
#define WIGGLE_DURATION_US      200000  // 200ms full sine (sway)
#define ABORT_BUMP_DURATION_US   50000  // 50ms rapid bump for abort pattern
#define ELEMENT_PAUSE_US         50000  // 50ms pause between elements
#define ABORT_PAUSE_US           25000  // 25ms short pause for abort (more urgent)
#define WIGGLE_AMPLITUDE_DPS    30.0f   // 30 deg/s amplitude

// Pattern element types
typedef enum {
    ELEMENT_BUMP = 0,   // Half-sine (0 to peak to 0)
    ELEMENT_WIGGLE,     // Full sine (0 to peak to -peak to 0)
    ELEMENT_ABORT_BUMP  // Rapid half-sine for abort pattern
} patternElement_e;

// Maximum elements in a pattern
#define MAX_PATTERN_ELEMENTS 4

// ============================================================================
// Static State
// ============================================================================

typedef struct {
    patternElement_e elements[MAX_PATTERN_ELEMENTS];
    uint8_t elementCount;
    uint8_t currentElement;
    bool isPlaying;
    bool inPause;                       // Between elements
    bool isAbortPattern;                // Abort uses shorter timing
    timeUs_t phaseStartUs;
    float currentOffset;
} feedbackState_t;

static feedbackState_t feedbackState;

// ============================================================================
// Pattern Definitions - Each state has a unique pattern
// ============================================================================

// Patterns: B=bump, W=wiggle
// State 1: B        (quick nudge)
// State 2: W        (full sway)
// State 3: B-B      (dit-dit)
// State 4: W-W      (big milestone)
// State 5: B-W      (dit-dah)
// State 6: W-B      (dah-dit)
// State 7: B-B-B    (dit-dit-dit)
// Complete: W-W-W   (celebration)

static void setPatternForState(uint8_t stateNumber)
{
    feedbackState.elementCount = 0;
    
    switch (stateNumber) {
        case 1:  // HOVER_LOCK: B
            feedbackState.elements[0] = ELEMENT_BUMP;
            feedbackState.elementCount = 1;
            break;
        case 2:  // THROTTLE_SWEEP: W
            feedbackState.elements[0] = ELEMENT_WIGGLE;
            feedbackState.elementCount = 1;
            break;
        case 3:  // NOISE_CONFIRM: B-B
            feedbackState.elements[0] = ELEMENT_BUMP;
            feedbackState.elements[1] = ELEMENT_BUMP;
            feedbackState.elementCount = 2;
            break;
        case 4:  // PD_RATIO_SEEK: W-W
            feedbackState.elements[0] = ELEMENT_WIGGLE;
            feedbackState.elements[1] = ELEMENT_WIGGLE;
            feedbackState.elementCount = 2;
            break;
        case 5:  // PD_SCALE_UP: B-W
            feedbackState.elements[0] = ELEMENT_BUMP;
            feedbackState.elements[1] = ELEMENT_WIGGLE;
            feedbackState.elementCount = 2;
            break;
        case 6:  // F_TUNE: W-B
            feedbackState.elements[0] = ELEMENT_WIGGLE;
            feedbackState.elements[1] = ELEMENT_BUMP;
            feedbackState.elementCount = 2;
            break;
        case 7:  // PD_RETUNE: B-B-B
            feedbackState.elements[0] = ELEMENT_BUMP;
            feedbackState.elements[1] = ELEMENT_BUMP;
            feedbackState.elements[2] = ELEMENT_BUMP;
            feedbackState.elementCount = 3;
            break;
        case 8:  // COMPLETE: W-W-W
            feedbackState.elements[0] = ELEMENT_WIGGLE;
            feedbackState.elements[1] = ELEMENT_WIGGLE;
            feedbackState.elements[2] = ELEMENT_WIGGLE;
            feedbackState.elementCount = 3;
            break;
        case 9:  // ABORT: Rapid B-B-B-B (urgent alarm pattern)
            feedbackState.elements[0] = ELEMENT_ABORT_BUMP;
            feedbackState.elements[1] = ELEMENT_ABORT_BUMP;
            feedbackState.elements[2] = ELEMENT_ABORT_BUMP;
            feedbackState.elements[3] = ELEMENT_ABORT_BUMP;
            feedbackState.elementCount = 4;
            feedbackState.isAbortPattern = true;
            break;
        default:
            feedbackState.elements[0] = ELEMENT_WIGGLE;
            feedbackState.elements[1] = ELEMENT_WIGGLE;
            feedbackState.elements[2] = ELEMENT_WIGGLE;
            feedbackState.elementCount = 3;
            break;
    }
}

// ============================================================================
// Initialization
// ============================================================================

void autotuneFeedbackInit(void)
{
    feedbackState.elementCount = 0;
    feedbackState.currentElement = 0;
    feedbackState.isPlaying = false;
    feedbackState.inPause = false;
    feedbackState.isAbortPattern = false;
    feedbackState.phaseStartUs = 0;
    feedbackState.currentOffset = 0.0f;
}

// ============================================================================
// Update
// ============================================================================

void autotuneFeedbackUpdate(timeUs_t currentTimeUs)
{
    if (!feedbackState.isPlaying) {
        feedbackState.currentOffset = 0.0f;
        return;
    }
    
    const timeUs_t phaseDuration = currentTimeUs - feedbackState.phaseStartUs;
    
    // Use shorter pause for abort pattern (more urgent feel)
    const timeUs_t pauseDuration = feedbackState.isAbortPattern ? ABORT_PAUSE_US : ELEMENT_PAUSE_US;
    
    if (feedbackState.inPause) {
        // In pause between elements
        feedbackState.currentOffset = 0.0f;
        if (phaseDuration >= pauseDuration) {
            feedbackState.inPause = false;
            feedbackState.phaseStartUs = currentTimeUs;
        }
        return;
    }
    
    // Currently playing an element
    const patternElement_e element = feedbackState.elements[feedbackState.currentElement];
    
    // Get duration based on element type
    timeUs_t elementDuration;
    if (element == ELEMENT_ABORT_BUMP) {
        elementDuration = ABORT_BUMP_DURATION_US;
    } else if (element == ELEMENT_BUMP) {
        elementDuration = BUMP_DURATION_US;
    } else {
        elementDuration = WIGGLE_DURATION_US;
    }
    
    if (phaseDuration >= elementDuration) {
        // Element complete, move to next
        feedbackState.currentElement++;
        
        if (feedbackState.currentElement >= feedbackState.elementCount) {
            // Pattern complete - reset abort flag
            feedbackState.isPlaying = false;
            feedbackState.isAbortPattern = false;
            feedbackState.currentOffset = 0.0f;
        } else {
            // Start pause before next element
            feedbackState.inPause = true;
            feedbackState.phaseStartUs = currentTimeUs;
            feedbackState.currentOffset = 0.0f;
        }
    } else {
        // Generate waveform based on element type
        const float phase = (float)phaseDuration / (float)elementDuration;
        
        if (element == ELEMENT_BUMP || element == ELEMENT_ABORT_BUMP) {
            // Half-sine: just the positive lobe (0 -> 1 -> 0)
            feedbackState.currentOffset = sin_approx(phase * M_PIf) * WIGGLE_AMPLITUDE_DPS;
        } else {
            // Full sine wave (0 -> 1 -> 0 -> -1 -> 0)
            feedbackState.currentOffset = sin_approx(phase * 2.0f * M_PIf) * WIGGLE_AMPLITUDE_DPS;
        }
    }
}

// ============================================================================
// Feedback Triggers
// ============================================================================

static void startStatePattern(uint8_t stateNumber, timeUs_t currentTimeUs)
{
    setPatternForState(stateNumber);
    feedbackState.currentElement = 0;
    feedbackState.isPlaying = true;
    feedbackState.inPause = false;
    feedbackState.phaseStartUs = currentTimeUs;
    feedbackState.currentOffset = 0.0f;
}

void autotuneFeedbackHoverLocked(void)
{
    startStatePattern(1, micros());  // State 1 pattern
}

void autotuneFeedbackFiltersSet(void)
{
    startStatePattern(3, micros());  // State 3 pattern (noise confirm)
}

void autotuneFeedbackAxisComplete(uint8_t axis)
{
    UNUSED(axis);
    startStatePattern(5, micros());  // State 5 pattern
}

void autotuneFeedbackIssue(void)
{
    startStatePattern(7, micros());  // State 7 pattern (triple bump)
}

void autotuneFeedbackComplete(void)
{
    startStatePattern(8, micros());  // Complete pattern (triple wiggle)
}

void autotuneFeedbackStateAdvance(uint8_t stateNumber)
{
    // Play the unique pattern for this state number
    if (stateNumber > 0 && stateNumber <= 8) {
        startStatePattern(stateNumber, micros());
    }
}

// ============================================================================
// Status
// ============================================================================

bool autotuneFeedbackIsPlaying(void)
{
    return feedbackState.isPlaying;
}

feedbackPattern_e autotuneFeedbackGetPattern(void)
{
    // Pattern enum is deprecated - return NONE or a placeholder
    return feedbackState.isPlaying ? FEEDBACK_PATTERN_HOVER_LOCKED : FEEDBACK_PATTERN_NONE;
}

float autotuneFeedbackGetRollOffset(void)
{
    return feedbackState.currentOffset;
}

#endif // USE_AUTOTUNE_V2
