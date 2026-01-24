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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

// ============================================================================
// Wiggle Pattern Definitions
// ============================================================================

typedef enum {
    FEEDBACK_PATTERN_NONE = 0,
    FEEDBACK_PATTERN_HOVER_LOCKED,      // 1 wiggle - hover lock achieved
    FEEDBACK_PATTERN_FILTERS_SET,       // 2 wiggles - filter characterization done
    FEEDBACK_PATTERN_AXIS_COMPLETE,     // 1 wiggle - single axis done
    FEEDBACK_PATTERN_ISSUE,             // 3 wiggles - problem detected
    FEEDBACK_PATTERN_ALL_COMPLETE,      // 4 wiggles - all tuning complete
    FEEDBACK_PATTERN_COUNT
} feedbackPattern_e;

// ============================================================================
// Initialization
// ============================================================================

// Initialize feedback system
void autotuneFeedbackInit(void);

// ============================================================================
// Update
// ============================================================================

// Update feedback system (call every loop)
void autotuneFeedbackUpdate(timeUs_t currentTimeUs);

// ============================================================================
// Feedback Triggers
// ============================================================================

// Queue hover lock feedback
void autotuneFeedbackHoverLocked(void);

// Queue filter characterization complete feedback
void autotuneFeedbackFiltersSet(void);

// Queue axis complete feedback
void autotuneFeedbackAxisComplete(uint8_t axis);

// Queue issue/warning feedback
void autotuneFeedbackIssue(void);

// Queue abort feedback - distinct urgent pattern
void autotuneFeedbackAbort(void);

// Queue all complete feedback
void autotuneFeedbackComplete(void);

// Queue state advance feedback - wiggles N times for state N
void autotuneFeedbackStateAdvance(uint8_t stateNumber);

// ============================================================================
// Status
// ============================================================================

// Check if feedback is currently playing
bool autotuneFeedbackIsPlaying(void);

// Get currently playing pattern
feedbackPattern_e autotuneFeedbackGetPattern(void);

// Get current setpoint offset for wiggle effect (deg/s)
// Should be called from PID loop and added to roll setpoint
float autotuneFeedbackGetRollOffset(void);
