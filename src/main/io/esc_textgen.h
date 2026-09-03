/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ESC_TEXTGEN_SENSOR_ID                 0x0C
#define ESC_TEXTGEN_PAYLOAD_SIZE                16
#define ESC_TEXTGEN_LINE_COUNT                   9
#define ESC_TEXTGEN_LINE_LENGTH                 13
#define ESC_TEXTGEN_LINE_REFRESH               254
#define ESC_TEXTGEN_LINE_CLEAR                 255

#define ESC_TEXTGEN_THROTTLE_SAFE_US           1000
#define ESC_TEXTGEN_CHANNEL_MIN_US             1000
#define ESC_TEXTGEN_CHANNEL_CENTER_US          1500
#define ESC_TEXTGEN_CHANNEL_MAX_US             2000
#define ESC_TEXTGEN_NAVIGATION_PULSE_MS          250
#define ESC_TEXTGEN_SESSION_TIMEOUT_MS          5000

typedef enum {
    ESC_TEXTGEN_PAYLOAD_INVALID,
    ESC_TEXTGEN_PAYLOAD_NOT_TEXTGEN,
    ESC_TEXTGEN_PAYLOAD_INSTANCE_MISMATCH,
    ESC_TEXTGEN_PAYLOAD_LINE_UPDATED,
    ESC_TEXTGEN_PAYLOAD_REFRESH,
    ESC_TEXTGEN_PAYLOAD_CLEARED,
} escTextGenPayloadResult_e;

typedef struct escTextGenDisplay_s {
    char lines[ESC_TEXTGEN_LINE_COUNT][ESC_TEXTGEN_LINE_LENGTH + 1];
    uint16_t validLineMask;
    uint32_t revision;
    uint8_t instance;
    bool instanceSelected;
    bool refreshPending;
} escTextGenDisplay_t;

void escTextGenDisplayInit(escTextGenDisplay_t *display);
escTextGenPayloadResult_e escTextGenDisplayApplyPayload(escTextGenDisplay_t *display,
    const uint8_t *payload, size_t length);
void escTextGenDisplayAcknowledgeRefresh(escTextGenDisplay_t *display);

typedef enum {
    ESC_TEXTGEN_NAVIGATION_NONE,
    ESC_TEXTGEN_NAVIGATION_UP,
    ESC_TEXTGEN_NAVIGATION_DOWN,
    ESC_TEXTGEN_NAVIGATION_LEFT,
    ESC_TEXTGEN_NAVIGATION_RIGHT,
} escTextGenNavigation_e;

typedef enum {
    ESC_TEXTGEN_SESSION_STOP_NONE,
    ESC_TEXTGEN_SESSION_STOP_USER,
    ESC_TEXTGEN_SESSION_STOP_ARMED,
    ESC_TEXTGEN_SESSION_STOP_THROTTLE_NOT_LOW,
    ESC_TEXTGEN_SESSION_STOP_FAILSAFE,
    ESC_TEXTGEN_SESSION_STOP_LINK_LOST,
    ESC_TEXTGEN_SESSION_STOP_TIMEOUT,
    ESC_TEXTGEN_SESSION_STOP_MALFORMED_TELEMETRY,
} escTextGenSessionStopReason_e;

typedef struct escTextGenSafety_s {
    bool armed;
    bool throttleLow;
    bool failsafe;
    bool linkAvailable;
} escTextGenSafety_t;

typedef struct escTextGenChannelOverride_s {
    uint16_t throttlePulseUs;
    uint16_t aileronPulseUs;
    uint16_t elevatorPulseUs;
    bool active;
} escTextGenChannelOverride_t;

typedef struct escTextGenSession_s {
    uint32_t startedAtMs;
    uint32_t lastActivityAtMs;
    uint32_t navigationStartedAtMs;
    escTextGenNavigation_e navigation;
    escTextGenSessionStopReason_e stopReason;
    bool active;
    bool powerCycleRequired;
} escTextGenSession_t;

void escTextGenSessionInit(escTextGenSession_t *session);
bool escTextGenSessionEnter(escTextGenSession_t *session, uint32_t nowMs, const escTextGenSafety_t *safety);
void escTextGenSessionUpdate(escTextGenSession_t *session, uint32_t nowMs, const escTextGenSafety_t *safety);
void escTextGenSessionNoteActivity(escTextGenSession_t *session, uint32_t nowMs);
void escTextGenSessionMarkSettingChanged(escTextGenSession_t *session);
void escTextGenSessionStop(escTextGenSession_t *session, escTextGenSessionStopReason_e reason);
bool escTextGenSessionRequestNavigation(escTextGenSession_t *session, escTextGenNavigation_e navigation,
    uint32_t nowMs);
escTextGenChannelOverride_t escTextGenSessionGetChannelOverride(const escTextGenSession_t *session, uint32_t nowMs);
