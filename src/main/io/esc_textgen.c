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

#include <string.h>

#include "platform.h"

#include "io/esc_textgen.h"

// The 16-byte payload format follows Spektrum's public telemetry sensor definition:
// https://github.com/SpektrumRC/SpektrumDocumentation/blob/master/Telemetry/spektrumTelemetrySensors.h

static bool selectPayloadInstance(escTextGenDisplay_t *display, uint8_t instance)
{
    if (display->instanceSelected && display->instance != instance) {
        return false;
    }

    display->instance = instance;
    display->instanceSelected = true;
    return true;
}

void escTextGenDisplayInit(escTextGenDisplay_t *display)
{
    if (display) {
        memset(display, 0, sizeof(*display));
    }
}

escTextGenPayloadResult_e escTextGenDisplayApplyPayload(escTextGenDisplay_t *display,
    const uint8_t *payload, size_t length)
{
    if (!display || !payload || length != ESC_TEXTGEN_PAYLOAD_SIZE) {
        return ESC_TEXTGEN_PAYLOAD_INVALID;
    }

    if (payload[0] != ESC_TEXTGEN_SENSOR_ID) {
        return ESC_TEXTGEN_PAYLOAD_NOT_TEXTGEN;
    }

    const uint8_t instance = payload[1];
    const uint8_t lineNumber = payload[2];

    if (lineNumber == ESC_TEXTGEN_LINE_REFRESH) {
        if (!selectPayloadInstance(display, instance)) {
            return ESC_TEXTGEN_PAYLOAD_INSTANCE_MISMATCH;
        }
        display->refreshPending = true;
        display->revision++;
        return ESC_TEXTGEN_PAYLOAD_REFRESH;
    }

    if (lineNumber == ESC_TEXTGEN_LINE_CLEAR) {
        if (!selectPayloadInstance(display, instance)) {
            return ESC_TEXTGEN_PAYLOAD_INSTANCE_MISMATCH;
        }
        memset(display->lines, 0, sizeof(display->lines));
        display->validLineMask = 0;
        display->refreshPending = true;
        display->revision++;
        return ESC_TEXTGEN_PAYLOAD_CLEARED;
    }

    if (lineNumber >= ESC_TEXTGEN_LINE_COUNT) {
        return ESC_TEXTGEN_PAYLOAD_INVALID;
    }

    char line[ESC_TEXTGEN_LINE_LENGTH + 1] = {0};
    bool terminated = false;
    for (size_t i = 0; i < ESC_TEXTGEN_LINE_LENGTH; i++) {
        const uint8_t character = payload[3 + i];
        if (terminated || character == 0) {
            terminated = true;
            continue;
        }
        if (character < 0x20 || character > 0x7E) {
            return ESC_TEXTGEN_PAYLOAD_INVALID;
        }
        line[i] = character;
    }

    if (!selectPayloadInstance(display, instance)) {
        return ESC_TEXTGEN_PAYLOAD_INSTANCE_MISMATCH;
    }

    memcpy(display->lines[lineNumber], line, sizeof(line));
    display->validLineMask |= 1U << lineNumber;
    display->revision++;
    return ESC_TEXTGEN_PAYLOAD_LINE_UPDATED;
}

void escTextGenDisplayAcknowledgeRefresh(escTextGenDisplay_t *display)
{
    if (display) {
        display->refreshPending = false;
    }
}

void escTextGenSessionInit(escTextGenSession_t *session)
{
    if (session) {
        memset(session, 0, sizeof(*session));
    }
}

static escTextGenSessionStopReason_e getSafetyStopReason(const escTextGenSafety_t *safety)
{
    if (!safety) {
        return ESC_TEXTGEN_SESSION_STOP_LINK_LOST;
    }
    if (safety->failsafe) {
        return ESC_TEXTGEN_SESSION_STOP_FAILSAFE;
    }
    if (safety->armed) {
        return ESC_TEXTGEN_SESSION_STOP_ARMED;
    }
    if (!safety->throttleLow) {
        return ESC_TEXTGEN_SESSION_STOP_THROTTLE_NOT_LOW;
    }
    if (!safety->thrustReverseNormal) {
        return ESC_TEXTGEN_SESSION_STOP_THRUST_REVERSE_ACTIVE;
    }
    if (!safety->linkAvailable) {
        return ESC_TEXTGEN_SESSION_STOP_LINK_LOST;
    }
    return ESC_TEXTGEN_SESSION_STOP_NONE;
}

bool escTextGenSessionEnter(escTextGenSession_t *session, uint32_t nowMs, const escTextGenSafety_t *safety)
{
    if (!session) {
        return false;
    }

    const escTextGenSessionStopReason_e stopReason = getSafetyStopReason(safety);
    if (stopReason != ESC_TEXTGEN_SESSION_STOP_NONE) {
        escTextGenSessionStop(session, stopReason);
        return false;
    }

    session->startedAtMs = nowMs;
    session->lastActivityAtMs = nowMs;
    session->navigationStartedAtMs = nowMs;
    session->navigation = ESC_TEXTGEN_NAVIGATION_NONE;
    session->stopReason = ESC_TEXTGEN_SESSION_STOP_NONE;
    session->active = true;
    session->powerCycleRequired = false;
    return true;
}

void escTextGenSessionUpdate(escTextGenSession_t *session, uint32_t nowMs, const escTextGenSafety_t *safety)
{
    if (!session || !session->active) {
        return;
    }

    const escTextGenSessionStopReason_e stopReason = getSafetyStopReason(safety);
    if (stopReason != ESC_TEXTGEN_SESSION_STOP_NONE) {
        escTextGenSessionStop(session, stopReason);
        return;
    }

    if ((uint32_t)(nowMs - session->lastActivityAtMs) > ESC_TEXTGEN_SESSION_TIMEOUT_MS) {
        escTextGenSessionStop(session, ESC_TEXTGEN_SESSION_STOP_TIMEOUT);
    }
}

void escTextGenSessionNoteActivity(escTextGenSession_t *session, uint32_t nowMs)
{
    if (session && session->active) {
        session->lastActivityAtMs = nowMs;
    }
}

void escTextGenSessionMarkSettingChanged(escTextGenSession_t *session)
{
    if (session && session->active) {
        session->powerCycleRequired = true;
    }
}

void escTextGenSessionStop(escTextGenSession_t *session, escTextGenSessionStopReason_e reason)
{
    if (!session) {
        return;
    }

    session->active = false;
    session->navigation = ESC_TEXTGEN_NAVIGATION_NONE;
    session->stopReason = reason;
}

bool escTextGenSessionRequestNavigation(escTextGenSession_t *session, escTextGenNavigation_e navigation,
    uint32_t nowMs)
{
    if (!session || !session->active || navigation <= ESC_TEXTGEN_NAVIGATION_NONE ||
        navigation > ESC_TEXTGEN_NAVIGATION_RIGHT) {
        return false;
    }

    if (session->navigation != ESC_TEXTGEN_NAVIGATION_NONE &&
        (uint32_t)(nowMs - session->navigationStartedAtMs) < ESC_TEXTGEN_NAVIGATION_PULSE_MS) {
        return false;
    }

    session->navigation = navigation;
    session->navigationStartedAtMs = nowMs;
    return true;
}

escTextGenChannelOverride_t escTextGenSessionGetChannelOverride(const escTextGenSession_t *session, uint32_t nowMs)
{
    escTextGenChannelOverride_t channelOverride = {
        .throttlePulseUs = ESC_TEXTGEN_THROTTLE_SAFE_US,
        .aileronPulseUs = ESC_TEXTGEN_CHANNEL_CENTER_US,
        .elevatorPulseUs = ESC_TEXTGEN_CHANNEL_CENTER_US,
        .active = session && session->active,
    };

    if (!channelOverride.active || session->navigation == ESC_TEXTGEN_NAVIGATION_NONE ||
        (uint32_t)(nowMs - session->navigationStartedAtMs) >= ESC_TEXTGEN_NAVIGATION_PULSE_MS) {
        return channelOverride;
    }

    switch (session->navigation) {
    case ESC_TEXTGEN_NAVIGATION_UP:
        channelOverride.elevatorPulseUs = ESC_TEXTGEN_CHANNEL_MAX_US;
        break;
    case ESC_TEXTGEN_NAVIGATION_DOWN:
        channelOverride.elevatorPulseUs = ESC_TEXTGEN_CHANNEL_MIN_US;
        break;
    case ESC_TEXTGEN_NAVIGATION_LEFT:
        channelOverride.aileronPulseUs = ESC_TEXTGEN_CHANNEL_MIN_US;
        break;
    case ESC_TEXTGEN_NAVIGATION_RIGHT:
        channelOverride.aileronPulseUs = ESC_TEXTGEN_CHANNEL_MAX_US;
        break;
    case ESC_TEXTGEN_NAVIGATION_NONE:
        break;
    }

    return channelOverride;
}
