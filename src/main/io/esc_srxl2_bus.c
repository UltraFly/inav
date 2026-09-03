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

#include "io/esc_srxl2.h"
#include "io/esc_srxl2_bus.h"

// The shared-PWM startup guard and handshake sequence follow section 7.2.1 of the public
// Spektrum SRXL2 specification (Rev K): https://github.com/SpektrumRC/SRXL2

static bool isEscDeviceId(uint8_t deviceId)
{
    return (deviceId & 0xF0) == SRXL2_ESC_DEVICE_ID_DEFAULT;
}

void srxl2EscBusInit(srxl2EscBus_t *bus, uint32_t nowMs, uint8_t sourceDeviceId,
    uint8_t priority, bool supportsHighBaud, uint8_t info, uint32_t uid)
{
    if (!bus) {
        return;
    }

    memset(bus, 0, sizeof(*bus));
    bus->state = SRXL2_ESC_BUS_LISTEN_GUARD;
    bus->stateStartedAtMs = nowMs;
    bus->uid = uid;
    bus->baudRate = SRXL2_ESC_BUS_BAUD_DEFAULT;
    bus->sourceDeviceId = sourceDeviceId;
    bus->priority = priority;
    bus->info = info;
    bus->supportsHighBaud = supportsHighBaud;
}

void srxl2EscBusDisable(srxl2EscBus_t *bus)
{
    if (bus) {
        bus->state = SRXL2_ESC_BUS_DISABLED;
        bus->hasTelemetry = false;
    }
}

void srxl2EscBusRestartDiscovery(srxl2EscBus_t *bus, uint32_t nowMs)
{
    if (!bus) {
        return;
    }

    bus->state = SRXL2_ESC_BUS_LISTEN_GUARD;
    bus->stateStartedAtMs = nowMs;
    bus->lastHandshakeTxAtMs = 0;
    bus->lastTelemetryAtMs = 0;
    bus->baudRate = SRXL2_ESC_BUS_BAUD_DEFAULT;
    bus->escDeviceId = 0;
    bus->escSupportsHighBaud = false;
    bus->srxl2Detected = false;
    bus->hasTelemetry = false;
}

bool srxl2EscBusProcessFrame(srxl2EscBus_t *bus, const uint8_t *frame, size_t length, uint32_t nowMs)
{
    if (!bus || bus->state == SRXL2_ESC_BUS_DISABLED || bus->state == SRXL2_ESC_BUS_PWM_FALLBACK) {
        return false;
    }

    srxl2EscHandshake_t handshake;
    if (!srxl2EscDecodeHandshake(frame, length, &handshake) || !isEscDeviceId(handshake.sourceDeviceId)) {
        return false;
    }

    if (handshake.destinationDeviceId == 0) {
        bus->srxl2Detected = true;
        bus->escDeviceId = handshake.sourceDeviceId;
        bus->escSupportsHighBaud = handshake.supportsHighBaud;
        bus->baudRate = SRXL2_ESC_BUS_BAUD_DEFAULT;
        bus->state = SRXL2_ESC_BUS_SEND_DIRECTED_HANDSHAKE;
        bus->stateStartedAtMs = nowMs;
        bus->hasTelemetry = false;
        return true;
    }

    if (handshake.destinationDeviceId == bus->sourceDeviceId &&
        handshake.sourceDeviceId == bus->escDeviceId &&
        (bus->state == SRXL2_ESC_BUS_SEND_DIRECTED_HANDSHAKE ||
            bus->state == SRXL2_ESC_BUS_WAIT_HANDSHAKE_REPLY)) {
        bus->escSupportsHighBaud = handshake.supportsHighBaud;
        bus->state = SRXL2_ESC_BUS_SEND_FINAL_HANDSHAKE;
        bus->stateStartedAtMs = nowMs;
        return true;
    }

    return false;
}

size_t srxl2EscBusBuildPendingFrame(srxl2EscBus_t *bus, uint32_t nowMs, uint8_t *frame, size_t capacity)
{
    if (!bus) {
        return 0;
    }

    if (bus->state == SRXL2_ESC_BUS_LISTEN_GUARD) {
        if ((uint32_t)(nowMs - bus->stateStartedAtMs) >= SRXL2_ESC_BUS_STARTUP_GUARD_MS) {
            bus->state = SRXL2_ESC_BUS_PWM_FALLBACK;
        }
        return 0;
    }

    if (bus->state == SRXL2_ESC_BUS_WAIT_HANDSHAKE_REPLY) {
        if ((uint32_t)(nowMs - bus->lastHandshakeTxAtMs) < SRXL2_ESC_BUS_HANDSHAKE_RETRY_MS) {
            return 0;
        }
        bus->state = SRXL2_ESC_BUS_SEND_DIRECTED_HANDSHAKE;
    }

    if (bus->state == SRXL2_ESC_BUS_SEND_DIRECTED_HANDSHAKE) {
        const size_t length = srxl2EscBuildHandshake(frame, capacity, bus->sourceDeviceId,
            bus->escDeviceId, bus->priority, bus->supportsHighBaud, bus->info, bus->uid);
        if (length) {
            bus->lastHandshakeTxAtMs = nowMs;
            bus->state = SRXL2_ESC_BUS_WAIT_HANDSHAKE_REPLY;
            bus->stateStartedAtMs = nowMs;
        }
        return length;
    }

    if (bus->state == SRXL2_ESC_BUS_SEND_FINAL_HANDSHAKE) {
        const bool useHighBaud = bus->supportsHighBaud && bus->escSupportsHighBaud;
        const size_t length = srxl2EscBuildHandshake(frame, capacity, bus->sourceDeviceId,
            SRXL2_ESC_DEVICE_ID_BROADCAST, bus->priority, useHighBaud, bus->info, bus->uid);
        if (length) {
            bus->state = SRXL2_ESC_BUS_WAIT_FINAL_TX_COMPLETE;
            bus->stateStartedAtMs = nowMs;
        }
        return length;
    }

    return 0;
}

void srxl2EscBusOnFrameTransmitted(srxl2EscBus_t *bus, uint32_t nowMs)
{
    if (!bus || bus->state != SRXL2_ESC_BUS_WAIT_FINAL_TX_COMPLETE) {
        return;
    }

    bus->baudRate = bus->supportsHighBaud && bus->escSupportsHighBaud ?
        SRXL2_ESC_BUS_BAUD_HIGH : SRXL2_ESC_BUS_BAUD_DEFAULT;
    bus->state = SRXL2_ESC_BUS_RUNNING;
    bus->stateStartedAtMs = nowMs;
    bus->hasTelemetry = false;
}

bool srxl2EscBusCanSendControl(const srxl2EscBus_t *bus)
{
    return bus && bus->state == SRXL2_ESC_BUS_RUNNING;
}

uint32_t srxl2EscBusGetBaudRate(const srxl2EscBus_t *bus)
{
    return bus ? bus->baudRate : 0;
}

size_t srxl2EscBusBuildControlFrame(const srxl2EscBus_t *bus, uint8_t *frame, size_t capacity,
    uint8_t replyDeviceId, int8_t rssi, uint16_t frameLosses,
    const srxl2EscChannel_t *channels, size_t channelCount, bool failsafe)
{
    if (!srxl2EscBusCanSendControl(bus)) {
        return 0;
    }

    return srxl2EscBuildControlFrame(frame, capacity, replyDeviceId, rssi, frameLosses,
        channels, channelCount, failsafe);
}

void srxl2EscBusNoteTelemetry(srxl2EscBus_t *bus, uint32_t nowMs)
{
    if (srxl2EscBusCanSendControl(bus)) {
        bus->lastTelemetryAtMs = nowMs;
        bus->hasTelemetry = true;
    }
}

bool srxl2EscBusTelemetryIsFresh(const srxl2EscBus_t *bus, uint32_t nowMs, uint32_t timeoutMs)
{
    return srxl2EscBusCanSendControl(bus) && bus->hasTelemetry &&
        (uint32_t)(nowMs - bus->lastTelemetryAtMs) <= timeoutMs;
}
