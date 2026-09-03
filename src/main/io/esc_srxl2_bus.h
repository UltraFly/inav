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

#include "io/esc_srxl2.h"

#define SRXL2_ESC_BUS_STARTUP_GUARD_MS      200
#define SRXL2_ESC_BUS_HANDSHAKE_RETRY_MS     50

#define SRXL2_ESC_BUS_BAUD_DEFAULT       115200
#define SRXL2_ESC_BUS_BAUD_HIGH          400000

typedef enum {
    SRXL2_ESC_BUS_DISABLED,
    SRXL2_ESC_BUS_LISTEN_GUARD,
    SRXL2_ESC_BUS_SEND_DIRECTED_HANDSHAKE,
    SRXL2_ESC_BUS_WAIT_HANDSHAKE_REPLY,
    SRXL2_ESC_BUS_SEND_FINAL_HANDSHAKE,
    SRXL2_ESC_BUS_WAIT_FINAL_TX_COMPLETE,
    SRXL2_ESC_BUS_RUNNING,
    SRXL2_ESC_BUS_PWM_FALLBACK,
} srxl2EscBusState_e;

typedef struct srxl2EscBus_s {
    srxl2EscBusState_e state;
    uint32_t stateStartedAtMs;
    uint32_t lastHandshakeTxAtMs;
    uint32_t lastTelemetryAtMs;
    uint32_t uid;
    uint32_t baudRate;
    uint8_t sourceDeviceId;
    uint8_t escDeviceId;
    uint8_t priority;
    uint8_t info;
    bool supportsHighBaud;
    bool escSupportsHighBaud;
    bool srxl2Detected;
    bool hasTelemetry;
} srxl2EscBus_t;

void srxl2EscBusInit(srxl2EscBus_t *bus, uint32_t nowMs, uint8_t sourceDeviceId,
    uint8_t priority, bool supportsHighBaud, uint8_t info, uint32_t uid);
void srxl2EscBusDisable(srxl2EscBus_t *bus);
// The caller must release PWM and put the shared signal pin in receive mode before restarting discovery.
void srxl2EscBusRestartDiscovery(srxl2EscBus_t *bus, uint32_t nowMs);

bool srxl2EscBusProcessFrame(srxl2EscBus_t *bus, const uint8_t *frame, size_t length, uint32_t nowMs);
size_t srxl2EscBusBuildPendingFrame(srxl2EscBus_t *bus, uint32_t nowMs, uint8_t *frame, size_t capacity);
void srxl2EscBusOnFrameTransmitted(srxl2EscBus_t *bus, uint32_t nowMs);

bool srxl2EscBusCanSendControl(const srxl2EscBus_t *bus);
uint32_t srxl2EscBusGetBaudRate(const srxl2EscBus_t *bus);
size_t srxl2EscBusBuildControlFrame(const srxl2EscBus_t *bus, uint8_t *frame, size_t capacity,
    uint8_t replyDeviceId, int8_t rssi, uint16_t frameLosses,
    const srxl2EscChannel_t *channels, size_t channelCount, bool failsafe);

void srxl2EscBusNoteTelemetry(srxl2EscBus_t *bus, uint32_t nowMs);
bool srxl2EscBusTelemetryIsFresh(const srxl2EscBus_t *bus, uint32_t nowMs, uint32_t timeoutMs);
