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

#define SRXL2_ESC_HANDSHAKE_FRAME_SIZE       14
#define SRXL2_ESC_CONTROL_FRAME_BASE_SIZE    14
#define SRXL2_ESC_MAX_CHANNELS                32
#define SRXL2_ESC_CONTROL_FRAME_MAX_SIZE     (SRXL2_ESC_CONTROL_FRAME_BASE_SIZE + 2 * SRXL2_ESC_MAX_CHANNELS)
#define SRXL2_ESC_TELEMETRY_FRAME_SIZE       22

#define SRXL2_ESC_DEVICE_ID_DEFAULT    0x40
#define SRXL2_ESC_DEVICE_ID_BROADCAST  0xFF

// Observed on a working Spektrum receiver-to-Avian connection. These values
// retain the two low SRXL2 flag bits as zero and still require hardware capture
// verification on the direct FC-to-ESC connection.
#define SRXL2_ESC_CHANNEL_MIN          0x2AA0
#define SRXL2_ESC_CHANNEL_CENTER       0x8000
#define SRXL2_ESC_CHANNEL_MAX          0xD554

#define SRXL2_ESC_PWM_MIN              1000
#define SRXL2_ESC_PWM_CENTER           1500
#define SRXL2_ESC_PWM_MAX              2000

typedef enum {
    SRXL2_ESC_TELEMETRY_ERPM_VALID         = (1 << 0),
    SRXL2_ESC_TELEMETRY_VOLTAGE_VALID      = (1 << 1),
    SRXL2_ESC_TELEMETRY_FET_TEMP_VALID     = (1 << 2),
    SRXL2_ESC_TELEMETRY_CURRENT_VALID      = (1 << 3),
    SRXL2_ESC_TELEMETRY_BEC_TEMP_VALID     = (1 << 4),
    SRXL2_ESC_TELEMETRY_BEC_CURRENT_VALID  = (1 << 5),
    SRXL2_ESC_TELEMETRY_BEC_VOLTAGE_VALID  = (1 << 6),
    SRXL2_ESC_TELEMETRY_THROTTLE_VALID     = (1 << 7),
    SRXL2_ESC_TELEMETRY_POWER_OUT_VALID    = (1 << 8),
} srxl2EscTelemetryValidity_e;

typedef struct srxl2EscTelemetry_s {
    uint16_t valid;
    uint8_t destinationDeviceId;
    uint8_t secondaryId;
    uint32_t electricalRpm;
    uint16_t voltageCentiVolts;
    uint16_t fetTemperatureDeciCelsius;
    uint16_t currentCentiAmps;
    uint16_t becTemperatureDeciCelsius;
    uint16_t becCurrentDeciAmps;
    uint16_t becVoltageCentiVolts;
    uint8_t throttleHalfPercent;
    uint8_t powerOutHalfPercent;
} srxl2EscTelemetry_t;

typedef struct srxl2EscHandshake_s {
    uint8_t sourceDeviceId;
    uint8_t destinationDeviceId;
    uint8_t priority;
    bool supportsHighBaud;
    uint8_t info;
    uint32_t uid;
} srxl2EscHandshake_t;

typedef enum {
    SRXL2_ESC_DECODE_INVALID,
    SRXL2_ESC_DECODE_NOT_ESC_TELEMETRY,
    SRXL2_ESC_DECODE_OK,
} srxl2EscDecodeResult_e;

typedef struct srxl2EscChannel_s {
    uint8_t channel;
    uint16_t pulseUs;
} srxl2EscChannel_t;

struct escSensorData_s;

uint16_t srxl2EscPwmToChannelValue(uint16_t pulseUs);

size_t srxl2EscBuildHandshake(uint8_t *frame, size_t capacity, uint8_t sourceDeviceId,
    uint8_t destinationDeviceId, uint8_t priority, bool supportsHighBaud, uint8_t info, uint32_t uid);

bool srxl2EscDecodeHandshake(const uint8_t *frame, size_t length, srxl2EscHandshake_t *handshake);

size_t srxl2EscBuildControlFrame(uint8_t *frame, size_t capacity, uint8_t replyDeviceId,
    int8_t rssi, uint16_t frameLosses, const srxl2EscChannel_t *channels, size_t channelCount, bool failsafe);

srxl2EscDecodeResult_e srxl2EscDecodeTelemetry(const uint8_t *frame, size_t length, srxl2EscTelemetry_t *telemetry);

// Updates only the standard INAV ESC fields present in telemetry and returns their validity mask.
uint16_t srxl2EscApplyTelemetry(const srxl2EscTelemetry_t *telemetry, uint8_t motorPoleCount,
    struct escSensorData_s *sensorData);
