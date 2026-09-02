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

#include "common/crc.h"
#include "common/streambuf.h"

#include "io/esc_srxl2.h"

#define SRXL2_ID                         0xA6
#define SRXL2_PACKET_TYPE_HANDSHAKE      0x21
#define SRXL2_PACKET_TYPE_TELEMETRY      0x80
#define SRXL2_PACKET_TYPE_CONTROL        0xCD
#define SRXL2_CONTROL_COMMAND_CHANNEL    0x00
#define SRXL2_CONTROL_COMMAND_FAILSAFE   0x01
#define SPEKTRUM_TELEMETRY_DEVICE_ESC    0x20
#define SPEKTRUM_TELEMETRY_NO_DATA_16    0xFFFF
#define SPEKTRUM_TELEMETRY_NO_DATA_8     0xFF

static void srxl2EscAppendCrc(sbuf_t *dst, uint8_t *start)
{
    const uint16_t crc = crc16_ccitt_update(0, start, (uint32_t)(sbufPtr(dst) - start));
    sbufWriteU16BigEndian(dst, crc);
}

static uint16_t readU16BigEndian(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

uint16_t srxl2EscPwmToChannelValue(uint16_t pulseUs)
{
    const uint16_t constrainedPulseUs = pulseUs < SRXL2_ESC_PWM_MIN ? SRXL2_ESC_PWM_MIN :
        (pulseUs > SRXL2_ESC_PWM_MAX ? SRXL2_ESC_PWM_MAX : pulseUs);
    uint32_t value;

    if (constrainedPulseUs <= SRXL2_ESC_PWM_CENTER) {
        value = ((uint32_t)(constrainedPulseUs - SRXL2_ESC_PWM_MIN) * 0x8000) /
            (SRXL2_ESC_PWM_CENTER - SRXL2_ESC_PWM_MIN);
    } else {
        value = 0x8000 + ((uint32_t)(constrainedPulseUs - SRXL2_ESC_PWM_CENTER) * (0xFFFC - 0x8000)) /
            (SRXL2_ESC_PWM_MAX - SRXL2_ESC_PWM_CENTER);
    }

    return value & 0xFFFC;
}

size_t srxl2EscBuildHandshake(uint8_t *frame, size_t capacity, uint8_t sourceDeviceId,
    uint8_t destinationDeviceId, uint8_t priority, bool supportsHighBaud, uint8_t info, uint32_t uid)
{
    if (!frame || capacity < SRXL2_ESC_HANDSHAKE_FRAME_SIZE) {
        return 0;
    }

    sbuf_t buffer;
    sbuf_t *dst = sbufInit(&buffer, frame, frame + capacity);

    sbufWriteU8(dst, SRXL2_ID);
    sbufWriteU8(dst, SRXL2_PACKET_TYPE_HANDSHAKE);
    sbufWriteU8(dst, SRXL2_ESC_HANDSHAKE_FRAME_SIZE);
    sbufWriteU8(dst, sourceDeviceId);
    sbufWriteU8(dst, destinationDeviceId);
    sbufWriteU8(dst, priority < 1 ? 1 : (priority > 100 ? 100 : priority));
    sbufWriteU8(dst, supportsHighBaud ? 1 : 0);
    sbufWriteU8(dst, info);
    sbufWriteU32(dst, uid);
    srxl2EscAppendCrc(dst, frame);

    return sbufPtr(dst) - frame;
}

size_t srxl2EscBuildControlFrame(uint8_t *frame, size_t capacity, uint8_t replyDeviceId,
    int8_t rssi, uint16_t frameLosses, uint8_t throttleChannel, uint16_t throttlePulseUs, bool failsafe)
{
    if (!frame || capacity < SRXL2_ESC_CONTROL_FRAME_SIZE || throttleChannel >= 32) {
        return 0;
    }

    sbuf_t buffer;
    sbuf_t *dst = sbufInit(&buffer, frame, frame + capacity);

    sbufWriteU8(dst, SRXL2_ID);
    sbufWriteU8(dst, SRXL2_PACKET_TYPE_CONTROL);
    sbufWriteU8(dst, SRXL2_ESC_CONTROL_FRAME_SIZE);
    sbufWriteU8(dst, failsafe ? SRXL2_CONTROL_COMMAND_FAILSAFE : SRXL2_CONTROL_COMMAND_CHANNEL);
    sbufWriteU8(dst, replyDeviceId);
    sbufWriteU8(dst, rssi);
    sbufWriteU16(dst, frameLosses);
    sbufWriteU32(dst, 1UL << throttleChannel);
    sbufWriteU16(dst, srxl2EscPwmToChannelValue(throttlePulseUs));
    srxl2EscAppendCrc(dst, frame);

    return sbufPtr(dst) - frame;
}

srxl2EscDecodeResult_e srxl2EscDecodeTelemetry(const uint8_t *frame, size_t length, srxl2EscTelemetry_t *telemetry)
{
    if (!frame || !telemetry || length != SRXL2_ESC_TELEMETRY_FRAME_SIZE ||
        frame[0] != SRXL2_ID || frame[1] != SRXL2_PACKET_TYPE_TELEMETRY || frame[2] != length ||
        crc16_ccitt_update(0, frame, length) != 0) {
        return SRXL2_ESC_DECODE_INVALID;
    }

    if (frame[4] != SPEKTRUM_TELEMETRY_DEVICE_ESC) {
        return SRXL2_ESC_DECODE_NOT_ESC_TELEMETRY;
    }

    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->destinationDeviceId = frame[3];
    telemetry->secondaryId = frame[5];

    const uint16_t rpm = readU16BigEndian(&frame[6]);
    if (rpm != SPEKTRUM_TELEMETRY_NO_DATA_16) {
        telemetry->electricalRpm = (uint32_t)rpm * 10;
        telemetry->valid |= SRXL2_ESC_TELEMETRY_ERPM_VALID;
    }

    const uint16_t voltage = readU16BigEndian(&frame[8]);
    if (voltage != SPEKTRUM_TELEMETRY_NO_DATA_16) {
        telemetry->voltageCentiVolts = voltage;
        telemetry->valid |= SRXL2_ESC_TELEMETRY_VOLTAGE_VALID;
    }

    const uint16_t fetTemperature = readU16BigEndian(&frame[10]);
    if (fetTemperature != SPEKTRUM_TELEMETRY_NO_DATA_16) {
        telemetry->fetTemperatureDeciCelsius = fetTemperature;
        telemetry->valid |= SRXL2_ESC_TELEMETRY_FET_TEMP_VALID;
    }

    const uint16_t current = readU16BigEndian(&frame[12]);
    if (current != SPEKTRUM_TELEMETRY_NO_DATA_16) {
        telemetry->currentCentiAmps = current;
        telemetry->valid |= SRXL2_ESC_TELEMETRY_CURRENT_VALID;
    }

    const uint16_t becTemperature = readU16BigEndian(&frame[14]);
    if (becTemperature != SPEKTRUM_TELEMETRY_NO_DATA_16) {
        telemetry->becTemperatureDeciCelsius = becTemperature;
        telemetry->valid |= SRXL2_ESC_TELEMETRY_BEC_TEMP_VALID;
    }

    if (frame[16] != SPEKTRUM_TELEMETRY_NO_DATA_8) {
        telemetry->becCurrentDeciAmps = frame[16];
        telemetry->valid |= SRXL2_ESC_TELEMETRY_BEC_CURRENT_VALID;
    }

    if (frame[17] != SPEKTRUM_TELEMETRY_NO_DATA_8) {
        telemetry->becVoltageCentiVolts = frame[17] * 5;
        telemetry->valid |= SRXL2_ESC_TELEMETRY_BEC_VOLTAGE_VALID;
    }

    if (frame[18] != SPEKTRUM_TELEMETRY_NO_DATA_8) {
        telemetry->throttleHalfPercent = frame[18];
        telemetry->valid |= SRXL2_ESC_TELEMETRY_THROTTLE_VALID;
    }

    if (frame[19] != SPEKTRUM_TELEMETRY_NO_DATA_8) {
        telemetry->powerOutHalfPercent = frame[19];
        telemetry->valid |= SRXL2_ESC_TELEMETRY_POWER_OUT_VALID;
    }

    return SRXL2_ESC_DECODE_OK;
}
