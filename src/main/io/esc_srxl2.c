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

#include <limits.h>
#include <string.h>

#include "platform.h"

#include "common/crc.h"
#include "common/streambuf.h"

#include "io/esc_srxl2.h"

#include "sensors/esc_sensor_data.h"

#define SRXL2_ID                         0xA6
#define SRXL2_PACKET_TYPE_HANDSHAKE      0x21
#define SRXL2_PACKET_TYPE_TELEMETRY      0x80
#define SRXL2_PACKET_TYPE_CONTROL        0xCD
#define SRXL2_CONTROL_COMMAND_CHANNEL    0x00
#define SRXL2_CONTROL_COMMAND_FAILSAFE   0x01
#define SPEKTRUM_TELEMETRY_DEVICE_ESC    0x20
#define SPEKTRUM_TELEMETRY_NO_DATA_16    0xFFFF
#define SPEKTRUM_TELEMETRY_NO_DATA_8     0xFF
#define SPEKTRUM_ESC_THROTTLE_MAX         200

// Packet layout and channel ordering follow the public Spektrum SRXL2 reference implementation:
// https://github.com/SpektrumRC/SRXL2

static void srxl2EscAppendCrc(sbuf_t *dst, uint8_t *start)
{
    const uint16_t crc = crc16_ccitt_update(0, start, (uint32_t)(sbufPtr(dst) - start));
    sbufWriteU16BigEndian(dst, crc);
}

static uint16_t readU16BigEndian(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

static uint32_t readU32LittleEndian(const uint8_t *data)
{
    return data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

uint16_t srxl2EscPwmToChannelValue(uint16_t pulseUs)
{
    const uint16_t constrainedPulseUs = pulseUs < SRXL2_ESC_PWM_MIN ? SRXL2_ESC_PWM_MIN :
        (pulseUs > SRXL2_ESC_PWM_MAX ? SRXL2_ESC_PWM_MAX : pulseUs);
    uint32_t value;

    if (constrainedPulseUs <= SRXL2_ESC_PWM_CENTER) {
        value = SRXL2_ESC_CHANNEL_MIN +
            ((uint32_t)(constrainedPulseUs - SRXL2_ESC_PWM_MIN) *
                (SRXL2_ESC_CHANNEL_CENTER - SRXL2_ESC_CHANNEL_MIN)) /
                (SRXL2_ESC_PWM_CENTER - SRXL2_ESC_PWM_MIN);
    } else {
        value = SRXL2_ESC_CHANNEL_CENTER +
            ((uint32_t)(constrainedPulseUs - SRXL2_ESC_PWM_CENTER) *
                (SRXL2_ESC_CHANNEL_MAX - SRXL2_ESC_CHANNEL_CENTER)) /
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

bool srxl2EscDecodeHandshake(const uint8_t *frame, size_t length, srxl2EscHandshake_t *handshake)
{
    if (!frame || !handshake || length != SRXL2_ESC_HANDSHAKE_FRAME_SIZE ||
        frame[0] != SRXL2_ID || frame[1] != SRXL2_PACKET_TYPE_HANDSHAKE || frame[2] != length ||
        crc16_ccitt_update(0, frame, length) != 0) {
        return false;
    }

    handshake->sourceDeviceId = frame[3];
    handshake->destinationDeviceId = frame[4];
    handshake->priority = frame[5];
    handshake->supportsHighBaud = (frame[6] & 1) != 0;
    handshake->info = frame[7];
    handshake->uid = readU32LittleEndian(&frame[8]);

    return true;
}

size_t srxl2EscBuildControlFrame(uint8_t *frame, size_t capacity, uint8_t replyDeviceId,
    int8_t rssi, uint16_t frameLosses, const srxl2EscChannel_t *channels, size_t channelCount, bool failsafe)
{
    if (!frame || !channels || channelCount == 0 || channelCount > 32) {
        return 0;
    }

    const size_t frameSize = SRXL2_ESC_CONTROL_FRAME_BASE_SIZE + (2 * channelCount);
    if (capacity < frameSize) {
        return 0;
    }

    uint32_t channelMask = 0;
    for (size_t i = 0; i < channelCount; i++) {
        if (channels[i].channel >= 32) {
            return 0;
        }

        const uint32_t channelBit = 1UL << channels[i].channel;
        if (channelMask & channelBit) {
            return 0;
        }
        channelMask |= channelBit;
    }

    sbuf_t buffer;
    sbuf_t *dst = sbufInit(&buffer, frame, frame + capacity);

    sbufWriteU8(dst, SRXL2_ID);
    sbufWriteU8(dst, SRXL2_PACKET_TYPE_CONTROL);
    sbufWriteU8(dst, frameSize);
    sbufWriteU8(dst, failsafe ? SRXL2_CONTROL_COMMAND_FAILSAFE : SRXL2_CONTROL_COMMAND_CHANNEL);
    sbufWriteU8(dst, failsafe ? 0 : replyDeviceId);
    sbufWriteU8(dst, rssi);
    sbufWriteU16(dst, frameLosses);
    sbufWriteU32(dst, channelMask);

    for (uint8_t channel = 0; channel < 32; channel++) {
        const uint32_t channelBit = 1UL << channel;
        if (!(channelMask & channelBit)) {
            continue;
        }

        for (size_t i = 0; i < channelCount; i++) {
            if (channels[i].channel == channel) {
                sbufWriteU16(dst, srxl2EscPwmToChannelValue(channels[i].pulseUs));
                break;
            }
        }
    }
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

    if (frame[18] <= SPEKTRUM_ESC_THROTTLE_MAX) {
        telemetry->throttleHalfPercent = frame[18];
        telemetry->valid |= SRXL2_ESC_TELEMETRY_THROTTLE_VALID;
    }

    if (frame[19] != SPEKTRUM_TELEMETRY_NO_DATA_8) {
        telemetry->powerOutHalfPercent = frame[19];
        telemetry->valid |= SRXL2_ESC_TELEMETRY_POWER_OUT_VALID;
    }

    return SRXL2_ESC_DECODE_OK;
}

uint16_t srxl2EscApplyTelemetry(const srxl2EscTelemetry_t *telemetry, uint8_t motorPoleCount,
    struct escSensorData_s *sensorData)
{
    if (!telemetry || !sensorData) {
        return 0;
    }

    uint16_t applied = 0;

    if ((telemetry->valid & SRXL2_ESC_TELEMETRY_ERPM_VALID) &&
        motorPoleCount >= 2 && (motorPoleCount % 2) == 0) {
        const uint32_t polePairs = motorPoleCount / 2;
        sensorData->rpm = (telemetry->electricalRpm + (polePairs / 2)) / polePairs;
        applied |= SRXL2_ESC_TELEMETRY_ERPM_VALID;
    }

    if (telemetry->valid & SRXL2_ESC_TELEMETRY_VOLTAGE_VALID) {
        sensorData->voltage = telemetry->voltageCentiVolts > INT16_MAX ? INT16_MAX : telemetry->voltageCentiVolts;
        applied |= SRXL2_ESC_TELEMETRY_VOLTAGE_VALID;
    }

    if (telemetry->valid & SRXL2_ESC_TELEMETRY_FET_TEMP_VALID) {
        sensorData->temperature = (telemetry->fetTemperatureDeciCelsius + 5) / 10;
        applied |= SRXL2_ESC_TELEMETRY_FET_TEMP_VALID;
    }

    if (telemetry->valid & SRXL2_ESC_TELEMETRY_CURRENT_VALID) {
        sensorData->current = telemetry->currentCentiAmps;
        applied |= SRXL2_ESC_TELEMETRY_CURRENT_VALID;
    }

    if (applied) {
        sensorData->dataAge = 0;
    }

    return applied;
}
