/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <stdint.h>
#include <cstring>
#include <limits.h>

extern "C" {
    #include "common/crc.h"

    #include "io/esc_srxl2.h"

    #include "sensors/esc_sensor_data.h"
}

#include "gtest/gtest.h"

static void updateFrameCrc(uint8_t *frame, size_t length)
{
    const uint16_t crc = crc16_ccitt_update(0, frame, length - 2);
    frame[length - 2] = crc >> 8;
    frame[length - 1] = crc;
}

TEST(Srxl2EscTest, ConvertsPwmToSrxl2ChannelRange)
{
    EXPECT_EQ(SRXL2_ESC_CHANNEL_MIN, srxl2EscPwmToChannelValue(900));
    EXPECT_EQ(SRXL2_ESC_CHANNEL_MIN, srxl2EscPwmToChannelValue(1000));
    EXPECT_EQ(SRXL2_ESC_CHANNEL_CENTER, srxl2EscPwmToChannelValue(1500));
    EXPECT_EQ(SRXL2_ESC_CHANNEL_MAX, srxl2EscPwmToChannelValue(2000));
    EXPECT_EQ(SRXL2_ESC_CHANNEL_MAX, srxl2EscPwmToChannelValue(2100));
}

TEST(Srxl2EscTest, BuildsHandshakeAtDefaultBaud)
{
    uint8_t frame[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    const uint8_t expected[] = {
        0xA6, 0x21, 0x0E, 0x31, 0x40, 0x0A, 0x00, 0x00,
        0x78, 0x56, 0x34, 0x12, 0x4E, 0xBD,
    };

    EXPECT_EQ(sizeof(frame), srxl2EscBuildHandshake(frame, sizeof(frame), 0x31, 0x40, 10, false, 0, 0x12345678));
    EXPECT_EQ(0, memcmp(expected, frame, sizeof(frame)));
}

TEST(Srxl2EscTest, BuildsSingleChannelControlFrame)
{
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_BASE_SIZE + 2];
    const uint8_t expected[] = {
        0xA6, 0xCD, 0x10, 0x00, 0x40, 0x64, 0x34, 0x12,
        0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x8E, 0xB3,
    };
    const srxl2EscChannel_t channels[] = {{0, 1500}};

    EXPECT_EQ(sizeof(frame), srxl2EscBuildControlFrame(frame, sizeof(frame), 0x40, 100, 0x1234,
        channels, sizeof(channels) / sizeof(channels[0]), false));
    EXPECT_EQ(0, memcmp(expected, frame, sizeof(frame)));
}

TEST(Srxl2EscTest, BuildsMultipleChannelsInChannelOrder)
{
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_BASE_SIZE + 8];
    const srxl2EscChannel_t channels[] = {
        {2, 1000},
        {0, 1250},
        {4, 2000},
        {1, 1750},
    };

    ASSERT_EQ(sizeof(frame), srxl2EscBuildControlFrame(frame, sizeof(frame), 0x40, 100, 0x1234,
        channels, sizeof(channels) / sizeof(channels[0]), false));
    EXPECT_EQ(0x17, frame[8]);
    EXPECT_EQ(0x00, frame[9]);
    EXPECT_EQ(0x00, frame[10]);
    EXPECT_EQ(0x00, frame[11]);
    EXPECT_EQ(srxl2EscPwmToChannelValue(1250), (uint16_t)(frame[12] | (frame[13] << 8)));
    EXPECT_EQ(srxl2EscPwmToChannelValue(1750), (uint16_t)(frame[14] | (frame[15] << 8)));
    EXPECT_EQ(srxl2EscPwmToChannelValue(1000), (uint16_t)(frame[16] | (frame[17] << 8)));
    EXPECT_EQ(srxl2EscPwmToChannelValue(2000), (uint16_t)(frame[18] | (frame[19] << 8)));
    EXPECT_EQ(0, crc16_ccitt_update(0, frame, sizeof(frame)));
}

TEST(Srxl2EscTest, FailsafeControlFrameCannotRequestTelemetry)
{
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_BASE_SIZE + 2];
    const srxl2EscChannel_t channels[] = {{0, 1000}};

    ASSERT_EQ(sizeof(frame), srxl2EscBuildControlFrame(frame, sizeof(frame), 0x40, -100, 3,
        channels, sizeof(channels) / sizeof(channels[0]), true));
    EXPECT_EQ(0x01, frame[3]);
    EXPECT_EQ(0x00, frame[4]);
    EXPECT_EQ(0, crc16_ccitt_update(0, frame, sizeof(frame)));
}

TEST(Srxl2EscTest, RejectsInvalidControlChannels)
{
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    const srxl2EscChannel_t invalidChannel[] = {{32, 1000}};
    const srxl2EscChannel_t duplicateChannels[] = {{0, 1000}, {0, 1500}};

    EXPECT_EQ((size_t)0, srxl2EscBuildControlFrame(frame, sizeof(frame), 0x40, 100, 0,
        invalidChannel, sizeof(invalidChannel) / sizeof(invalidChannel[0]), false));
    EXPECT_EQ((size_t)0, srxl2EscBuildControlFrame(frame, sizeof(frame), 0x40, 100, 0,
        duplicateChannels, sizeof(duplicateChannels) / sizeof(duplicateChannels[0]), false));
    EXPECT_EQ((size_t)0, srxl2EscBuildControlFrame(frame, sizeof(frame), 0x40, 100, 0,
        NULL, 0, false));
}

TEST(Srxl2EscTest, RejectsUndersizedControlBuffer)
{
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_BASE_SIZE + 1];
    const srxl2EscChannel_t channels[] = {{0, 1000}};

    EXPECT_EQ((size_t)0, srxl2EscBuildControlFrame(frame, sizeof(frame), 0x40, 100, 0,
        channels, sizeof(channels) / sizeof(channels[0]), false));
}

TEST(Srxl2EscTest, DecodesEscTelemetryAndUnits)
{
    const uint8_t frame[] = {
        0xA6, 0x80, 0x16, 0x31, 0x20, 0x00,
        0x04, 0xD2, 0x09, 0x92, 0x01, 0xC2, 0x0C, 0x80,
        0x01, 0x90, 0x1B, 0xA8, 0x64, 0x5A, 0x42, 0x77,
    };
    srxl2EscTelemetry_t telemetry;

    ASSERT_EQ(SRXL2_ESC_DECODE_OK, srxl2EscDecodeTelemetry(frame, sizeof(frame), &telemetry));
    EXPECT_EQ((uint16_t)0x1FF, telemetry.valid);
    EXPECT_EQ((uint32_t)12340, telemetry.electricalRpm);
    EXPECT_EQ(2450, telemetry.voltageCentiVolts);
    EXPECT_EQ(450, telemetry.fetTemperatureDeciCelsius);
    EXPECT_EQ(3200, telemetry.currentCentiAmps);
    EXPECT_EQ(400, telemetry.becTemperatureDeciCelsius);
    EXPECT_EQ(27, telemetry.becCurrentDeciAmps);
    EXPECT_EQ(840, telemetry.becVoltageCentiVolts);
    EXPECT_EQ(100, telemetry.throttleHalfPercent);
    EXPECT_EQ(90, telemetry.powerOutHalfPercent);
}

TEST(Srxl2EscTest, PreservesNoDataSemantics)
{
    const uint8_t frame[] = {
        0xA6, 0x80, 0x16, 0x31, 0x20, 0x02,
        0xFF, 0xFF, 0x09, 0x92, 0xFF, 0xFF, 0x00, 0x00,
        0xFF, 0xFF, 0xFF, 0xA8, 0xFF, 0x00, 0x26, 0xD2,
    };
    srxl2EscTelemetry_t telemetry;

    ASSERT_EQ(SRXL2_ESC_DECODE_OK, srxl2EscDecodeTelemetry(frame, sizeof(frame), &telemetry));
    EXPECT_EQ(SRXL2_ESC_TELEMETRY_VOLTAGE_VALID |
        SRXL2_ESC_TELEMETRY_CURRENT_VALID |
        SRXL2_ESC_TELEMETRY_BEC_VOLTAGE_VALID |
        SRXL2_ESC_TELEMETRY_POWER_OUT_VALID, telemetry.valid);
}

TEST(Srxl2EscTest, RejectsTruncatedAndNonEscTelemetry)
{
    uint8_t frame[] = {
        0xA6, 0x80, 0x16, 0x31, 0x20, 0x00,
        0x04, 0xD2, 0x09, 0x92, 0x01, 0xC2, 0x0C, 0x80,
        0x01, 0x90, 0x1B, 0xA8, 0x64, 0x5A, 0x42, 0x77,
    };
    srxl2EscTelemetry_t telemetry;

    EXPECT_EQ(SRXL2_ESC_DECODE_INVALID, srxl2EscDecodeTelemetry(frame, sizeof(frame) - 1, &telemetry));

    frame[4] = 0x0C;
    updateFrameCrc(frame, sizeof(frame));
    EXPECT_EQ(SRXL2_ESC_DECODE_NOT_ESC_TELEMETRY, srxl2EscDecodeTelemetry(frame, sizeof(frame), &telemetry));
}

TEST(Srxl2EscTest, IgnoresOutOfRangeThrottle)
{
    uint8_t frame[] = {
        0xA6, 0x80, 0x16, 0x31, 0x20, 0x00,
        0x04, 0xD2, 0x09, 0x92, 0x01, 0xC2, 0x0C, 0x80,
        0x01, 0x90, 0x1B, 0xA8, 0xC9, 0x5A, 0x00, 0x00,
    };
    srxl2EscTelemetry_t telemetry;
    updateFrameCrc(frame, sizeof(frame));

    ASSERT_EQ(SRXL2_ESC_DECODE_OK, srxl2EscDecodeTelemetry(frame, sizeof(frame), &telemetry));
    EXPECT_EQ(0, telemetry.valid & SRXL2_ESC_TELEMETRY_THROTTLE_VALID);
    EXPECT_NE(0, telemetry.valid & SRXL2_ESC_TELEMETRY_POWER_OUT_VALID);
}

TEST(Srxl2EscTest, MapsStandardTelemetryIntoInavUnits)
{
    srxl2EscTelemetry_t telemetry = {};
    telemetry.valid = SRXL2_ESC_TELEMETRY_ERPM_VALID |
        SRXL2_ESC_TELEMETRY_VOLTAGE_VALID |
        SRXL2_ESC_TELEMETRY_FET_TEMP_VALID |
        SRXL2_ESC_TELEMETRY_CURRENT_VALID;
    telemetry.electricalRpm = 12340;
    telemetry.voltageCentiVolts = 2450;
    telemetry.fetTemperatureDeciCelsius = 456;
    telemetry.currentCentiAmps = 3200;
    escSensorData_t sensorData = {};
    sensorData.dataAge = ESC_DATA_INVALID;

    EXPECT_EQ(telemetry.valid, srxl2EscApplyTelemetry(&telemetry, 14, &sensorData));
    EXPECT_EQ(0, sensorData.dataAge);
    EXPECT_EQ(1763U, sensorData.rpm);
    EXPECT_EQ(2450, sensorData.voltage);
    EXPECT_EQ(46, sensorData.temperature);
    EXPECT_EQ(3200, sensorData.current);
}

TEST(Srxl2EscTest, PreservesUnavailableStandardFields)
{
    srxl2EscTelemetry_t telemetry = {};
    telemetry.valid = SRXL2_ESC_TELEMETRY_VOLTAGE_VALID;
    telemetry.voltageCentiVolts = 1200;
    escSensorData_t sensorData = {};
    sensorData.dataAge = 7;
    sensorData.temperature = 32;
    sensorData.voltage = 1100;
    sensorData.current = 100;
    sensorData.rpm = 2000;

    EXPECT_EQ(SRXL2_ESC_TELEMETRY_VOLTAGE_VALID, srxl2EscApplyTelemetry(&telemetry, 14, &sensorData));
    EXPECT_EQ(0, sensorData.dataAge);
    EXPECT_EQ(32, sensorData.temperature);
    EXPECT_EQ(1200, sensorData.voltage);
    EXPECT_EQ(100, sensorData.current);
    EXPECT_EQ(2000U, sensorData.rpm);
}

TEST(Srxl2EscTest, SkipsRpmForInvalidMotorPoleCountAndClampsVoltage)
{
    srxl2EscTelemetry_t telemetry = {};
    telemetry.valid = SRXL2_ESC_TELEMETRY_ERPM_VALID | SRXL2_ESC_TELEMETRY_VOLTAGE_VALID;
    telemetry.electricalRpm = 10000;
    telemetry.voltageCentiVolts = UINT16_MAX - 1;
    escSensorData_t sensorData = {};
    sensorData.dataAge = 4;
    sensorData.voltage = 1;
    sensorData.rpm = 42;

    EXPECT_EQ(SRXL2_ESC_TELEMETRY_VOLTAGE_VALID, srxl2EscApplyTelemetry(&telemetry, 3, &sensorData));
    EXPECT_EQ(0, sensorData.dataAge);
    EXPECT_EQ(INT16_MAX, sensorData.voltage);
    EXPECT_EQ(42U, sensorData.rpm);
}

TEST(Srxl2EscTest, NonStandardTelemetryDoesNotRefreshInavEscData)
{
    srxl2EscTelemetry_t telemetry = {};
    telemetry.valid = SRXL2_ESC_TELEMETRY_BEC_VOLTAGE_VALID | SRXL2_ESC_TELEMETRY_THROTTLE_VALID;
    telemetry.becVoltageCentiVolts = 840;
    telemetry.throttleHalfPercent = 100;
    escSensorData_t sensorData = {};
    sensorData.dataAge = 7;

    EXPECT_EQ(0, srxl2EscApplyTelemetry(&telemetry, 14, &sensorData));
    EXPECT_EQ(7, sensorData.dataAge);
}

TEST(Srxl2EscTest, RejectsBadCrc)
{
    uint8_t frame[] = {
        0xA6, 0x80, 0x16, 0x31, 0x20, 0x00,
        0x04, 0xD2, 0x09, 0x92, 0x01, 0xC2, 0x0C, 0x80,
        0x01, 0x90, 0x1B, 0xA8, 0x64, 0x5A, 0x42, 0x77,
    };
    srxl2EscTelemetry_t telemetry;
    frame[8] ^= 1;

    EXPECT_EQ(SRXL2_ESC_DECODE_INVALID, srxl2EscDecodeTelemetry(frame, sizeof(frame), &telemetry));
}
