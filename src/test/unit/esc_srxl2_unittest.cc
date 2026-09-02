/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <stdint.h>
#include <cstring>

extern "C" {
    #include "io/esc_srxl2.h"
}

#include "gtest/gtest.h"

TEST(Srxl2EscTest, ConvertsPwmToSrxl2ChannelRange)
{
    EXPECT_EQ(0x0000, srxl2EscPwmToChannelValue(900));
    EXPECT_EQ(0x0000, srxl2EscPwmToChannelValue(1000));
    EXPECT_EQ(0x8000, srxl2EscPwmToChannelValue(1500));
    EXPECT_EQ(0xFFFC, srxl2EscPwmToChannelValue(2000));
    EXPECT_EQ(0xFFFC, srxl2EscPwmToChannelValue(2100));
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
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_SIZE];
    const uint8_t expected[] = {
        0xA6, 0xCD, 0x10, 0x00, 0x40, 0x64, 0x34, 0x12,
        0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x8E, 0xB3,
    };

    EXPECT_EQ(sizeof(frame), srxl2EscBuildControlFrame(frame, sizeof(frame), 0x40, 100, 0x1234, 0, 1500, false));
    EXPECT_EQ(0, memcmp(expected, frame, sizeof(frame)));
}

TEST(Srxl2EscTest, RejectsInvalidControlChannel)
{
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_SIZE];

    EXPECT_EQ((size_t)0, srxl2EscBuildControlFrame(frame, sizeof(frame), 0x40, 100, 0, 32, 1000, false));
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
