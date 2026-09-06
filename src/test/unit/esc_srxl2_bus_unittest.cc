/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <stdint.h>

extern "C" {
    #include "io/esc_srxl2.h"
    #include "io/esc_srxl2_bus.h"
}

#include "gtest/gtest.h"

static const uint8_t ESC_DEVICE_ID = 0x40;
static const uint32_t FC_UID = 0x12345678;

static void initBus(srxl2EscBus_t *bus, uint32_t nowMs = 0)
{
    srxl2EscBusInit(bus, nowMs, 10, 0, FC_UID);
}

static size_t buildEscHandshake(uint8_t *frame, uint8_t destinationDeviceId, bool supportsHighBaud = true,
    uint8_t sourceDeviceId = ESC_DEVICE_ID)
{
    return srxl2EscBuildHandshake(frame, SRXL2_ESC_HANDSHAKE_FRAME_SIZE, sourceDeviceId,
        destinationDeviceId, 10, supportsHighBaud, 0, 0x87654321);
}

static void advanceBusToFinalHandshake(srxl2EscBus_t *bus, uint32_t nowMs, bool escSupportsHighBaud)
{
    uint8_t input[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    uint8_t output[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];

    ASSERT_EQ(sizeof(input), buildEscHandshake(input, 0, escSupportsHighBaud));
    ASSERT_TRUE(srxl2EscBusProcessFrame(bus, input, sizeof(input), nowMs));
    ASSERT_EQ(sizeof(output), srxl2EscBusBuildPendingFrame(bus, nowMs, output, sizeof(output)));

    ASSERT_EQ(sizeof(input), buildEscHandshake(input, SRXL2_ESC_BUS_MASTER_DEVICE_ID, escSupportsHighBaud));
    ASSERT_TRUE(srxl2EscBusProcessFrame(bus, input, sizeof(input), nowMs + 1));
}

static void advanceBusToRunning(srxl2EscBus_t *bus, uint32_t nowMs = 0, bool escSupportsHighBaud = true)
{
    uint8_t output[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    advanceBusToFinalHandshake(bus, nowMs, escSupportsHighBaud);
    ASSERT_EQ(sizeof(output), srxl2EscBusBuildPendingFrame(bus, nowMs + 1, output, sizeof(output)));
    srxl2EscBusOnFrameTransmitted(bus, nowMs + 2);
    ASSERT_EQ(SRXL2_ESC_BUS_RUNNING, bus->state);
}

TEST(Srxl2EscBusTest, SharedPwmPinRemainsSilentDuringStartupGuard)
{
    srxl2EscBus_t bus;
    uint8_t frame[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    initBus(&bus, 1000);

    EXPECT_EQ((size_t)0, srxl2EscBusBuildPendingFrame(&bus, 1000, frame, sizeof(frame)));
    EXPECT_EQ((size_t)0, srxl2EscBusBuildPendingFrame(&bus, 1199, frame, sizeof(frame)));
    EXPECT_EQ(SRXL2_ESC_BUS_LISTEN_GUARD, bus.state);
    EXPECT_FALSE(srxl2EscBusCanSendControl(&bus));
    EXPECT_EQ(SRXL2_ESC_BUS_BAUD, srxl2EscBusGetBaudRate(&bus));
}

TEST(Srxl2EscBusTest, MissingHandshakeSelectsPwmWithoutSerialTransmission)
{
    srxl2EscBus_t bus;
    uint8_t frame[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    initBus(&bus, 1000);

    EXPECT_EQ((size_t)0, srxl2EscBusBuildPendingFrame(&bus, 1200, frame, sizeof(frame)));
    EXPECT_EQ(SRXL2_ESC_BUS_PWM_FALLBACK, bus.state);
    EXPECT_FALSE(srxl2EscBusCanSendControl(&bus));
}

TEST(Srxl2EscBusTest, ValidEscHandshakeUsesReceiverIdAndStaysAt115200)
{
    srxl2EscBus_t bus;
    uint8_t input[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    uint8_t output[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    srxl2EscHandshake_t decoded;
    initBus(&bus);

    ASSERT_EQ(sizeof(input), buildEscHandshake(input, 0));
    ASSERT_TRUE(srxl2EscBusProcessFrame(&bus, input, sizeof(input), 20));
    ASSERT_EQ(sizeof(output), srxl2EscBusBuildPendingFrame(&bus, 20, output, sizeof(output)));
    ASSERT_TRUE(srxl2EscDecodeHandshake(output, sizeof(output), &decoded));
    EXPECT_EQ(SRXL2_ESC_BUS_MASTER_DEVICE_ID, decoded.sourceDeviceId);
    EXPECT_EQ(ESC_DEVICE_ID, decoded.destinationDeviceId);
    EXPECT_FALSE(decoded.supportsHighBaud);
    EXPECT_EQ(SRXL2_ESC_BUS_WAIT_HANDSHAKE_REPLY, bus.state);

    ASSERT_EQ(sizeof(input), buildEscHandshake(input, SRXL2_ESC_BUS_MASTER_DEVICE_ID));
    ASSERT_TRUE(srxl2EscBusProcessFrame(&bus, input, sizeof(input), 21));
    ASSERT_EQ(sizeof(output), srxl2EscBusBuildPendingFrame(&bus, 21, output, sizeof(output)));
    ASSERT_TRUE(srxl2EscDecodeHandshake(output, sizeof(output), &decoded));
    EXPECT_EQ(SRXL2_ESC_DEVICE_ID_BROADCAST, decoded.destinationDeviceId);
    EXPECT_FALSE(decoded.supportsHighBaud);
    EXPECT_EQ(SRXL2_ESC_BUS_WAIT_FINAL_TX_COMPLETE, bus.state);
    EXPECT_EQ(SRXL2_ESC_BUS_BAUD, srxl2EscBusGetBaudRate(&bus));
    EXPECT_FALSE(srxl2EscBusCanSendControl(&bus));

    srxl2EscBusOnFrameTransmitted(&bus, 22);
    EXPECT_EQ(SRXL2_ESC_BUS_RUNNING, bus.state);
    EXPECT_EQ(SRXL2_ESC_BUS_BAUD, srxl2EscBusGetBaudRate(&bus));
    EXPECT_TRUE(srxl2EscBusCanSendControl(&bus));
}

TEST(Srxl2EscBusTest, StaysAt115200WhenEscDoesNotAdvertiseHighBaud)
{
    srxl2EscBus_t bus;
    initBus(&bus);

    advanceBusToRunning(&bus, 0, false);

    EXPECT_EQ(SRXL2_ESC_BUS_BAUD, srxl2EscBusGetBaudRate(&bus));
}

TEST(Srxl2EscBusTest, InvalidOrNonEscHandshakeCannotUnlockTransmission)
{
    srxl2EscBus_t bus;
    uint8_t frame[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    initBus(&bus);

    ASSERT_EQ(sizeof(frame), buildEscHandshake(frame, 0));
    frame[8] ^= 1;
    EXPECT_FALSE(srxl2EscBusProcessFrame(&bus, frame, sizeof(frame), 20));

    ASSERT_EQ(sizeof(frame), buildEscHandshake(frame, 0, true, 0x31));
    EXPECT_FALSE(srxl2EscBusProcessFrame(&bus, frame, sizeof(frame), 40));

    EXPECT_EQ((size_t)0, srxl2EscBusBuildPendingFrame(&bus, 200, frame, sizeof(frame)));
    EXPECT_EQ(SRXL2_ESC_BUS_PWM_FALLBACK, bus.state);
}

TEST(Srxl2EscBusTest, WrongHandshakeReplyDoesNotCompleteDiscovery)
{
    srxl2EscBus_t bus;
    uint8_t frame[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    initBus(&bus);

    ASSERT_EQ(sizeof(frame), buildEscHandshake(frame, 0));
    ASSERT_TRUE(srxl2EscBusProcessFrame(&bus, frame, sizeof(frame), 10));
    ASSERT_EQ(sizeof(frame), srxl2EscBusBuildPendingFrame(&bus, 10, frame, sizeof(frame)));

    ASSERT_EQ(sizeof(frame), buildEscHandshake(frame, 0x32));
    EXPECT_FALSE(srxl2EscBusProcessFrame(&bus, frame, sizeof(frame), 11));
    EXPECT_EQ(SRXL2_ESC_BUS_WAIT_HANDSHAKE_REPLY, bus.state);
}

TEST(Srxl2EscBusTest, DirectedHandshakeRetriesAtProtocolInterval)
{
    srxl2EscBus_t bus;
    uint8_t frame[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    initBus(&bus);

    ASSERT_EQ(sizeof(frame), buildEscHandshake(frame, 0));
    ASSERT_TRUE(srxl2EscBusProcessFrame(&bus, frame, sizeof(frame), 10));
    ASSERT_EQ(sizeof(frame), srxl2EscBusBuildPendingFrame(&bus, 10, frame, sizeof(frame)));
    EXPECT_EQ((size_t)0, srxl2EscBusBuildPendingFrame(&bus, 59, frame, sizeof(frame)));
    EXPECT_EQ(sizeof(frame), srxl2EscBusBuildPendingFrame(&bus, 60, frame, sizeof(frame)));
    EXPECT_EQ(SRXL2_ESC_BUS_WAIT_HANDSHAKE_REPLY, bus.state);
}

TEST(Srxl2EscBusTest, UndersizedOutputDoesNotAdvanceHandshake)
{
    srxl2EscBus_t bus;
    uint8_t input[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    uint8_t output[SRXL2_ESC_HANDSHAKE_FRAME_SIZE - 1];
    initBus(&bus);

    ASSERT_EQ(SRXL2_ESC_HANDSHAKE_FRAME_SIZE, buildEscHandshake(input, 0));
    ASSERT_TRUE(srxl2EscBusProcessFrame(&bus, input, sizeof(input), 10));
    EXPECT_EQ((size_t)0, srxl2EscBusBuildPendingFrame(&bus, 10, output, sizeof(output)));
    EXPECT_EQ(SRXL2_ESC_BUS_SEND_DIRECTED_HANDSHAKE, bus.state);
}

TEST(Srxl2EscBusTest, EscBrownoutRestartsHandshakeAtDefaultBaud)
{
    srxl2EscBus_t bus;
    uint8_t frame[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    initBus(&bus);
    advanceBusToRunning(&bus);
    ASSERT_EQ(SRXL2_ESC_BUS_BAUD, srxl2EscBusGetBaudRate(&bus));

    ASSERT_EQ(sizeof(frame), buildEscHandshake(frame, 0));
    ASSERT_TRUE(srxl2EscBusProcessFrame(&bus, frame, sizeof(frame), 100));

    EXPECT_EQ(SRXL2_ESC_BUS_SEND_DIRECTED_HANDSHAKE, bus.state);
    EXPECT_EQ(SRXL2_ESC_BUS_BAUD, srxl2EscBusGetBaudRate(&bus));
    EXPECT_FALSE(srxl2EscBusCanSendControl(&bus));
}

TEST(Srxl2EscBusTest, LateEscCanBeDiscoveredOnlyAfterExplicitSafeRestart)
{
    srxl2EscBus_t bus;
    uint8_t frame[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    initBus(&bus);
    ASSERT_EQ((size_t)0, srxl2EscBusBuildPendingFrame(&bus, SRXL2_ESC_BUS_STARTUP_GUARD_MS,
        frame, sizeof(frame)));
    ASSERT_EQ(SRXL2_ESC_BUS_PWM_FALLBACK, bus.state);

    ASSERT_EQ(sizeof(frame), buildEscHandshake(frame, 0));
    EXPECT_FALSE(srxl2EscBusProcessFrame(&bus, frame, sizeof(frame), 500));

    srxl2EscBusRestartDiscovery(&bus, 1000);
    EXPECT_EQ(SRXL2_ESC_BUS_LISTEN_GUARD, bus.state);
    EXPECT_EQ(SRXL2_ESC_BUS_BAUD, srxl2EscBusGetBaudRate(&bus));
    EXPECT_FALSE(srxl2EscBusCanSendControl(&bus));
    EXPECT_TRUE(srxl2EscBusProcessFrame(&bus, frame, sizeof(frame), 1100));
    EXPECT_EQ(SRXL2_ESC_BUS_SEND_DIRECTED_HANDSHAKE, bus.state);
}

TEST(Srxl2EscBusTest, ControlFramesAreBlockedUntilHandshakeCompletes)
{
    srxl2EscBus_t bus;
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_BASE_SIZE + 2];
    const srxl2EscChannel_t channels[] = {{0, 1000}};
    initBus(&bus);

    EXPECT_EQ((size_t)0, srxl2EscBusBuildControlFrame(&bus, frame, sizeof(frame), ESC_DEVICE_ID,
        100, 0, channels, sizeof(channels) / sizeof(channels[0]), false));

    advanceBusToRunning(&bus);
    EXPECT_EQ(sizeof(frame), srxl2EscBusBuildControlFrame(&bus, frame, sizeof(frame), ESC_DEVICE_ID,
        100, 0, channels, sizeof(channels) / sizeof(channels[0]), false));
}

TEST(Srxl2EscBusTest, TelemetryFreshnessExpiresAndHandlesClockWrap)
{
    srxl2EscBus_t bus;
    initBus(&bus);
    advanceBusToRunning(&bus);

    EXPECT_FALSE(srxl2EscBusTelemetryIsFresh(&bus, 10, 10));
    srxl2EscBusNoteTelemetry(&bus, UINT32_MAX - 5);
    EXPECT_TRUE(srxl2EscBusTelemetryIsFresh(&bus, 3, 10));
    EXPECT_FALSE(srxl2EscBusTelemetryIsFresh(&bus, 6, 10));
}

TEST(Srxl2EscBusTest, DisabledBusCannotTransmitOrRefreshTelemetry)
{
    srxl2EscBus_t bus;
    uint8_t frame[SRXL2_ESC_HANDSHAKE_FRAME_SIZE];
    initBus(&bus);
    advanceBusToRunning(&bus);

    srxl2EscBusDisable(&bus);
    srxl2EscBusNoteTelemetry(&bus, 100);

    EXPECT_EQ(SRXL2_ESC_BUS_DISABLED, bus.state);
    EXPECT_FALSE(srxl2EscBusCanSendControl(&bus));
    EXPECT_FALSE(srxl2EscBusTelemetryIsFresh(&bus, 100, 10));
    EXPECT_EQ((size_t)0, srxl2EscBusBuildPendingFrame(&bus, 100, frame, sizeof(frame)));
}
