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
    #include "io/esc_srxl2_control.h"
}

#include "gtest/gtest.h"

static const uint8_t ESC_DEVICE_ID = 0x40;

static srxl2EscBus_t runningBus()
{
    srxl2EscBus_t bus = {};
    bus.state = SRXL2_ESC_BUS_RUNNING;
    bus.escDeviceId = ESC_DEVICE_ID;
    return bus;
}

static srxl2EscControlSafety_t normalSafety(bool armed = true)
{
    srxl2EscControlSafety_t safety = {};
    safety.armed = armed;
    return safety;
}

static uint32_t frameChannelMask(const uint8_t *frame)
{
    return frame[8] | ((uint32_t)frame[9] << 8) |
        ((uint32_t)frame[10] << 16) | ((uint32_t)frame[11] << 24);
}

static uint16_t frameFirstChannelValue(const uint8_t *frame)
{
    return frame[12] | ((uint16_t)frame[13] << 8);
}

TEST(Srxl2EscControlTest, SendsOnlyThrottleAtElevenMillisecondCadence)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    srxl2EscControlSchedulerInit(&scheduler, 1000, 2);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateThrottle(&scheduler, 1500, 75, 4, 1000));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafeThrottle(&scheduler, 1000));
    scheduler.telemetryRequestCountdown = SRXL2_ESC_TELEMETRY_REQUEST_INTERVAL_FRAMES - 1;
    const srxl2EscControlSafety_t safety = normalSafety();

    ASSERT_EQ((size_t)(SRXL2_ESC_CONTROL_FRAME_BASE_SIZE + 2),
        srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 1000, &safety,
            frame, sizeof(frame)));
    EXPECT_EQ(UINT32_C(1) << 2, frameChannelMask(frame));
    EXPECT_EQ(srxl2EscPwmToChannelValue(1500), frameFirstChannelValue(frame));
    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 11999,
        &safety, frame, sizeof(frame)));
    EXPECT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 12000,
        &safety, frame, sizeof(frame)));

    // A delayed caller sends one frame and advances without a catch-up burst.
    EXPECT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 50000,
        &safety, frame, sizeof(frame)));
    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 55999,
        &safety, frame, sizeof(frame)));
    EXPECT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 56000,
        &safety, frame, sizeof(frame)));
}

TEST(Srxl2EscControlTest, RequestsTelemetryOnlyEveryTenthControlFrame)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateThrottle(&scheduler, 1500, 100, 0, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafeThrottle(&scheduler, 1000));
    const srxl2EscControlSafety_t safety = normalSafety();

    for (unsigned frameIndex = 0; frameIndex < 21; frameIndex++) {
        const uint32_t nowUs = scheduler.nextFrameAtUs;
        ASSERT_TRUE(srxl2EscControlSchedulerUpdateThrottle(&scheduler, 1500, 100, 0, nowUs));
        ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus,
            nowUs, &safety, frame, sizeof(frame)));
        EXPECT_EQ((frameIndex % SRXL2_ESC_TELEMETRY_REQUEST_INTERVAL_FRAMES) == 0 ?
            ESC_DEVICE_ID : 0, frame[4]);
    }
    EXPECT_EQ(UINT32_C(3), scheduler.telemetryRequestCount);
    EXPECT_EQ(UINT32_C(3), scheduler.telemetryGuardedSlotCount);
}

TEST(Srxl2EscControlTest, TelemetryGrantReservesAnUninterruptedReplyWindow)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    srxl2EscControlSchedulerInit(&scheduler, 1000, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateThrottle(&scheduler, 1500, 100, 0, 1000));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafeThrottle(&scheduler, 1000));
    const srxl2EscControlSafety_t safety = normalSafety();

    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 1000,
        &safety, frame, sizeof(frame)));
    ASSERT_EQ(ESC_DEVICE_ID, frame[4]);
    EXPECT_EQ(UINT32_C(1), scheduler.telemetryRequestCount);
    EXPECT_EQ(UINT32_C(1), scheduler.telemetryGuardedSlotCount);
    EXPECT_EQ(UINT32_C(23000), scheduler.nextFrameAtUs);
    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 22999,
        &safety, frame, sizeof(frame)));
    EXPECT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 23000,
        &safety, frame, sizeof(frame)));
}

TEST(Srxl2EscControlTest, RemainsBlockedUntilTheBusHandshakeCompletes)
{
    srxl2EscBus_t bus = {};
    bus.state = SRXL2_ESC_BUS_LISTEN_GUARD;
    srxl2EscControlScheduler_t scheduler;
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateThrottle(&scheduler, 1000, 100, 0, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafeThrottle(&scheduler, 1000));
    const srxl2EscControlSafety_t safety = normalSafety();

    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, frame, sizeof(frame)));
    bus.state = SRXL2_ESC_BUS_RUNNING;
    bus.escDeviceId = ESC_DEVICE_ID;
    EXPECT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, frame, sizeof(frame)));
}

TEST(Srxl2EscControlTest, ForcesSafeThrottleWhileDisarmed)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateThrottle(&scheduler, 2000, 100, 0, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafeThrottle(&scheduler, 1000));
    const srxl2EscControlSafety_t safety = normalSafety(false);

    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, frame, sizeof(frame)));
    EXPECT_EQ(UINT32_C(1), frameChannelMask(frame));
    EXPECT_EQ(srxl2EscPwmToChannelValue(SRXL2_ESC_CONTROL_SAFE_THROTTLE_US),
        frameFirstChannelValue(frame));
    EXPECT_EQ(0x00, frame[3]);
    EXPECT_EQ(ESC_DEVICE_ID, frame[4]);
}

TEST(Srxl2EscControlTest, FailsafeAndStaleInputSendOnlySafeThrottleWithoutTelemetryRequest)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    srxl2EscControlSchedulerInit(&scheduler, 0, 4);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateThrottle(&scheduler, 1800, -100, 7, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafeThrottle(&scheduler, 1400));
    srxl2EscControlSafety_t safety = normalSafety();
    safety.failsafe = true;

    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, frame, sizeof(frame)));
    EXPECT_EQ(UINT32_C(1) << 4, frameChannelMask(frame));
    EXPECT_EQ(0x01, frame[3]);
    EXPECT_EQ(0x00, frame[4]);
    EXPECT_EQ(srxl2EscPwmToChannelValue(SRXL2_ESC_CONTROL_SAFE_THROTTLE_US),
        frameFirstChannelValue(frame));

    safety.failsafe = false;
    srxl2EscControlSchedulerResetTiming(&scheduler, SRXL2_ESC_CONTROL_INPUT_TIMEOUT_US + 1);
    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus,
        SRXL2_ESC_CONTROL_INPUT_TIMEOUT_US + 1, &safety, frame, sizeof(frame)));
    EXPECT_EQ(0x01, frame[3]);
    EXPECT_EQ(UINT32_C(1) << 4, frameChannelMask(frame));
}

TEST(Srxl2EscControlTest, RefusesFailsafeWhenSafeThrottleIsUndefined)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateThrottle(&scheduler, 1500, 100, 0, 0));
    srxl2EscControlSafety_t safety = normalSafety();
    safety.failsafe = true;

    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, frame, sizeof(frame)));
}

TEST(Srxl2EscControlTest, TimingAndFreshnessHandleThirtyTwoBitClockWrap)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    const uint32_t startedAtUs = UINT32_MAX - 5000;
    srxl2EscControlSchedulerInit(&scheduler, startedAtUs, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateThrottle(&scheduler, 1500, 100, 0, startedAtUs));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafeThrottle(&scheduler, 1000));
    scheduler.telemetryRequestCountdown = 1;
    const srxl2EscControlSafety_t safety = normalSafety();

    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, startedAtUs,
        &safety, frame, sizeof(frame)));
    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 5998,
        &safety, frame, sizeof(frame)));
    EXPECT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 5999,
        &safety, frame, sizeof(frame)));
    EXPECT_TRUE(srxl2EscControlSchedulerInputIsFresh(&scheduler, 7000));
}

TEST(Srxl2EscControlTest, RejectsUnrepresentableThrottleChannel)
{
    srxl2EscControlScheduler_t scheduler;
    srxl2EscControlSchedulerInit(&scheduler, 0, SRXL2_ESC_MAX_CHANNELS);

    EXPECT_FALSE(srxl2EscControlSchedulerUpdateThrottle(&scheduler, 1000, 90, 3, 10));
    EXPECT_FALSE(srxl2EscControlSchedulerSetFailsafeThrottle(&scheduler, 1000));
    EXPECT_FALSE(scheduler.throttleValid);
    EXPECT_FALSE(scheduler.failsafeThrottleValid);
}
