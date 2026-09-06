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

static uint16_t frameChannelValue(const uint8_t *frame, uint8_t channel)
{
    const size_t offset = 12 + (2 * channel);
    return frame[offset] | ((uint16_t)frame[offset + 1] << 8);
}

static void fillChannels(uint16_t *channels, size_t count, uint16_t firstPulseUs = 1000)
{
    for (size_t i = 0; i < count; i++) {
        channels[i] = firstPulseUs + (i * 20);
    }
}

TEST(Srxl2EscControlTest, SendsFirstFrameImmediatelyThenMaintainsElevenMillisecondCadence)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint16_t channels[9];
    uint16_t failsafe[9];
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    fillChannels(channels, 9);
    fillChannels(failsafe, 9, 1100);
    srxl2EscControlSchedulerInit(&scheduler, 1000, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 9, 75, 4, 1000));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, failsafe, 9));
    const srxl2EscControlSafety_t safety = normalSafety();

    EXPECT_EQ((size_t)(SRXL2_ESC_CONTROL_FRAME_BASE_SIZE + 18),
        srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 1000, &safety, NULL, 0,
            frame, sizeof(frame)));
    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 1000,
        &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 11999,
        &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 12000,
        &safety, NULL, 0, frame, sizeof(frame)));

    // A delayed caller sends one frame and advances to the next cadence slot without a burst.
    EXPECT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 50000,
        &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 55999,
        &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 56000,
        &safety, NULL, 0, frame, sizeof(frame)));
}

TEST(Srxl2EscControlTest, RemainsBlockedUntilTheBusHandshakeCompletes)
{
    srxl2EscBus_t bus = {};
    bus.state = SRXL2_ESC_BUS_LISTEN_GUARD;
    srxl2EscControlScheduler_t scheduler;
    const uint16_t channels[] = {1000, 1500};
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 2, 100, 0, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, channels, 2));
    const srxl2EscControlSafety_t safety = normalSafety();

    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, NULL, 0, frame, sizeof(frame)));
    bus.state = SRXL2_ESC_BUS_RUNNING;
    bus.escDeviceId = ESC_DEVICE_ID;
    EXPECT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, NULL, 0, frame, sizeof(frame)));
}

TEST(Srxl2EscControlTest, CapsInavChannelsAtTheThirtyTwoRepresentableBySrxl2)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint16_t channels[34];
    uint16_t failsafe[34];
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    fillChannels(channels, 34);
    fillChannels(failsafe, 34, 1200);
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 34, 100, 0, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, failsafe, 34));
    const srxl2EscControlSafety_t safety = normalSafety();

    ASSERT_EQ((size_t)SRXL2_ESC_CONTROL_FRAME_MAX_SIZE,
        srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0, &safety, NULL, 0,
            frame, sizeof(frame)));
    EXPECT_EQ(SRXL2_ESC_MAX_CHANNELS, scheduler.channelCount);
    EXPECT_EQ(UINT32_MAX, frameChannelMask(frame));
    EXPECT_EQ(srxl2EscPwmToChannelValue(channels[31]), frameChannelValue(frame, 31));
}

TEST(Srxl2EscControlTest, BuildsCompleteMasksForCommonChannelCounts)
{
    const srxl2EscBus_t bus = runningBus();
    const size_t counts[] = {9, 16, 32};
    uint16_t channels[SRXL2_ESC_MAX_CHANNELS];
    uint16_t failsafe[SRXL2_ESC_MAX_CHANNELS];
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    fillChannels(channels, SRXL2_ESC_MAX_CHANNELS);
    fillChannels(failsafe, SRXL2_ESC_MAX_CHANNELS, 1100);

    for (size_t count : counts) {
        srxl2EscControlScheduler_t scheduler;
        srxl2EscControlSchedulerInit(&scheduler, 0, 0);
        ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, count, 100, 0, 0));
        ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, failsafe, count));
        const srxl2EscControlSafety_t safety = normalSafety();
        ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
            &safety, NULL, 0, frame, sizeof(frame)));

        const uint32_t expectedMask = count == SRXL2_ESC_MAX_CHANNELS ?
            UINT32_MAX : (UINT32_C(1) << count) - 1;
        EXPECT_EQ(expectedMask, frameChannelMask(frame));
    }
}

TEST(Srxl2EscControlTest, PreservesChannelNineAndForwardsEveryConfiguredChannel)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint16_t channels[9];
    uint16_t failsafe[9];
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    fillChannels(channels, 9);
    fillChannels(failsafe, 9, 1100);
    channels[8] = 2000; // Human CH9 is zero-based protocol channel index 8.
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 9, 100, 0, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, failsafe, 9));
    const srxl2EscControlSafety_t safety = normalSafety();

    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_EQ(0x1FFU, frameChannelMask(frame));
    EXPECT_EQ(srxl2EscPwmToChannelValue(2000), frameChannelValue(frame, 8));
    EXPECT_EQ(ESC_DEVICE_ID, frame[4]);
}

TEST(Srxl2EscControlTest, ForcesSafeThrottleWhileDisarmedButLeavesAuxChannelsUnchanged)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint16_t channels[9];
    uint16_t failsafe[9];
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    fillChannels(channels, 9, 1500);
    fillChannels(failsafe, 9, 1000);
    channels[0] = 2000;
    channels[8] = 1800;
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 9, 100, 0, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, failsafe, 9));
    const srxl2EscControlSafety_t safety = normalSafety(false);

    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_EQ(srxl2EscPwmToChannelValue(SRXL2_ESC_CONTROL_SAFE_THROTTLE_US),
        frameChannelValue(frame, 0));
    EXPECT_EQ(srxl2EscPwmToChannelValue(1800), frameChannelValue(frame, 8));
    EXPECT_EQ(0x00, frame[3]);
}

TEST(Srxl2EscControlTest, UsesCompleteFailsafeVectorAndSuppressesTelemetryRequest)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint16_t channels[9];
    uint16_t failsafe[9];
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    fillChannels(channels, 9, 1400);
    fillChannels(failsafe, 9, 1100);
    channels[8] = 2000;
    failsafe[0] = 2000; // Scheduler must still enforce safe throttle.
    failsafe[8] = 1000; // Explicit non-reverse CH9 failsafe value.
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 9, -100, 7, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, failsafe, 9));
    srxl2EscControlSafety_t safety = normalSafety();
    safety.failsafe = true;

    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_EQ(0x01, frame[3]);
    EXPECT_EQ(0x00, frame[4]);
    EXPECT_EQ(srxl2EscPwmToChannelValue(SRXL2_ESC_CONTROL_SAFE_THROTTLE_US),
        frameChannelValue(frame, 0));
    EXPECT_EQ(srxl2EscPwmToChannelValue(1000), frameChannelValue(frame, 8));
}

TEST(Srxl2EscControlTest, StaleInputSelectsFailsafeInsteadOfReusingAuxValues)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint16_t channels[9];
    uint16_t failsafe[9];
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    fillChannels(channels, 9, 1400);
    fillChannels(failsafe, 9, 1200);
    channels[8] = 2000;
    failsafe[8] = 1000;
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 9, -110, 8, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, failsafe, 9));
    const srxl2EscControlSafety_t safety = normalSafety();

    EXPECT_TRUE(srxl2EscControlSchedulerInputIsFresh(&scheduler,
        SRXL2_ESC_CONTROL_INPUT_TIMEOUT_US));
    EXPECT_FALSE(srxl2EscControlSchedulerInputIsFresh(&scheduler,
        SRXL2_ESC_CONTROL_INPUT_TIMEOUT_US + 1));
    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus,
        SRXL2_ESC_CONTROL_INPUT_TIMEOUT_US + 1, &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_EQ(0x01, frame[3]);
    EXPECT_EQ(srxl2EscPwmToChannelValue(1000), frameChannelValue(frame, 8));
}

TEST(Srxl2EscControlTest, RefusesStaleFrameWhenAnyForwardedFailsafeChannelIsUndefined)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint16_t channels[9];
    uint16_t incompleteFailsafe[8];
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    fillChannels(channels, 9);
    fillChannels(incompleteFailsafe, 8);
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 9, 100, 0, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, incompleteFailsafe, 8));
    const srxl2EscControlSafety_t safety = normalSafety();

    EXPECT_FALSE(srxl2EscControlSchedulerFailsafeIsComplete(&scheduler));
    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus,
        SRXL2_ESC_CONTROL_INPUT_TIMEOUT_US + 1, &safety, NULL, 0, frame, sizeof(frame)));
}

TEST(Srxl2EscControlTest, ProgrammingOverridesOnlyRequestedChannelsAndAlwaysKeepsThrottleSafe)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    uint16_t channels[9];
    uint16_t failsafe[9];
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    fillChannels(channels, 9, 1400);
    fillChannels(failsafe, 9, 1000);
    channels[0] = 1800;
    channels[1] = 1450;
    channels[2] = 1550;
    channels[8] = 1200;
    const srxl2EscChannel_t overrides[] = {
        {1, 2000},
        {2, 1000},
        {0, 2000}, // A programming caller cannot override the safe throttle guard.
    };
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 9, 100, 0, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, failsafe, 9));
    srxl2EscControlSafety_t safety = normalSafety(false);
    safety.programming = true;

    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, overrides, sizeof(overrides) / sizeof(overrides[0]), frame, sizeof(frame)));
    EXPECT_EQ(0x00, frame[3]);
    EXPECT_EQ(srxl2EscPwmToChannelValue(1000), frameChannelValue(frame, 0));
    EXPECT_EQ(srxl2EscPwmToChannelValue(2000), frameChannelValue(frame, 1));
    EXPECT_EQ(srxl2EscPwmToChannelValue(1000), frameChannelValue(frame, 2));
    EXPECT_EQ(srxl2EscPwmToChannelValue(1200), frameChannelValue(frame, 8));
}

TEST(Srxl2EscControlTest, UnsafeOrMalformedProgrammingRequestFallsBackToFailsafe)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    const uint16_t channels[] = {1800, 1500, 1500};
    const uint16_t failsafe[] = {1000, 1400, 1600};
    const srxl2EscChannel_t invalidOverride[] = {{3, 2000}};
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    srxl2EscControlSchedulerInit(&scheduler, 0, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 3, 100, 0, 0));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, failsafe, 3));

    srxl2EscControlSafety_t safety = normalSafety(true);
    safety.programming = true;
    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 0,
        &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_EQ(0x01, frame[3]);

    safety.armed = false;
    srxl2EscControlSchedulerResetTiming(&scheduler, 1);
    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 1,
        &safety, invalidOverride, 1, frame, sizeof(frame)));
    EXPECT_EQ(0x01, frame[3]);
}

TEST(Srxl2EscControlTest, TimingAndFreshnessHandleThirtyTwoBitClockWrap)
{
    const srxl2EscBus_t bus = runningBus();
    srxl2EscControlScheduler_t scheduler;
    const uint16_t channels[] = {1500, 1500};
    const uint16_t failsafe[] = {1000, 1500};
    uint8_t frame[SRXL2_ESC_CONTROL_FRAME_MAX_SIZE];
    const uint32_t startedAtUs = UINT32_MAX - 5000;
    srxl2EscControlSchedulerInit(&scheduler, startedAtUs, 0);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 2, 100, 0,
        startedAtUs));
    ASSERT_TRUE(srxl2EscControlSchedulerSetFailsafe(&scheduler, failsafe, 2));
    const srxl2EscControlSafety_t safety = normalSafety();

    ASSERT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, startedAtUs,
        &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_EQ((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 5998,
        &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_NE((size_t)0, srxl2EscControlSchedulerBuildFrame(&scheduler, &bus, 5999,
        &safety, NULL, 0, frame, sizeof(frame)));
    EXPECT_TRUE(srxl2EscControlSchedulerInputIsFresh(&scheduler, 7000));
}

TEST(Srxl2EscControlTest, RejectsInvalidChannelSnapshotsWithoutReplacingValidData)
{
    srxl2EscControlScheduler_t scheduler;
    const uint16_t channels[] = {1000, 1500};
    srxl2EscControlSchedulerInit(&scheduler, 0, 1);
    ASSERT_TRUE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 2, 90, 3, 10));

    EXPECT_FALSE(srxl2EscControlSchedulerUpdateChannels(&scheduler, NULL, 2, 0, 0, 20));
    EXPECT_FALSE(srxl2EscControlSchedulerUpdateChannels(&scheduler, channels, 1, 0, 0, 20));
    EXPECT_EQ(2, scheduler.channelCount);
    EXPECT_EQ(90, scheduler.rssi);
    EXPECT_EQ(10U, scheduler.channelsUpdatedAtUs);
}
