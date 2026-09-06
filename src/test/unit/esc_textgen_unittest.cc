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
    #include "io/esc_textgen.h"
}

#include "gtest/gtest.h"

static escTextGenSafety_t safeConditions()
{
    escTextGenSafety_t safety = {};
    safety.throttleLow = true;
    safety.thrustReverseNormal = true;
    safety.linkAvailable = true;
    return safety;
}

static void makeTextPayload(uint8_t *payload, uint8_t instance, uint8_t lineNumber, const char *text)
{
    memset(payload, 0, ESC_TEXTGEN_PAYLOAD_SIZE);
    payload[0] = ESC_TEXTGEN_SENSOR_ID;
    payload[1] = instance;
    payload[2] = lineNumber;
    if (text) {
        const size_t length = strlen(text) < ESC_TEXTGEN_LINE_LENGTH ? strlen(text) : ESC_TEXTGEN_LINE_LENGTH;
        memcpy(&payload[3], text, length);
    }
}

TEST(EscTextGenDisplayTest, AppliesTitleAndMenuLines)
{
    escTextGenDisplay_t display;
    uint8_t payload[ESC_TEXTGEN_PAYLOAD_SIZE];
    escTextGenDisplayInit(&display);

    makeTextPayload(payload, 2, 0, "AVIAN ESC");
    EXPECT_EQ(ESC_TEXTGEN_PAYLOAD_LINE_UPDATED,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));
    makeTextPayload(payload, 2, 8, "Save & Exit");
    EXPECT_EQ(ESC_TEXTGEN_PAYLOAD_LINE_UPDATED,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));

    EXPECT_TRUE(display.instanceSelected);
    EXPECT_EQ(2, display.instance);
    EXPECT_STREQ("AVIAN ESC", display.lines[0]);
    EXPECT_STREQ("Save & Exit", display.lines[8]);
    EXPECT_EQ((1U << 0) | (1U << 8), display.validLineMask);
    EXPECT_EQ(2U, display.revision);
}

TEST(EscTextGenDisplayTest, PreservesFullWidthTextAndSpaces)
{
    escTextGenDisplay_t display;
    uint8_t payload[ESC_TEXTGEN_PAYLOAD_SIZE];
    escTextGenDisplayInit(&display);
    makeTextPayload(payload, 0, 1, "123 56789 ABC");

    ASSERT_EQ(ESC_TEXTGEN_PAYLOAD_LINE_UPDATED,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));
    EXPECT_STREQ("123 56789 ABC", display.lines[1]);
    EXPECT_EQ('\0', display.lines[1][ESC_TEXTGEN_LINE_LENGTH]);
}

TEST(EscTextGenDisplayTest, RejectsInvalidLineWithoutChangingDisplay)
{
    escTextGenDisplay_t display;
    uint8_t payload[ESC_TEXTGEN_PAYLOAD_SIZE];
    escTextGenDisplayInit(&display);
    makeTextPayload(payload, 0, 9, "invalid");

    EXPECT_EQ(ESC_TEXTGEN_PAYLOAD_INVALID,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));
    EXPECT_FALSE(display.instanceSelected);
    EXPECT_EQ(0, display.validLineMask);
    EXPECT_EQ(0U, display.revision);
}

TEST(EscTextGenDisplayTest, RejectsControlCharactersWithoutChangingDisplay)
{
    escTextGenDisplay_t display;
    uint8_t payload[ESC_TEXTGEN_PAYLOAD_SIZE];
    escTextGenDisplayInit(&display);
    makeTextPayload(payload, 0, 1, "valid");
    payload[4] = '\n';

    EXPECT_EQ(ESC_TEXTGEN_PAYLOAD_INVALID,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));
    EXPECT_FALSE(display.instanceSelected);
    EXPECT_EQ(0, display.validLineMask);
}

TEST(EscTextGenDisplayTest, IgnoresPaddingAfterNullTerminator)
{
    escTextGenDisplay_t display;
    uint8_t payload[ESC_TEXTGEN_PAYLOAD_SIZE];
    escTextGenDisplayInit(&display);
    makeTextPayload(payload, 0, 1, "OK");
    payload[8] = 0x01;

    EXPECT_EQ(ESC_TEXTGEN_PAYLOAD_LINE_UPDATED,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));
    EXPECT_STREQ("OK", display.lines[1]);
}

TEST(EscTextGenDisplayTest, DoesNotMixInstances)
{
    escTextGenDisplay_t display;
    uint8_t payload[ESC_TEXTGEN_PAYLOAD_SIZE];
    escTextGenDisplayInit(&display);
    makeTextPayload(payload, 1, 0, "FIRST");
    ASSERT_EQ(ESC_TEXTGEN_PAYLOAD_LINE_UPDATED,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));

    makeTextPayload(payload, 2, 1, "SECOND");
    EXPECT_EQ(ESC_TEXTGEN_PAYLOAD_INSTANCE_MISMATCH,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));
    EXPECT_STREQ("FIRST", display.lines[0]);
    EXPECT_EQ(0, display.lines[1][0]);
    EXPECT_EQ(1U, display.revision);
}

TEST(EscTextGenDisplayTest, HandlesRefreshAndAcknowledgement)
{
    escTextGenDisplay_t display;
    uint8_t payload[ESC_TEXTGEN_PAYLOAD_SIZE];
    escTextGenDisplayInit(&display);
    makeTextPayload(payload, 0, ESC_TEXTGEN_LINE_REFRESH, NULL);

    EXPECT_EQ(ESC_TEXTGEN_PAYLOAD_REFRESH,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));
    EXPECT_TRUE(display.refreshPending);
    EXPECT_EQ(1U, display.revision);

    escTextGenDisplayAcknowledgeRefresh(&display);
    EXPECT_FALSE(display.refreshPending);
}

TEST(EscTextGenDisplayTest, ClearErasesEveryLine)
{
    escTextGenDisplay_t display;
    uint8_t payload[ESC_TEXTGEN_PAYLOAD_SIZE];
    escTextGenDisplayInit(&display);
    makeTextPayload(payload, 0, 0, "AVIAN");
    ASSERT_EQ(ESC_TEXTGEN_PAYLOAD_LINE_UPDATED,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));
    makeTextPayload(payload, 0, ESC_TEXTGEN_LINE_CLEAR, NULL);

    EXPECT_EQ(ESC_TEXTGEN_PAYLOAD_CLEARED,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));
    EXPECT_EQ(0, display.validLineMask);
    EXPECT_EQ(0, display.lines[0][0]);
    EXPECT_TRUE(display.refreshPending);
    EXPECT_EQ(2U, display.revision);
}

TEST(EscTextGenDisplayTest, RejectsWrongSensorAndPayloadLength)
{
    escTextGenDisplay_t display;
    uint8_t payload[ESC_TEXTGEN_PAYLOAD_SIZE];
    escTextGenDisplayInit(&display);
    makeTextPayload(payload, 0, 0, "AVIAN");

    EXPECT_EQ(ESC_TEXTGEN_PAYLOAD_INVALID,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload) - 1));
    payload[0] = 0x20;
    EXPECT_EQ(ESC_TEXTGEN_PAYLOAD_NOT_TEXTGEN,
        escTextGenDisplayApplyPayload(&display, payload, sizeof(payload)));
}

TEST(EscTextGenSessionTest, RequiresDisarmLowThrottleHealthyLinkAndNoFailsafe)
{
    escTextGenSession_t session;
    escTextGenSessionInit(&session);

    escTextGenSafety_t safety = safeConditions();
    safety.armed = true;
    EXPECT_FALSE(escTextGenSessionEnter(&session, 0, &safety));
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_ARMED, session.stopReason);

    safety = safeConditions();
    safety.throttleLow = false;
    EXPECT_FALSE(escTextGenSessionEnter(&session, 0, &safety));
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_THROTTLE_NOT_LOW, session.stopReason);

    safety = safeConditions();
    safety.thrustReverseNormal = false;
    EXPECT_FALSE(escTextGenSessionEnter(&session, 0, &safety));
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_THRUST_REVERSE_ACTIVE, session.stopReason);

    safety = safeConditions();
    safety.failsafe = true;
    EXPECT_FALSE(escTextGenSessionEnter(&session, 0, &safety));
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_FAILSAFE, session.stopReason);

    safety = safeConditions();
    safety.linkAvailable = false;
    EXPECT_FALSE(escTextGenSessionEnter(&session, 0, &safety));
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_LINK_LOST, session.stopReason);
}

TEST(EscTextGenSessionTest, ActiveSessionForcesSafeNeutralChannels)
{
    escTextGenSession_t session;
    escTextGenSessionInit(&session);
    const escTextGenSafety_t safety = safeConditions();
    ASSERT_TRUE(escTextGenSessionEnter(&session, 10, &safety));

    const escTextGenChannelOverride_t channelOverride = escTextGenSessionGetChannelOverride(&session, 10);
    EXPECT_TRUE(channelOverride.active);
    EXPECT_EQ(ESC_TEXTGEN_THROTTLE_SAFE_US, channelOverride.throttlePulseUs);
    EXPECT_EQ(ESC_TEXTGEN_CHANNEL_CENTER_US, channelOverride.aileronPulseUs);
    EXPECT_EQ(ESC_TEXTGEN_CHANNEL_CENTER_US, channelOverride.elevatorPulseUs);
}

TEST(EscTextGenSessionTest, NavigationUsesBoundedPulsesThenReturnsNeutral)
{
    escTextGenSession_t session;
    escTextGenSessionInit(&session);
    const escTextGenSafety_t safety = safeConditions();
    ASSERT_TRUE(escTextGenSessionEnter(&session, 0, &safety));
    ASSERT_TRUE(escTextGenSessionRequestNavigation(&session, ESC_TEXTGEN_NAVIGATION_UP, 100));

    escTextGenChannelOverride_t channelOverride = escTextGenSessionGetChannelOverride(&session, 100);
    EXPECT_EQ(ESC_TEXTGEN_CHANNEL_MAX_US, channelOverride.elevatorPulseUs);
    EXPECT_EQ(ESC_TEXTGEN_CHANNEL_CENTER_US, channelOverride.aileronPulseUs);
    EXPECT_EQ(ESC_TEXTGEN_THROTTLE_SAFE_US, channelOverride.throttlePulseUs);

    EXPECT_FALSE(escTextGenSessionRequestNavigation(&session, ESC_TEXTGEN_NAVIGATION_RIGHT, 349));
    channelOverride = escTextGenSessionGetChannelOverride(&session, 350);
    EXPECT_EQ(ESC_TEXTGEN_CHANNEL_CENTER_US, channelOverride.elevatorPulseUs);
    EXPECT_TRUE(escTextGenSessionRequestNavigation(&session, ESC_TEXTGEN_NAVIGATION_RIGHT, 350));
    channelOverride = escTextGenSessionGetChannelOverride(&session, 350);
    EXPECT_EQ(ESC_TEXTGEN_CHANNEL_MAX_US, channelOverride.aileronPulseUs);
}

TEST(EscTextGenSessionTest, AllNavigationDirectionsHaveExplicitPolarity)
{
    escTextGenSession_t session;
    escTextGenSessionInit(&session);
    const escTextGenSafety_t safety = safeConditions();
    ASSERT_TRUE(escTextGenSessionEnter(&session, 0, &safety));

    ASSERT_TRUE(escTextGenSessionRequestNavigation(&session, ESC_TEXTGEN_NAVIGATION_DOWN, 0));
    EXPECT_EQ(ESC_TEXTGEN_CHANNEL_MIN_US,
        escTextGenSessionGetChannelOverride(&session, 0).elevatorPulseUs);
    ASSERT_TRUE(escTextGenSessionRequestNavigation(&session, ESC_TEXTGEN_NAVIGATION_LEFT,
        ESC_TEXTGEN_NAVIGATION_PULSE_MS));
    EXPECT_EQ(ESC_TEXTGEN_CHANNEL_MIN_US,
        escTextGenSessionGetChannelOverride(&session, ESC_TEXTGEN_NAVIGATION_PULSE_MS).aileronPulseUs);
}

TEST(EscTextGenSessionTest, SafetyChangesAbortAndReleaseOverrides)
{
    escTextGenSession_t session;
    escTextGenSessionInit(&session);
    escTextGenSafety_t safety = safeConditions();
    ASSERT_TRUE(escTextGenSessionEnter(&session, 0, &safety));
    safety.armed = true;

    escTextGenSessionUpdate(&session, 1, &safety);

    EXPECT_FALSE(session.active);
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_ARMED, session.stopReason);
    EXPECT_FALSE(escTextGenSessionGetChannelOverride(&session, 1).active);
}

TEST(EscTextGenSessionTest, FailsafeLinkLossRaisedThrottleAndReverseAbort)
{
    escTextGenSession_t session;
    escTextGenSafety_t safety = safeConditions();
    escTextGenSessionInit(&session);

    ASSERT_TRUE(escTextGenSessionEnter(&session, 0, &safety));
    safety.failsafe = true;
    escTextGenSessionUpdate(&session, 1, &safety);
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_FAILSAFE, session.stopReason);

    safety = safeConditions();
    ASSERT_TRUE(escTextGenSessionEnter(&session, 2, &safety));
    safety.linkAvailable = false;
    escTextGenSessionUpdate(&session, 3, &safety);
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_LINK_LOST, session.stopReason);

    safety = safeConditions();
    ASSERT_TRUE(escTextGenSessionEnter(&session, 4, &safety));
    safety.throttleLow = false;
    escTextGenSessionUpdate(&session, 5, &safety);
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_THROTTLE_NOT_LOW, session.stopReason);

    safety = safeConditions();
    ASSERT_TRUE(escTextGenSessionEnter(&session, 6, &safety));
    safety.thrustReverseNormal = false;
    escTextGenSessionUpdate(&session, 7, &safety);
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_THRUST_REVERSE_ACTIVE, session.stopReason);
}

TEST(EscTextGenSessionTest, ActivityExtendsTimeout)
{
    escTextGenSession_t session;
    escTextGenSessionInit(&session);
    const escTextGenSafety_t safety = safeConditions();
    ASSERT_TRUE(escTextGenSessionEnter(&session, 100, &safety));

    escTextGenSessionUpdate(&session, 100 + ESC_TEXTGEN_SESSION_TIMEOUT_MS, &safety);
    ASSERT_TRUE(session.active);
    escTextGenSessionNoteActivity(&session, 2000);
    escTextGenSessionUpdate(&session, 2000 + ESC_TEXTGEN_SESSION_TIMEOUT_MS, &safety);
    ASSERT_TRUE(session.active);
    escTextGenSessionUpdate(&session, 2001 + ESC_TEXTGEN_SESSION_TIMEOUT_MS, &safety);
    EXPECT_FALSE(session.active);
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_TIMEOUT, session.stopReason);
}

TEST(EscTextGenSessionTest, TimeoutAndNavigationHandleClockWrap)
{
    escTextGenSession_t session;
    escTextGenSessionInit(&session);
    const escTextGenSafety_t safety = safeConditions();
    ASSERT_TRUE(escTextGenSessionEnter(&session, UINT32_MAX - 100, &safety));
    ASSERT_TRUE(escTextGenSessionRequestNavigation(&session, ESC_TEXTGEN_NAVIGATION_LEFT,
        UINT32_MAX - 10));

    EXPECT_EQ(ESC_TEXTGEN_CHANNEL_MIN_US,
        escTextGenSessionGetChannelOverride(&session, 20).aileronPulseUs);
    escTextGenSessionUpdate(&session, ESC_TEXTGEN_SESSION_TIMEOUT_MS - 101, &safety);
    EXPECT_TRUE(session.active);
    escTextGenSessionUpdate(&session, ESC_TEXTGEN_SESSION_TIMEOUT_MS - 100, &safety);
    EXPECT_FALSE(session.active);
}

TEST(EscTextGenSessionTest, ExplicitAndMalformedStopsReleaseOverrides)
{
    escTextGenSession_t session;
    escTextGenSessionInit(&session);
    const escTextGenSafety_t safety = safeConditions();
    ASSERT_TRUE(escTextGenSessionEnter(&session, 0, &safety));

    escTextGenSessionStop(&session, ESC_TEXTGEN_SESSION_STOP_MALFORMED_TELEMETRY);
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_MALFORMED_TELEMETRY, session.stopReason);
    EXPECT_FALSE(escTextGenSessionGetChannelOverride(&session, 0).active);

    ASSERT_TRUE(escTextGenSessionEnter(&session, 1, &safety));
    escTextGenSessionStop(&session, ESC_TEXTGEN_SESSION_STOP_USER);
    EXPECT_EQ(ESC_TEXTGEN_SESSION_STOP_USER, session.stopReason);
    EXPECT_FALSE(escTextGenSessionGetChannelOverride(&session, 1).active);
}

TEST(EscTextGenSessionTest, PowerCycleRequirementSurvivesExitAndResetsOnNewSession)
{
    escTextGenSession_t session;
    escTextGenSessionInit(&session);
    const escTextGenSafety_t safety = safeConditions();
    ASSERT_TRUE(escTextGenSessionEnter(&session, 0, &safety));

    escTextGenSessionMarkSettingChanged(&session);
    EXPECT_TRUE(session.powerCycleRequired);
    escTextGenSessionStop(&session, ESC_TEXTGEN_SESSION_STOP_USER);
    EXPECT_TRUE(session.powerCycleRequired);

    ASSERT_TRUE(escTextGenSessionEnter(&session, 1, &safety));
    EXPECT_FALSE(session.powerCycleRequired);
}
