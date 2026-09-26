/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <vector>

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"
#include "common/axis.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "io/gps.h"
#include "navigation/navigation.h"
#include "sensors/battery.h"
#include "sensors/bec_voltage.h"
#include "sensors/sensors.h"
#include "sensors/pitotmeter.h"
#include "sensors/temperature.h"
#include "telemetry/crsf.h"
#include "telemetry/msp_shared.h"
#include "telemetry/telemetry.h"

PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
PG_REGISTER(navConfig_t, navConfig, PG_NAV_CONFIG, 0);

uint32_t armingFlags;
uint32_t flightModeFlags;
uint32_t stateFlags;
attitudeEulerAngles_t attitude;
gpsSolutionData_t gpsSol;
}

namespace {
bool becConfigured;
bool becAvailable;
uint16_t becCentivolts;
bool telemetryBufferEmpty;
timeUs_t nowUs = 1000000;

struct SentFrame {
    timeUs_t timestamp;
    std::vector<uint8_t> bytes;
};
std::vector<SentFrame> sentFrames;

// Reference CRSF vectors include the CRC over type and payload only.
const std::vector<uint8_t> bec6120mV = {0xC8, 0x05, 0x0E, 0xC8, 0x17, 0xE8, 0x28};
const std::vector<uint8_t> becZero = {0xC8, 0x05, 0x0E, 0xC8, 0x00, 0x00, 0x56};
const std::vector<uint8_t> battery = {
    0xC8, 0x0A, 0x08, 0x00, 0xF7, 0x00, 0x7B, 0x01, 0x02, 0x03, 0x40, 0x2A
};

class CrsfBecVoltageTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        becConfigured = false;
        becAvailable = false;
        becCentivolts = 612;
        telemetryBufferEmpty = true;
        std::memset(telemetryConfigMutable(), 0, sizeof(telemetryConfig_t));
        sentFrames.clear();
        nowUs += 1000000;
    }

    std::vector<uint8_t> getFrame(crsfFrameType_e type)
    {
        uint8_t frame[CRSF_FRAME_SIZE_MAX];
        const int size = getCrsfFrame(frame, type);
        return std::vector<uint8_t>(frame, frame + size);
    }

    void tick(timeUs_t interval)
    {
        nowUs += interval;
        handleCrsfTelemetry(nowUs);
    }
};
}

TEST_F(CrsfBecVoltageTest, EncodesGeneralVoltageSourceAndMillivoltsWithValidCrc)
{
    becConfigured = becAvailable = true;
    EXPECT_EQ(bec6120mV, getFrame(CRSF_FRAMETYPE_VOLTAGE));
}

TEST_F(CrsfBecVoltageTest, ZeroIsAValidMeasurement)
{
    becConfigured = becAvailable = true;
    becCentivolts = 0;
    EXPECT_EQ(becZero, getFrame(CRSF_FRAMETYPE_VOLTAGE));
}

TEST_F(CrsfBecVoltageTest, UnconfiguredOrUnavailableMeasurementProducesNoFrame)
{
    EXPECT_TRUE(getFrame(CRSF_FRAMETYPE_VOLTAGE).empty());
    becConfigured = true;
    EXPECT_TRUE(getFrame(CRSF_FRAMETYPE_VOLTAGE).empty());
    becAvailable = true;
    EXPECT_FALSE(getFrame(CRSF_FRAMETYPE_VOLTAGE).empty());
    becAvailable = false; // the sensor also returns false after its freshness timeout
    EXPECT_TRUE(getFrame(CRSF_FRAMETYPE_VOLTAGE).empty());
}

TEST_F(CrsfBecVoltageTest, RejectsUnrepresentableVoltageInsteadOfWrapping)
{
    becConfigured = becAvailable = true;
    becCentivolts = 6553;
    const auto largestFrame = getFrame(CRSF_FRAMETYPE_VOLTAGE);
    ASSERT_EQ(7u, largestFrame.size());
    EXPECT_EQ(0xFF, largestFrame[4]);
    EXPECT_EQ(0xFA, largestFrame[5]);
    becCentivolts = 6554;
    EXPECT_TRUE(getFrame(CRSF_FRAMETYPE_VOLTAGE).empty());
    becCentivolts = UINT16_MAX;
    EXPECT_TRUE(getFrame(CRSF_FRAMETYPE_VOLTAGE).empty());
}

TEST_F(CrsfBecVoltageTest, ExistingBatteryPayloadIsIndependentOfBecAvailability)
{
    EXPECT_EQ(battery, getFrame(CRSF_FRAMETYPE_BATTERY_SENSOR));
    becConfigured = becAvailable = true;
    EXPECT_EQ(battery, getFrame(CRSF_FRAMETYPE_BATTERY_SENSOR));
    becAvailable = false;
    EXPECT_EQ(battery, getFrame(CRSF_FRAMETYPE_BATTERY_SENSOR));
}

TEST_F(CrsfBecVoltageTest, DisabledBecPreservesExistingScheduleAndBatteryCadence)
{
    initCrsfTelemetry();
    tick(0);
    for (unsigned i = 1; i < 9; i++) {
        tick(33333);
    }
    ASSERT_EQ(9u, sentFrames.size());
    const uint8_t expectedTypes[] = {0x1E, 0x08, 0x21};
    for (unsigned i = 0; i < sentFrames.size(); i++) {
        EXPECT_EQ(expectedTypes[i % 3], sentFrames[i].bytes[2]);
    }
    EXPECT_EQ(battery, sentFrames[1].bytes);
    EXPECT_EQ(99999u, sentFrames[4].timestamp - sentFrames[1].timestamp);
    EXPECT_EQ(99999u, sentFrames[7].timestamp - sentFrames[4].timestamp);
}

TEST_F(CrsfBecVoltageTest, EnabledBecUsesOwnSlotAndRetainsTenHertzBatteryRate)
{
    becConfigured = becAvailable = true;
    initCrsfTelemetry();
    tick(0);
    for (unsigned i = 1; i < 12; i++) {
        tick(25000);
    }
    ASSERT_EQ(12u, sentFrames.size());
    const uint8_t expectedTypes[] = {0x1E, 0x08, 0x21, 0x0E};
    for (unsigned i = 0; i < sentFrames.size(); i++) {
        EXPECT_EQ(expectedTypes[i % 4], sentFrames[i].bytes[2]);
    }
    EXPECT_EQ(bec6120mV, sentFrames[3].bytes);
    EXPECT_EQ(battery, sentFrames[1].bytes);
    EXPECT_EQ(100000u, sentFrames[5].timestamp - sentFrames[1].timestamp);
    EXPECT_EQ(100000u, sentFrames[7].timestamp - sentFrames[3].timestamp);
}

TEST_F(CrsfBecVoltageTest, UnavailableScheduledSensorDoesNotPublishAPlaceholder)
{
    becConfigured = true;
    initCrsfTelemetry();
    tick(0);
    tick(25000);
    tick(25000);
    tick(25000);
    ASSERT_EQ(3u, sentFrames.size());
    becAvailable = true;
    becCentivolts = 0;
    for (unsigned i = 0; i < 4; i++) {
        tick(25000);
    }
    ASSERT_EQ(7u, sentFrames.size());
    EXPECT_EQ(becZero, sentFrames.back().bytes);
    becAvailable = false;
    for (unsigned i = 0; i < 4; i++) {
        tick(25000);
    }
    EXPECT_EQ(10u, sentFrames.size());
}

TEST_F(CrsfBecVoltageTest, BusyReceiverBufferDoesNotConsumeTheBecSlot)
{
    becConfigured = becAvailable = true;
    initCrsfTelemetry();
    tick(0);
    tick(25000);
    tick(25000);
    telemetryBufferEmpty = false;
    tick(25000);
    ASSERT_EQ(3u, sentFrames.size());
    telemetryBufferEmpty = true;
    tick(25000);
    ASSERT_EQ(4u, sentFrames.size());
    EXPECT_EQ(bec6120mV, sentFrames.back().bytes);
}

extern "C" {
bool becVoltageIsConfigured(void) { return becConfigured; }
bool becVoltageGet(uint16_t *voltage)
{
    if (!becConfigured || !becAvailable) {
        return false;
    }
    *voltage = becCentivolts;
    return true;
}

bool crsfRxIsActive(void) { return true; }
bool crsfRxIsTelemetryBufEmpty(void) { return telemetryBufferEmpty; }
void crsfRxSendTelemetryData(void) {}
void crsfRxWriteTelemetryData(const void *data, int length)
{
    const auto *bytes = static_cast<const uint8_t *>(data);
    sentFrames.push_back({nowUs, std::vector<uint8_t>(bytes, bytes + length)});
}

uint16_t getBatteryVoltage(void) { return 2478; }
uint16_t getBatteryAverageCellVoltage(void) { return 413; }
int16_t getAmperage(void) { return 1234; }
int32_t getMAhDrawn(void) { return 0x010203; }
uint8_t calculateBatteryPercentage(void) { return 64; }
float getEstimatedActualPosition(int) { return 0; }
float getEstimatedActualVelocity(int) { return 0; }
bool feature(uint32_t) { return false; }
bool IS_RC_MODE_ACTIVE(boxId_e) { return false; }
bool isWaypointMissionRTHActive(void) { return false; }
bool navigationRequiresAngleMode(void) { return false; }
bool sensors(uint32_t) { return false; }
float getAirspeedEstimate(void) { return 0; }
bool getSensorTemperature(uint8_t, int16_t *) { return false; }
bool handleMspFrame(uint8_t *, int) { return false; }
bool sendMspReply(uint8_t, mspResponseFnPtr) { return false; }

int tfp_sprintf(char *buffer, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    const int size = vsprintf(buffer, format, args);
    va_end(args);
    return size;
}
}
