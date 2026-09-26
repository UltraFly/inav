/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify this software
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * INAV is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 */

#include "gtest/gtest.h"

extern "C" {
#include <stdint.h>
#include <string.h>

#include "common/time.h"
#include "drivers/adc.h"
#include "drivers/time.h"
#include "fc/config.h"
#include "sensors/bec_voltage.h"

extern const becVoltageConfig_t pgResetTemplate_becVoltageConfig;
}

static bool adcAvailable;
static int adcAllocation;
static uint16_t adcSample;
static timeUs_t clockUs;

extern "C" {
bool adcIsFunctionAvailable(uint8_t function)
{
    EXPECT_EQ(ADC_BEC, function);
    return adcAvailable;
}

int adcGetFunctionChannelAllocation(uint8_t function)
{
    EXPECT_EQ(ADC_BEC, function);
    return adcAllocation;
}

uint16_t adcGetChannel(uint8_t function)
{
    EXPECT_EQ(ADC_BEC, function);
    return adcSample;
}

timeUs_t micros(void)
{
    return clockUs;
}
}

class BecVoltage : public ::testing::Test {
protected:
    drv_adc_config_t runtime;

    void SetUp() override
    {
        memset(&runtime, 0, sizeof(runtime));
        *becVoltageConfigMutable() = pgResetTemplate_becVoltageConfig;
        adcAvailable = false;
        adcAllocation = ADC_CHN_NONE;
        adcSample = 0;
        clockUs = 1000000;
        becVoltageConfigureAdc(&runtime);
    }

    void configure(uint8_t channel = ADC_CHN_2, uint16_t scale = 620)
    {
        becVoltageConfigMutable()->adcChannel = channel;
        becVoltageConfigMutable()->scale = scale;
        becVoltageConfigureAdc(&runtime);
        adcAllocation = runtime.adcFunctionChannel[ADC_BEC];
        adcAvailable = adcAllocation != ADC_CHN_NONE;
    }

    void update(uint16_t raw, timeUs_t elapsed = 20000)
    {
        clockUs += elapsed;
        adcSample = raw;
        becVoltageUpdate(clockUs);
    }

    uint16_t voltage()
    {
        uint16_t result = UINT16_MAX;
        EXPECT_TRUE(becVoltageGet(&result));
        return result;
    }

    void expectUnavailable()
    {
        uint16_t result = 12345;
        EXPECT_FALSE(becVoltageGet(&result));
        EXPECT_EQ(12345, result);
    }
};

TEST_F(BecVoltage, DefaultsAreDisabledWithoutChangingExistingAdcStorage)
{
    EXPECT_EQ(ADC_CHN_NONE, becVoltageConfig()->adcChannel);
    EXPECT_EQ(100, becVoltageConfig()->scale);
    EXPECT_FALSE(becVoltageIsConfigured());
    EXPECT_EQ(4u, sizeof(adcChannelConfig_t));
    EXPECT_EQ(4, ADC_FUNCTION_COUNT);
    EXPECT_EQ(5, ADC_RUNTIME_FUNCTION_COUNT);
    EXPECT_EQ(5u, sizeof(drv_adc_config_t));
    EXPECT_EQ(0, ADC_BATTERY);
    EXPECT_EQ(1, ADC_RSSI);
    EXPECT_EQ(2, ADC_CURRENT);
    EXPECT_EQ(3, ADC_AIRSPEED);
    EXPECT_EQ(ADC_CHN_NONE, runtime.adcFunctionChannel[ADC_BEC]);
    expectUnavailable();
}

TEST_F(BecVoltage, AllocatesIndependentChannelWithoutChangingOtherFunctions)
{
    runtime.adcFunctionChannel[ADC_BATTERY] = ADC_CHN_3;
    runtime.adcFunctionChannel[ADC_RSSI] = ADC_CHN_1;
    runtime.adcFunctionChannel[ADC_CURRENT] = ADC_CHN_4;
    runtime.adcFunctionChannel[ADC_AIRSPEED] = ADC_CHN_5;
    const drv_adc_config_t before = runtime;

    configure();

    EXPECT_TRUE(becVoltageIsConfigured());
    EXPECT_EQ(ADC_CHN_2, runtime.adcFunctionChannel[ADC_BEC]);
    for (int i = ADC_BATTERY; i <= ADC_AIRSPEED; i++) {
        EXPECT_EQ(before.adcFunctionChannel[i], runtime.adcFunctionChannel[i]);
    }
}

TEST_F(BecVoltage, ExistingFunctionRetainsChannelOnEveryAllocationConflict)
{
    for (int function = ADC_BATTERY; function <= ADC_AIRSPEED; function++) {
        memset(&runtime, 0, sizeof(runtime));
        runtime.adcFunctionChannel[function] = ADC_CHN_2;
        configure();
        EXPECT_EQ(ADC_CHN_2, runtime.adcFunctionChannel[function]);
        EXPECT_EQ(ADC_CHN_NONE, runtime.adcFunctionChannel[ADC_BEC]);
        update(1201);
        expectUnavailable();
    }
}

TEST_F(BecVoltage, DisabledInvalidAndZeroScaleConfigurationsCannotPublish)
{
    const uint8_t channels[] = { ADC_CHN_NONE, ADC_CHN_MAX + 1, UINT8_MAX };
    for (uint8_t channel : channels) {
        configure(channel);
        EXPECT_FALSE(becVoltageIsConfigured());
        EXPECT_EQ(ADC_CHN_NONE, runtime.adcFunctionChannel[ADC_BEC]);
        update(1201);
        expectUnavailable();
    }

    configure(ADC_CHN_2, 0);
    EXPECT_FALSE(becVoltageIsConfigured());
    EXPECT_EQ(ADC_CHN_NONE, runtime.adcFunctionChannel[ADC_BEC]);
    update(1201);
    expectUnavailable();
}

TEST_F(BecVoltage, ConfiguredMeterIsUnavailableUntilFirstSample)
{
    configure();
    expectUnavailable();
    update(1201);
    EXPECT_EQ(600, voltage());
}

TEST_F(BecVoltage, MissingHardwareDoesNotBecomeAZeroVoltReading)
{
    configure();
    adcAvailable = false;
    update(0);
    expectUnavailable();
}

TEST_F(BecVoltage, FirstZeroVoltSampleIsValid)
{
    configure();
    update(0);
    EXPECT_EQ(0, voltage());
}

TEST_F(BecVoltage, CalibrationUsesCentivoltsAndSeedsFirstSample)
{
    configure(ADC_CHN_2, 100);
    update(4095);
    EXPECT_EQ(330, voltage());

    configure(ADC_CHN_2, 620);
    update(4095);
    EXPECT_EQ(2046, voltage());

    configure(ADC_CHN_2, 630);
    update(4095);
    EXPECT_EQ(2079, voltage());
}

TEST_F(BecVoltage, SubsequentSamplesAreFilteredAndConvergeToZero)
{
    configure();
    update(1201);
    EXPECT_EQ(600, voltage());
    update(0);
    EXPECT_GT(voltage(), 0);
    EXPECT_LT(voltage(), 600);

    for (int i = 0; i < 100; i++) {
        update(0);
    }
    EXPECT_EQ(0, voltage());
}

TEST_F(BecVoltage, OutOfRangeCalibrationCannotWrapIntoPlausibleVoltage)
{
    configure(ADC_CHN_2, UINT16_MAX);
    update(4095);
    expectUnavailable();

    becVoltageConfigMutable()->scale = 620;
    update(1201);
    EXPECT_EQ(600, voltage());
}

TEST_F(BecVoltage, LostHardwareInvalidatesOldReadingAndRecoveryReseedsFilter)
{
    configure();
    update(1201);
    EXPECT_EQ(600, voltage());
    adcAvailable = false;
    expectUnavailable();
    update(0);
    expectUnavailable();

    adcAvailable = true;
    update(2048);
    EXPECT_EQ(1023, voltage());
}

TEST_F(BecVoltage, ChangedChannelRequiresMatchingRuntimeAllocation)
{
    configure();
    update(1201);
    becVoltageConfigMutable()->adcChannel = ADC_CHN_1;
    expectUnavailable();
    update(2048);
    expectUnavailable();
}

TEST_F(BecVoltage, StaleMeasurementDoesNotRemainAvailable)
{
    configure();
    update(1201);
    clockUs += 499999;
    EXPECT_EQ(600, voltage());
    clockUs += 2;
    expectUnavailable();

    update(2048);
    EXPECT_EQ(1023, voltage());
}

TEST_F(BecVoltage, FreshnessSurvivesMicrosecondClockWrap)
{
    configure();
    clockUs = static_cast<timeUs_t>(TIMEUS_MAX - 100000);
    update(1201, 0);
    clockUs += 200000;
    EXPECT_EQ(600, voltage());
    clockUs += 300001;
    expectUnavailable();
}

TEST_F(BecVoltage, ReconfiguringDoesNotPublishPreviousMeasurement)
{
    configure();
    update(1201);
    EXPECT_EQ(600, voltage());
    configure(ADC_CHN_1);
    expectUnavailable();
    update(0);
    EXPECT_EQ(0, voltage());
}

TEST_F(BecVoltage, DisablingConfiguredMeterImmediatelyInvalidatesOldReading)
{
    configure();
    update(1201);
    becVoltageConfigMutable()->adcChannel = ADC_CHN_NONE;
    expectUnavailable();

    configure();
    update(1201);
    becVoltageConfigMutable()->scale = 0;
    expectUnavailable();
}
