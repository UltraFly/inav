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
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_ADC

#include "common/filter.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "drivers/time.h"
#include "fc/settings.h"
#include "sensors/bec_voltage.h"

#ifndef VBEC_SCALE_DEFAULT
#define VBEC_SCALE_DEFAULT 100
#endif

#define BEC_VOLTAGE_TIMEOUT_US 500000
#define BEC_VOLTAGE_LPF_HZ 1.0f

PG_REGISTER_WITH_RESET_TEMPLATE(becVoltageConfig_t, becVoltageConfig, PG_BEC_VOLTAGE_CONFIG, 0);

PG_RESET_TEMPLATE(becVoltageConfig_t, becVoltageConfig,
    .scale = VBEC_SCALE_DEFAULT,
    .adcChannel = SETTING_VBEC_ADC_CHANNEL_DEFAULT,
);

static pt1Filter_t voltageFilter;
static timeUs_t lastSampleTimeUs;
static bool sampleValid;

bool becVoltageIsConfigured(void)
{
    return becVoltageConfig()->adcChannel >= ADC_CHN_1
        && becVoltageConfig()->adcChannel <= ADC_CHN_MAX
        && becVoltageConfig()->scale > 0;
}

void becVoltageConfigureAdc(drv_adc_config_t *adcConfig)
{
    sampleValid = false;
    adcConfig->adcFunctionChannel[ADC_BEC] = ADC_CHN_NONE;
    if (!becVoltageIsConfigured()) {
        return;
    }

    // Preserve the existing ADC users when a conflicting VBEC channel is selected.
    const uint8_t channel = becVoltageConfig()->adcChannel;
    for (unsigned i = 0; i < ADC_BEC; i++) {
        if (adcConfig->adcFunctionChannel[i] == channel) {
            return;
        }
    }
    adcConfig->adcFunctionChannel[ADC_BEC] = channel;
}

static bool becVoltageInputAvailable(void)
{
    return becVoltageIsConfigured() && adcIsFunctionAvailable(ADC_BEC)
        && adcGetFunctionChannelAllocation(ADC_BEC) == becVoltageConfig()->adcChannel;
}

void becVoltageUpdate(timeUs_t currentTimeUs)
{
    if (!becVoltageInputAvailable()) {
        sampleValid = false;
        return;
    }

    // Use the same nominal 3.3 V reference and scale convention as VBAT.
    const uint32_t sample = (uint64_t)adcGetChannel(ADC_BEC) * becVoltageConfig()->scale * 3300
        / (4095 * 1000);
    if (sample > UINT16_MAX) {
        sampleValid = false;
        return;
    }

    const timeDelta_t elapsed = cmpTimeUs(currentTimeUs, lastSampleTimeUs);
    if (!sampleValid || elapsed < 0 || elapsed > BEC_VOLTAGE_TIMEOUT_US) {
        pt1FilterInit(&voltageFilter, BEC_VOLTAGE_LPF_HZ, 0.02f);
        pt1FilterReset(&voltageFilter, sample);
    } else if (elapsed > 0) {
        pt1FilterApply3(&voltageFilter, sample, elapsed * 1e-6f);
    }
    lastSampleTimeUs = currentTimeUs;
    sampleValid = true;
}

bool becVoltageGet(uint16_t *voltage)
{
    const timeDelta_t age = cmpTimeUs(micros(), lastSampleTimeUs);
    if (!sampleValid || !becVoltageInputAvailable() || age < 0 || age > BEC_VOLTAGE_TIMEOUT_US) {
        return false;
    }

    *voltage = (uint16_t)(voltageFilter.state + 0.5f);
    return true;
}

#endif
