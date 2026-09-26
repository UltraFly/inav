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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "config/parameter_group.h"
#include "drivers/adc.h"

typedef struct becVoltageConfig_s {
    uint16_t scale;      // Divider ratio multiplied by 100, as for vbat_scale.
    uint8_t adcChannel;  // ADC_CHN_NONE disables the meter; changes require reboot.
} becVoltageConfig_t;

PG_DECLARE(becVoltageConfig_t, becVoltageConfig);

void becVoltageConfigureAdc(drv_adc_config_t *adcConfig);
bool becVoltageIsConfigured(void);
void becVoltageUpdate(timeUs_t currentTimeUs);
// Returns a filtered voltage in centivolts. False leaves the output untouched; zero is a valid measurement.
bool becVoltageGet(uint16_t *voltage);
