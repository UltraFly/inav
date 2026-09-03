/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <stdint.h>

#include "common/axis.h"
#include "config/parameter_group.h"

typedef struct fwGyroAssistConfigStorage_s {
    // Percent of full mixer correction produced at 100 degrees/second.
    uint8_t gain[XYZ_AXIS_COUNT];

    // Maximum correction as a percentage of full mixer command.
    uint8_t correctionLimit[XYZ_AXIS_COUNT];

    // Stick-priority strength: 0 has no effect, 100 reaches zero gain at full
    // stick, and values above 100 reach zero gain before full stick.
    uint8_t stickPriority[XYZ_AXIS_COUNT];

    uint16_t stopReleaseTimeMs[XYZ_AXIS_COUNT];
    uint16_t stopLockTimeMs[XYZ_AXIS_COUNT];

    uint8_t gyroLpfHz;
    uint16_t transitionTimeMs;
} fwGyroAssistConfigStorage_t;

PG_DECLARE(fwGyroAssistConfigStorage_t, fwGyroAssistConfig);
