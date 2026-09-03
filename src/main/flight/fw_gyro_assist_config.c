/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "platform.h"

#ifdef USE_FW_GYRO_ASSIST

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "flight/fw_gyro_assist_config.h"

PG_REGISTER_WITH_RESET_TEMPLATE(fwGyroAssistConfigStorage_t, fwGyroAssistConfig,
    PG_FW_GYRO_ASSIST_CONFIG, 1);

// Zero gain makes enabling the AUX mode harmless until the pilot has
// deliberately configured and bench-checked each correction axis.
PG_RESET_TEMPLATE(fwGyroAssistConfigStorage_t, fwGyroAssistConfig,
    .gain = { 0, 0, 0 },
    .correctionLimit = { 20, 20, 20 },
    .stickPriority = { 100, 100, 100 },
    .stopReleaseTimeMs = { 50, 50, 50 },
    .stopLockTimeMs = { 250, 250, 250 },
    .gyroLpfHz = 20,
    .transitionTimeMs = 250
);

#endif
