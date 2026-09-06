/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "io/esc_srxl2_bus.h"

#define SRXL2_ESC_CONTROL_PERIOD_US                 11000U
#define SRXL2_ESC_CONTROL_INPUT_TIMEOUT_US          100000U
#define SRXL2_ESC_CONTROL_SAFE_THROTTLE_US            1000U

typedef struct srxl2EscControlSafety_s {
    bool armed;
    bool failsafe;
    bool programming;
} srxl2EscControlSafety_t;

typedef struct srxl2EscControlScheduler_s {
    uint16_t channelPulseUs[SRXL2_ESC_MAX_CHANNELS];
    uint16_t failsafePulseUs[SRXL2_ESC_MAX_CHANNELS];
    uint32_t channelsUpdatedAtUs;
    uint32_t nextFrameAtUs;
    int8_t rssi;
    uint16_t frameLosses;
    uint8_t channelCount;
    uint8_t failsafeChannelCount;
    uint8_t throttleChannel;
    bool channelsValid;
    bool failsafeValid;
} srxl2EscControlScheduler_t;

void srxl2EscControlSchedulerInit(srxl2EscControlScheduler_t *scheduler, uint32_t nowUs,
    uint8_t throttleChannel);
void srxl2EscControlSchedulerResetTiming(srxl2EscControlScheduler_t *scheduler, uint32_t nowUs);

// INAV receiver channels are sequential and zero-based. Channels above index 31 are not representable
// by SRXL2 and are deliberately omitted while the original channel numbering is preserved.
bool srxl2EscControlSchedulerUpdateChannels(srxl2EscControlScheduler_t *scheduler,
    const uint16_t *pulseUs, size_t channelCount, int8_t rssi, uint16_t frameLosses, uint32_t nowUs);
bool srxl2EscControlSchedulerSetFailsafe(srxl2EscControlScheduler_t *scheduler,
    const uint16_t *pulseUs, size_t channelCount);

bool srxl2EscControlSchedulerInputIsFresh(const srxl2EscControlScheduler_t *scheduler, uint32_t nowUs);
bool srxl2EscControlSchedulerFailsafeIsComplete(const srxl2EscControlScheduler_t *scheduler);

// Programming overrides are applied only while programming is requested and disarmed. The throttle
// channel is always forced to its safe value during programming, regardless of the supplied overrides.
size_t srxl2EscControlSchedulerBuildFrame(srxl2EscControlScheduler_t *scheduler,
    const srxl2EscBus_t *bus, uint32_t nowUs, const srxl2EscControlSafety_t *safety,
    const srxl2EscChannel_t *programmingOverrides, size_t programmingOverrideCount,
    uint8_t *frame, size_t capacity);
