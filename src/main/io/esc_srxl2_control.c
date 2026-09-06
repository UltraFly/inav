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

#include <string.h>

#include "platform.h"

#include "io/esc_srxl2_control.h"

// The 11 ms cadence and failsafe semantics follow sections 6.2 and 7.7 of the public
// Spektrum SRXL2 specification (Rev K): https://github.com/SpektrumRC/SRXL2

static uint8_t representableChannelCount(size_t channelCount)
{
    return channelCount > SRXL2_ESC_MAX_CHANNELS ? SRXL2_ESC_MAX_CHANNELS : channelCount;
}

static bool controlFrameIsDue(const srxl2EscControlScheduler_t *scheduler, uint32_t nowUs)
{
    return (int32_t)(nowUs - scheduler->nextFrameAtUs) >= 0;
}

static void advanceControlFrameSchedule(srxl2EscControlScheduler_t *scheduler, uint32_t nowUs)
{
    do {
        scheduler->nextFrameAtUs += SRXL2_ESC_CONTROL_PERIOD_US;
    } while (controlFrameIsDue(scheduler, nowUs));
}

static bool programmingOverridesAreValid(const srxl2EscControlScheduler_t *scheduler,
    const srxl2EscChannel_t *overrides, size_t overrideCount)
{
    if (overrideCount && !overrides) {
        return false;
    }

    uint32_t overrideMask = 0;
    for (size_t i = 0; i < overrideCount; i++) {
        if (overrides[i].channel >= scheduler->channelCount) {
            return false;
        }

        const uint32_t channelBit = UINT32_C(1) << overrides[i].channel;
        if (overrideMask & channelBit) {
            return false;
        }
        overrideMask |= channelBit;
    }

    return true;
}

void srxl2EscControlSchedulerInit(srxl2EscControlScheduler_t *scheduler, uint32_t nowUs,
    uint8_t throttleChannel)
{
    if (!scheduler) {
        return;
    }

    memset(scheduler, 0, sizeof(*scheduler));
    scheduler->nextFrameAtUs = nowUs;
    scheduler->throttleChannel = throttleChannel;
}

void srxl2EscControlSchedulerResetTiming(srxl2EscControlScheduler_t *scheduler, uint32_t nowUs)
{
    if (scheduler) {
        scheduler->nextFrameAtUs = nowUs;
    }
}

bool srxl2EscControlSchedulerUpdateChannels(srxl2EscControlScheduler_t *scheduler,
    const uint16_t *pulseUs, size_t channelCount, int8_t rssi, uint16_t frameLosses, uint32_t nowUs)
{
    if (!scheduler || !pulseUs || channelCount == 0) {
        return false;
    }

    const uint8_t count = representableChannelCount(channelCount);
    if (scheduler->throttleChannel >= count) {
        return false;
    }

    memcpy(scheduler->channelPulseUs, pulseUs, count * sizeof(pulseUs[0]));
    scheduler->channelCount = count;
    scheduler->channelsUpdatedAtUs = nowUs;
    scheduler->rssi = rssi;
    scheduler->frameLosses = frameLosses;
    scheduler->channelsValid = true;
    return true;
}

bool srxl2EscControlSchedulerSetFailsafe(srxl2EscControlScheduler_t *scheduler,
    const uint16_t *pulseUs, size_t channelCount)
{
    if (!scheduler || !pulseUs || channelCount == 0) {
        return false;
    }

    const uint8_t count = representableChannelCount(channelCount);
    if (scheduler->throttleChannel >= count) {
        return false;
    }

    memcpy(scheduler->failsafePulseUs, pulseUs, count * sizeof(pulseUs[0]));
    scheduler->failsafeChannelCount = count;
    scheduler->failsafeValid = true;
    return true;
}

bool srxl2EscControlSchedulerInputIsFresh(const srxl2EscControlScheduler_t *scheduler, uint32_t nowUs)
{
    return scheduler && scheduler->channelsValid &&
        (uint32_t)(nowUs - scheduler->channelsUpdatedAtUs) <= SRXL2_ESC_CONTROL_INPUT_TIMEOUT_US;
}

bool srxl2EscControlSchedulerFailsafeIsComplete(const srxl2EscControlScheduler_t *scheduler)
{
    if (!scheduler || !scheduler->failsafeValid) {
        return false;
    }

    return !scheduler->channelsValid || scheduler->failsafeChannelCount >= scheduler->channelCount;
}

size_t srxl2EscControlSchedulerBuildFrame(srxl2EscControlScheduler_t *scheduler,
    const srxl2EscBus_t *bus, uint32_t nowUs, const srxl2EscControlSafety_t *safety,
    const srxl2EscChannel_t *programmingOverrides, size_t programmingOverrideCount,
    uint8_t *frame, size_t capacity)
{
    if (!scheduler || !frame || !srxl2EscBusCanSendControl(bus) ||
        !controlFrameIsDue(scheduler, nowUs)) {
        return 0;
    }

    bool useFailsafe = !safety || safety->failsafe ||
        !srxl2EscControlSchedulerInputIsFresh(scheduler, nowUs) ||
        (safety->programming && safety->armed);

    if (!useFailsafe && safety->programming &&
        !programmingOverridesAreValid(scheduler, programmingOverrides, programmingOverrideCount)) {
        useFailsafe = true;
    }

    if (useFailsafe && !srxl2EscControlSchedulerFailsafeIsComplete(scheduler)) {
        return 0;
    }

    const uint8_t channelCount = useFailsafe ? scheduler->failsafeChannelCount : scheduler->channelCount;
    srxl2EscChannel_t channels[SRXL2_ESC_MAX_CHANNELS];
    for (uint8_t channel = 0; channel < channelCount; channel++) {
        channels[channel].channel = channel;
        channels[channel].pulseUs = useFailsafe ?
            scheduler->failsafePulseUs[channel] : scheduler->channelPulseUs[channel];
    }

    if (!useFailsafe && safety->programming) {
        for (size_t i = 0; i < programmingOverrideCount; i++) {
            channels[programmingOverrides[i].channel].pulseUs = programmingOverrides[i].pulseUs;
        }
    }

    if (useFailsafe || !safety->armed || safety->programming) {
        channels[scheduler->throttleChannel].pulseUs = SRXL2_ESC_CONTROL_SAFE_THROTTLE_US;
    }

    const size_t length = srxl2EscBusBuildControlFrame(bus, frame, capacity,
        useFailsafe ? 0 : bus->escDeviceId, scheduler->rssi, scheduler->frameLosses,
        channels, channelCount, useFailsafe);
    if (length) {
        advanceControlFrameSchedule(scheduler, nowUs);
    }

    return length;
}
