/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <string.h>

#include "platform.h"

#include "io/esc_srxl2_control.h"

// The 11 ms cadence and failsafe semantics follow sections 6.2 and 7.7 of the public
// Spektrum SRXL2 specification (Rev K): https://github.com/SpektrumRC/SRXL2

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

bool srxl2EscControlSchedulerUpdateThrottle(srxl2EscControlScheduler_t *scheduler,
    uint16_t pulseUs, int8_t rssi, uint16_t frameLosses, uint32_t nowUs)
{
    if (!scheduler || scheduler->throttleChannel >= SRXL2_ESC_MAX_CHANNELS) {
        return false;
    }

    scheduler->throttlePulseUs = pulseUs;
    scheduler->throttleUpdatedAtUs = nowUs;
    scheduler->rssi = rssi;
    scheduler->frameLosses = frameLosses;
    scheduler->throttleValid = true;
    return true;
}

bool srxl2EscControlSchedulerSetFailsafeThrottle(srxl2EscControlScheduler_t *scheduler,
    uint16_t pulseUs)
{
    if (!scheduler || scheduler->throttleChannel >= SRXL2_ESC_MAX_CHANNELS) {
        return false;
    }

    scheduler->failsafeThrottlePulseUs = pulseUs;
    scheduler->failsafeThrottleValid = true;
    return true;
}

bool srxl2EscControlSchedulerInputIsFresh(const srxl2EscControlScheduler_t *scheduler,
    uint32_t nowUs)
{
    return scheduler && scheduler->throttleValid &&
        (uint32_t)(nowUs - scheduler->throttleUpdatedAtUs) <= SRXL2_ESC_CONTROL_INPUT_TIMEOUT_US;
}

size_t srxl2EscControlSchedulerBuildFrame(srxl2EscControlScheduler_t *scheduler,
    const srxl2EscBus_t *bus, uint32_t nowUs, const srxl2EscControlSafety_t *safety,
    uint8_t *frame, size_t capacity)
{
    if (!scheduler || !frame || scheduler->throttleChannel >= SRXL2_ESC_MAX_CHANNELS ||
        !srxl2EscBusCanSendControl(bus) || !controlFrameIsDue(scheduler, nowUs)) {
        return 0;
    }

    const bool useFailsafe = !safety || safety->failsafe ||
        !srxl2EscControlSchedulerInputIsFresh(scheduler, nowUs);
    if (useFailsafe && !scheduler->failsafeThrottleValid) {
        return 0;
    }

    srxl2EscChannel_t throttle = {
        .channel = scheduler->throttleChannel,
        .pulseUs = useFailsafe ? scheduler->failsafeThrottlePulseUs : scheduler->throttlePulseUs,
    };

    if (useFailsafe || !safety->armed) {
        throttle.pulseUs = SRXL2_ESC_CONTROL_SAFE_THROTTLE_US;
    }

    const size_t length = srxl2EscBusBuildControlFrame(bus, frame, capacity,
        useFailsafe ? 0 : bus->escDeviceId, scheduler->rssi, scheduler->frameLosses,
        &throttle, 1, useFailsafe);
    if (length) {
        advanceControlFrameSchedule(scheduler, nowUs);
    }

    return length;
}
