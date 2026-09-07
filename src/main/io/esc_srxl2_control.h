/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "io/esc_srxl2_bus.h"

#define SRXL2_ESC_CONTROL_PERIOD_US                 11000U
#define SRXL2_ESC_CONTROL_INPUT_TIMEOUT_US          100000U
#define SRXL2_ESC_CONTROL_SAFE_THROTTLE_US            1000U
#define SRXL2_ESC_TELEMETRY_REQUEST_INTERVAL_FRAMES      30U
// A third-party Firma trace observed replies as late as 16 ms. Skip the next 11 ms slot after
// granting telemetry so the ESC owns an uninterrupted response window. Verify on Avian hardware.
#define SRXL2_ESC_TELEMETRY_RESPONSE_WINDOW_US        20000U

typedef struct srxl2EscControlSafety_s {
    bool armed;
    bool failsafe;
} srxl2EscControlSafety_t;

// The telemetry feature deliberately schedules only the configured throttle channel.
// Full receiver-channel forwarding and programming overrides belong to Avian TextGen.
typedef struct srxl2EscControlScheduler_s {
    uint16_t throttlePulseUs;
    uint16_t failsafeThrottlePulseUs;
    uint32_t throttleUpdatedAtUs;
    uint32_t nextFrameAtUs;
    int8_t rssi;
    uint16_t frameLosses;
    uint8_t throttleChannel;
    uint8_t telemetryRequestCountdown;
    uint32_t telemetryRequestCount;
    uint32_t telemetryGuardedSlotCount;
    bool throttleValid;
    bool failsafeThrottleValid;
} srxl2EscControlScheduler_t;

void srxl2EscControlSchedulerInit(srxl2EscControlScheduler_t *scheduler, uint32_t nowUs,
    uint8_t throttleChannel);
void srxl2EscControlSchedulerResetTiming(srxl2EscControlScheduler_t *scheduler, uint32_t nowUs);

bool srxl2EscControlSchedulerUpdateThrottle(srxl2EscControlScheduler_t *scheduler,
    uint16_t pulseUs, int8_t rssi, uint16_t frameLosses, uint32_t nowUs);
bool srxl2EscControlSchedulerSetFailsafeThrottle(srxl2EscControlScheduler_t *scheduler,
    uint16_t pulseUs);
bool srxl2EscControlSchedulerInputIsFresh(const srxl2EscControlScheduler_t *scheduler,
    uint32_t nowUs);

size_t srxl2EscControlSchedulerBuildFrame(srxl2EscControlScheduler_t *scheduler,
    const srxl2EscBus_t *bus, uint32_t nowUs, const srxl2EscControlSafety_t *safety,
    uint8_t *frame, size_t capacity);
