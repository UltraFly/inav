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
#include <stdint.h>

#define FW_GYRO_ASSIST_AXIS_COUNT              3
#define FW_GYRO_ASSIST_MAX_DELTA_TIME_SECONDS   0.1f
#define FW_GYRO_ASSIST_MAX_GAIN                 0.01f
#define FW_GYRO_ASSIST_MAX_CORRECTION            0.5f
#define FW_GYRO_ASSIST_MAX_LPF_HZ              100.0f
#define FW_GYRO_ASSIST_MIN_TRANSITION_TIME_MS   20
#define FW_GYRO_ASSIST_MAX_TRANSITION_TIME_MS   2000
#define FW_GYRO_ASSIST_MAX_PRIORITY              2.0f
#define FW_GYRO_ASSIST_MAX_STOP_TIME_MS          2000

typedef enum {
    FW_GYRO_ASSIST_PHASE_INACTIVE,
    FW_GYRO_ASSIST_PHASE_ENTERING,
    FW_GYRO_ASSIST_PHASE_ACTIVE,
    FW_GYRO_ASSIST_PHASE_EXITING,
    FW_GYRO_ASSIST_PHASE_INHIBITED,
} fwGyroAssistPhase_e;

typedef enum {
    FW_GYRO_ASSIST_REASON_NONE,
    FW_GYRO_ASSIST_REASON_NOT_REQUESTED,
    FW_GYRO_ASSIST_REASON_NOT_FIXED_WING,
    FW_GYRO_ASSIST_REASON_DISARMED,
    FW_GYRO_ASSIST_REASON_FAILSAFE,
    FW_GYRO_ASSIST_REASON_NAVIGATION,
    FW_GYRO_ASSIST_REASON_LAUNCH,
    FW_GYRO_ASSIST_REASON_AUTOTRIM,
    FW_GYRO_ASSIST_REASON_MODE_NOT_ALLOWED,
    FW_GYRO_ASSIST_REASON_GYRO_INVALID,
    FW_GYRO_ASSIST_REASON_GYRO_STALE,
    FW_GYRO_ASSIST_REASON_INVALID_TIMESTEP,
    FW_GYRO_ASSIST_REASON_INVALID_INPUT,
    FW_GYRO_ASSIST_REASON_INVALID_CONFIGURATION,
} fwGyroAssistReason_e;

typedef struct fwGyroAssistConfig_s {
    // Normalized mixer command per degree/second. Must be non-negative.
    float gyroGain[FW_GYRO_ASSIST_AXIS_COUNT];

    // Maximum absolute gyro correction in normalized mixer units [0, 1].
    float correctionLimit[FW_GYRO_ASSIST_AXIS_COUNT];

    // Stick-priority strength [0, 2]. Zero leaves gain independent of stick.
    float stickPriority[FW_GYRO_ASSIST_AXIS_COUNT];

    // PT1 time constants for reducing gain on stick movement (release) and
    // restoring it after recentering (lock). Zero applies the target at once.
    uint16_t stopReleaseTimeMs[FW_GYRO_ASSIST_AXIS_COUNT];
    uint16_t stopLockTimeMs[FW_GYRO_ASSIST_AXIS_COUNT];

    // Zero bypasses the gyro low-pass filter.
    float gyroLpfHz;

    // Cross-fade time between the existing controller and this controller.
    uint16_t transitionTimeMs;

    // Maximum accepted age of the gyro sample.
    uint16_t gyroTimeoutMs;
} fwGyroAssistConfig_t;

typedef struct fwGyroAssistConditions_s {
    bool requested;
    bool fixedWing;
    bool armed;
    bool failsafe;
    bool navigationActive;
    bool launchActive;
    bool autotrimActive;
    bool modeAllowed;
    bool gyroValid;
} fwGyroAssistConditions_t;

typedef struct fwGyroAssistInput_s {
    // Pilot and fallback commands use the logical mixer range [-1, 1].
    float pilotCommand[FW_GYRO_ASSIST_AXIS_COUNT];
    float fallbackCommand[FW_GYRO_ASSIST_AXIS_COUNT];
    float gyroRateDps[FW_GYRO_ASSIST_AXIS_COUNT];
    float deltaTimeSeconds;
    uint32_t nowMs;
    uint32_t gyroSampleTimeMs;
    fwGyroAssistConditions_t conditions;
} fwGyroAssistInput_t;

typedef struct fwGyroAssistAxisOutput_s {
    float pilotCommand;
    float filteredGyroRateDps;
    float priorityGainScale;
    float effectiveGyroGain;
    float correction;
    float surfaceCommand;
    float command;
    bool correctionLimited;
} fwGyroAssistAxisOutput_t;

typedef struct fwGyroAssistOutput_s {
    fwGyroAssistAxisOutput_t axis[FW_GYRO_ASSIST_AXIS_COUNT];
    float blend;
    fwGyroAssistPhase_e phase;
    fwGyroAssistReason_e reason;
    bool usingGyroAssist;
    bool hardInhibit;
} fwGyroAssistOutput_t;

typedef struct fwGyroAssistController_s {
    float filteredGyroRateDps[FW_GYRO_ASSIST_AXIS_COUNT];
    float priorityGainScale[FW_GYRO_ASSIST_AXIS_COUNT];
    float blend;
    fwGyroAssistPhase_e phase;
    bool filterInitialized;
} fwGyroAssistController_t;

void fwGyroAssistInit(fwGyroAssistController_t *controller);
void fwGyroAssistReset(fwGyroAssistController_t *controller);
bool fwGyroAssistConfigIsValid(const fwGyroAssistConfig_t *config);
fwGyroAssistOutput_t fwGyroAssistUpdate(fwGyroAssistController_t *controller,
    const fwGyroAssistConfig_t *config, const fwGyroAssistInput_t *input);
