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

#define FW_SURFACE_DEFLECTION_AXIS_COUNT              3
#define FW_SURFACE_DEFLECTION_MAX_DELTA_TIME_SECONDS   0.1f

typedef enum {
    FW_SURFACE_DEFLECTION_PHASE_INACTIVE,
    FW_SURFACE_DEFLECTION_PHASE_ENTERING,
    FW_SURFACE_DEFLECTION_PHASE_ACTIVE,
    FW_SURFACE_DEFLECTION_PHASE_EXITING,
    FW_SURFACE_DEFLECTION_PHASE_INHIBITED,
} fwSurfaceDeflectionPhase_e;

typedef enum {
    FW_SURFACE_DEFLECTION_REASON_NONE,
    FW_SURFACE_DEFLECTION_REASON_NOT_REQUESTED,
    FW_SURFACE_DEFLECTION_REASON_NOT_FIXED_WING,
    FW_SURFACE_DEFLECTION_REASON_DISARMED,
    FW_SURFACE_DEFLECTION_REASON_FAILSAFE,
    FW_SURFACE_DEFLECTION_REASON_NAVIGATION,
    FW_SURFACE_DEFLECTION_REASON_LAUNCH,
    FW_SURFACE_DEFLECTION_REASON_AUTOTRIM,
    FW_SURFACE_DEFLECTION_REASON_MODE_NOT_ALLOWED,
    FW_SURFACE_DEFLECTION_REASON_GYRO_INVALID,
    FW_SURFACE_DEFLECTION_REASON_GYRO_STALE,
    FW_SURFACE_DEFLECTION_REASON_INVALID_TIMESTEP,
    FW_SURFACE_DEFLECTION_REASON_INVALID_INPUT,
    FW_SURFACE_DEFLECTION_REASON_INVALID_CONFIGURATION,
} fwSurfaceDeflectionReason_e;

typedef struct fwSurfaceDeflectionConfig_s {
    // Normalized mixer command per degree/second. Must be non-negative.
    float gyroGain[FW_SURFACE_DEFLECTION_AXIS_COUNT];

    // Maximum absolute gyro correction in normalized mixer units [0, 1].
    float correctionLimit[FW_SURFACE_DEFLECTION_AXIS_COUNT];

    // Zero bypasses the gyro low-pass filter.
    float gyroLpfHz;

    // Cross-fade time between the existing controller and this controller.
    uint16_t transitionTimeMs;

    // Maximum accepted age of the gyro sample.
    uint16_t gyroTimeoutMs;
} fwSurfaceDeflectionConfig_t;

typedef struct fwSurfaceDeflectionConditions_s {
    bool requested;
    bool fixedWing;
    bool armed;
    bool failsafe;
    bool navigationActive;
    bool launchActive;
    bool autotrimActive;
    bool modeAllowed;
    bool gyroValid;
} fwSurfaceDeflectionConditions_t;

typedef struct fwSurfaceDeflectionInput_s {
    // Pilot and fallback commands use the logical mixer range [-1, 1].
    float pilotCommand[FW_SURFACE_DEFLECTION_AXIS_COUNT];
    float fallbackCommand[FW_SURFACE_DEFLECTION_AXIS_COUNT];
    float gyroRateDps[FW_SURFACE_DEFLECTION_AXIS_COUNT];
    float deltaTimeSeconds;
    uint32_t nowMs;
    uint32_t gyroSampleTimeMs;
    fwSurfaceDeflectionConditions_t conditions;
} fwSurfaceDeflectionInput_t;

typedef struct fwSurfaceDeflectionAxisOutput_s {
    float pilotCommand;
    float filteredGyroRateDps;
    float correction;
    float surfaceCommand;
    float command;
    bool correctionLimited;
} fwSurfaceDeflectionAxisOutput_t;

typedef struct fwSurfaceDeflectionOutput_s {
    fwSurfaceDeflectionAxisOutput_t axis[FW_SURFACE_DEFLECTION_AXIS_COUNT];
    float blend;
    fwSurfaceDeflectionPhase_e phase;
    fwSurfaceDeflectionReason_e reason;
    bool usingSurfaceDeflection;
    bool hardInhibit;
} fwSurfaceDeflectionOutput_t;

typedef struct fwSurfaceDeflectionController_s {
    float filteredGyroRateDps[FW_SURFACE_DEFLECTION_AXIS_COUNT];
    float blend;
    fwSurfaceDeflectionPhase_e phase;
    bool filterInitialized;
} fwSurfaceDeflectionController_t;

void fwSurfaceDeflectionInit(fwSurfaceDeflectionController_t *controller);
void fwSurfaceDeflectionReset(fwSurfaceDeflectionController_t *controller);
bool fwSurfaceDeflectionConfigIsValid(const fwSurfaceDeflectionConfig_t *config);
fwSurfaceDeflectionOutput_t fwSurfaceDeflectionUpdate(fwSurfaceDeflectionController_t *controller,
    const fwSurfaceDeflectionConfig_t *config, const fwSurfaceDeflectionInput_t *input);
