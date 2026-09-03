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

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "platform.h"

#include "flight/fw_surface_deflection.h"

#define TWO_PI_FLOAT 6.28318530717958647692f

static float clampUnit(float value)
{
    if (value < -1.0f) {
        return -1.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

static float clampRange(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static bool valueIsFinite(float value)
{
    return !isnan(value) && !isinf(value);
}

void fwSurfaceDeflectionInit(fwSurfaceDeflectionController_t *controller)
{
    fwSurfaceDeflectionReset(controller);
}

void fwSurfaceDeflectionReset(fwSurfaceDeflectionController_t *controller)
{
    if (controller) {
        memset(controller, 0, sizeof(*controller));
        controller->phase = FW_SURFACE_DEFLECTION_PHASE_INACTIVE;
    }
}

bool fwSurfaceDeflectionConfigIsValid(const fwSurfaceDeflectionConfig_t *config)
{
    if (!config || !valueIsFinite(config->gyroLpfHz) || config->gyroLpfHz < 0.0f ||
        config->transitionTimeMs == 0 || config->gyroTimeoutMs == 0) {
        return false;
    }

    for (unsigned axis = 0; axis < FW_SURFACE_DEFLECTION_AXIS_COUNT; axis++) {
        if (!valueIsFinite(config->gyroGain[axis]) || config->gyroGain[axis] < 0.0f ||
            !valueIsFinite(config->correctionLimit[axis]) || config->correctionLimit[axis] < 0.0f ||
            config->correctionLimit[axis] > 1.0f) {
            return false;
        }
    }

    return true;
}

static bool inputValuesAreValid(const fwSurfaceDeflectionInput_t *input)
{
    if (!input || !valueIsFinite(input->deltaTimeSeconds) || input->deltaTimeSeconds <= 0.0f ||
        input->deltaTimeSeconds > FW_SURFACE_DEFLECTION_MAX_DELTA_TIME_SECONDS) {
        return false;
    }

    for (unsigned axis = 0; axis < FW_SURFACE_DEFLECTION_AXIS_COUNT; axis++) {
        if (!valueIsFinite(input->pilotCommand[axis]) || !valueIsFinite(input->fallbackCommand[axis]) ||
            !valueIsFinite(input->gyroRateDps[axis])) {
            return false;
        }
    }

    return true;
}

static fwSurfaceDeflectionOutput_t makeFallbackOutput(const fwSurfaceDeflectionInput_t *input,
    fwSurfaceDeflectionPhase_e phase, fwSurfaceDeflectionReason_e reason, bool hardInhibit)
{
    fwSurfaceDeflectionOutput_t output = {
        .phase = phase,
        .reason = reason,
        .hardInhibit = hardInhibit,
    };

    if (!input) {
        return output;
    }

    for (unsigned axis = 0; axis < FW_SURFACE_DEFLECTION_AXIS_COUNT; axis++) {
        const float pilot = valueIsFinite(input->pilotCommand[axis]) ? clampUnit(input->pilotCommand[axis]) : 0.0f;
        const float fallback = valueIsFinite(input->fallbackCommand[axis]) ?
            clampUnit(input->fallbackCommand[axis]) : 0.0f;
        output.axis[axis].pilotCommand = pilot;
        output.axis[axis].surfaceCommand = pilot;
        output.axis[axis].command = fallback;
    }

    return output;
}

static fwSurfaceDeflectionReason_e getHardInhibitReason(const fwSurfaceDeflectionConfig_t *config,
    const fwSurfaceDeflectionInput_t *input)
{
    if (!input) {
        return FW_SURFACE_DEFLECTION_REASON_INVALID_INPUT;
    }
    if (!input->conditions.fixedWing) {
        return FW_SURFACE_DEFLECTION_REASON_NOT_FIXED_WING;
    }
    if (!input->conditions.armed) {
        return FW_SURFACE_DEFLECTION_REASON_DISARMED;
    }
    if (input->conditions.failsafe) {
        return FW_SURFACE_DEFLECTION_REASON_FAILSAFE;
    }
    if (!fwSurfaceDeflectionConfigIsValid(config)) {
        return FW_SURFACE_DEFLECTION_REASON_INVALID_CONFIGURATION;
    }
    if (!input->conditions.gyroValid) {
        return FW_SURFACE_DEFLECTION_REASON_GYRO_INVALID;
    }
    if ((uint32_t)(input->nowMs - input->gyroSampleTimeMs) > config->gyroTimeoutMs) {
        return FW_SURFACE_DEFLECTION_REASON_GYRO_STALE;
    }
    if (!valueIsFinite(input->deltaTimeSeconds) || input->deltaTimeSeconds <= 0.0f ||
        input->deltaTimeSeconds > FW_SURFACE_DEFLECTION_MAX_DELTA_TIME_SECONDS) {
        return FW_SURFACE_DEFLECTION_REASON_INVALID_TIMESTEP;
    }
    if (!inputValuesAreValid(input)) {
        return FW_SURFACE_DEFLECTION_REASON_INVALID_INPUT;
    }
    return FW_SURFACE_DEFLECTION_REASON_NONE;
}

static fwSurfaceDeflectionReason_e getRequestedStateReason(const fwSurfaceDeflectionInput_t *input)
{
    if (!input->conditions.requested) {
        return FW_SURFACE_DEFLECTION_REASON_NOT_REQUESTED;
    }
    if (input->conditions.navigationActive) {
        return FW_SURFACE_DEFLECTION_REASON_NAVIGATION;
    }
    if (input->conditions.launchActive) {
        return FW_SURFACE_DEFLECTION_REASON_LAUNCH;
    }
    if (input->conditions.autotrimActive) {
        return FW_SURFACE_DEFLECTION_REASON_AUTOTRIM;
    }
    if (!input->conditions.modeAllowed) {
        return FW_SURFACE_DEFLECTION_REASON_MODE_NOT_ALLOWED;
    }
    return FW_SURFACE_DEFLECTION_REASON_NONE;
}

static float updateBlend(fwSurfaceDeflectionController_t *controller, const fwSurfaceDeflectionConfig_t *config,
    float deltaTimeSeconds, bool targetActive)
{
    const float blendStep = deltaTimeSeconds * 1000.0f / config->transitionTimeMs;

    if (targetActive) {
        if (controller->phase == FW_SURFACE_DEFLECTION_PHASE_INACTIVE ||
            controller->phase == FW_SURFACE_DEFLECTION_PHASE_INHIBITED) {
            controller->blend = 0.0f;
            controller->phase = FW_SURFACE_DEFLECTION_PHASE_ENTERING;
        } else if (controller->phase == FW_SURFACE_DEFLECTION_PHASE_EXITING) {
            controller->phase = FW_SURFACE_DEFLECTION_PHASE_ENTERING;
        } else if (controller->phase == FW_SURFACE_DEFLECTION_PHASE_ENTERING) {
            controller->blend = clampRange(controller->blend + blendStep, 0.0f, 1.0f);
            if (controller->blend >= 1.0f) {
                controller->phase = FW_SURFACE_DEFLECTION_PHASE_ACTIVE;
            }
        }
    } else {
        if (controller->phase == FW_SURFACE_DEFLECTION_PHASE_ACTIVE ||
            controller->phase == FW_SURFACE_DEFLECTION_PHASE_ENTERING) {
            controller->phase = FW_SURFACE_DEFLECTION_PHASE_EXITING;
        } else if (controller->phase == FW_SURFACE_DEFLECTION_PHASE_EXITING) {
            controller->blend = clampRange(controller->blend - blendStep, 0.0f, 1.0f);
            if (controller->blend <= 0.0f) {
                controller->phase = FW_SURFACE_DEFLECTION_PHASE_INACTIVE;
                controller->filterInitialized = false;
            }
        } else {
            controller->phase = FW_SURFACE_DEFLECTION_PHASE_INACTIVE;
            controller->blend = 0.0f;
            controller->filterInitialized = false;
        }
    }

    return controller->blend;
}

static void updateGyroFilter(fwSurfaceDeflectionController_t *controller,
    const fwSurfaceDeflectionConfig_t *config, const fwSurfaceDeflectionInput_t *input)
{
    if (!controller->filterInitialized) {
        memcpy(controller->filteredGyroRateDps, input->gyroRateDps, sizeof(controller->filteredGyroRateDps));
        controller->filterInitialized = true;
        return;
    }

    if (config->gyroLpfHz == 0.0f) {
        memcpy(controller->filteredGyroRateDps, input->gyroRateDps, sizeof(controller->filteredGyroRateDps));
        return;
    }

    const float filterTime = input->deltaTimeSeconds * TWO_PI_FLOAT * config->gyroLpfHz;
    const float alpha = valueIsFinite(filterTime) ? filterTime / (1.0f + filterTime) : 1.0f;
    for (unsigned axis = 0; axis < FW_SURFACE_DEFLECTION_AXIS_COUNT; axis++) {
        controller->filteredGyroRateDps[axis] +=
            alpha * (input->gyroRateDps[axis] - controller->filteredGyroRateDps[axis]);
    }
}

fwSurfaceDeflectionOutput_t fwSurfaceDeflectionUpdate(fwSurfaceDeflectionController_t *controller,
    const fwSurfaceDeflectionConfig_t *config, const fwSurfaceDeflectionInput_t *input)
{
    if (!controller) {
        return makeFallbackOutput(input, FW_SURFACE_DEFLECTION_PHASE_INHIBITED,
            FW_SURFACE_DEFLECTION_REASON_INVALID_INPUT, true);
    }

    // A dormant opt-in feature must not turn an unrelated gyro/configuration
    // problem into a flight-mode fault. Once a transition has started, all
    // validation remains active until the existing controller has full control.
    if (input && !input->conditions.requested && controller->blend <= 0.0f) {
        fwSurfaceDeflectionReset(controller);
        return makeFallbackOutput(input, controller->phase,
            FW_SURFACE_DEFLECTION_REASON_NOT_REQUESTED, false);
    }

    const fwSurfaceDeflectionReason_e hardInhibitReason = getHardInhibitReason(config, input);
    if (hardInhibitReason != FW_SURFACE_DEFLECTION_REASON_NONE) {
        fwSurfaceDeflectionReset(controller);
        controller->phase = FW_SURFACE_DEFLECTION_PHASE_INHIBITED;
        return makeFallbackOutput(input, controller->phase, hardInhibitReason, true);
    }

    const fwSurfaceDeflectionReason_e requestedStateReason = getRequestedStateReason(input);
    const bool targetActive = requestedStateReason == FW_SURFACE_DEFLECTION_REASON_NONE;

    updateGyroFilter(controller, config, input);
    const float blend = updateBlend(controller, config, input->deltaTimeSeconds, targetActive);

    fwSurfaceDeflectionOutput_t output = makeFallbackOutput(input, controller->phase,
        requestedStateReason, false);
    output.blend = blend;
    output.usingSurfaceDeflection = targetActive || blend > 0.0f;

    for (unsigned axis = 0; axis < FW_SURFACE_DEFLECTION_AXIS_COUNT; axis++) {
        fwSurfaceDeflectionAxisOutput_t *axisOutput = &output.axis[axis];
        axisOutput->filteredGyroRateDps = controller->filteredGyroRateDps[axis];

        const float rawCorrection = -controller->filteredGyroRateDps[axis] * config->gyroGain[axis];
        const float correctionLimit = config->correctionLimit[axis];
        const float limitedCorrection = clampRange(rawCorrection, -correctionLimit, correctionLimit);

        // Reserve the remaining command range for the pilot. At full stick the
        // disturbance correction is zero, so stabilization cannot reduce throw.
        const float pilotHeadroom = 1.0f - fabsf(axisOutput->pilotCommand);
        axisOutput->correction = clampRange(limitedCorrection, -pilotHeadroom, pilotHeadroom);
        axisOutput->correctionLimited = axisOutput->correction != rawCorrection;
        axisOutput->surfaceCommand = clampUnit(axisOutput->pilotCommand + axisOutput->correction);

        const float fallback = axisOutput->command;
        axisOutput->command = clampUnit(fallback + blend * (axisOutput->surfaceCommand - fallback));
    }

    return output;
}
