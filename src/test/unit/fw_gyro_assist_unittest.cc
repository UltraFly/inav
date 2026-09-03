#include "gtest/gtest.h"

#include <cmath>
#include <cstdint>
#include <limits>

extern "C" {
#include "flight/fw_gyro_assist.h"
}

namespace {

constexpr unsigned ROLL = 0;
constexpr unsigned PITCH = 1;
constexpr unsigned YAW = 2;

fwGyroAssistConfig_t defaultConfig()
{
    fwGyroAssistConfig_t config = {};
    for (unsigned axis = 0; axis < FW_GYRO_ASSIST_AXIS_COUNT; axis++) {
        config.gyroGain[axis] = 0.01f;
        config.correctionLimit[axis] = 0.25f;
        config.stickPriority[axis] = 0.0f;
    }
    config.gyroLpfHz = 0.0f;
    config.transitionTimeMs = 100;
    config.gyroTimeoutMs = 20;
    return config;
}

fwGyroAssistInput_t defaultInput()
{
    fwGyroAssistInput_t input = {};
    input.deltaTimeSeconds = 0.01f;
    input.nowMs = 1000;
    input.gyroSampleTimeMs = 1000;
    input.conditions.requested = true;
    input.conditions.fixedWing = true;
    input.conditions.armed = true;
    input.conditions.modeAllowed = true;
    input.conditions.gyroValid = true;
    return input;
}

fwGyroAssistOutput_t update(fwGyroAssistController_t *controller,
    const fwGyroAssistConfig_t *config, fwGyroAssistInput_t *input)
{
    const fwGyroAssistOutput_t output = fwGyroAssistUpdate(controller, config, input);
    input->nowMs += 10;
    input->gyroSampleTimeMs = input->nowMs;
    return output;
}

fwGyroAssistOutput_t advanceToActive(fwGyroAssistController_t *controller,
    const fwGyroAssistConfig_t *config, fwGyroAssistInput_t *input)
{
    fwGyroAssistOutput_t output = update(controller, config, input);
    for (unsigned i = 0; i < 10; i++) {
        output = update(controller, config, input);
    }
    return output;
}

} // namespace

TEST(FwGyroAssist, AcceptsBoundedConfiguration)
{
    fwGyroAssistConfig_t config = defaultConfig();
    EXPECT_TRUE(fwGyroAssistConfigIsValid(&config));

    config.correctionLimit[YAW] = 0.51f;
    EXPECT_FALSE(fwGyroAssistConfigIsValid(&config));
    config = defaultConfig();
    config.stickPriority[ROLL] = 2.01f;
    EXPECT_FALSE(fwGyroAssistConfigIsValid(&config));
    config = defaultConfig();
    config.stopLockTimeMs[ROLL] = FW_GYRO_ASSIST_MAX_STOP_TIME_MS + 1;
    EXPECT_FALSE(fwGyroAssistConfigIsValid(&config));
    config = defaultConfig();
    config.stopReleaseTimeMs[ROLL] = FW_GYRO_ASSIST_MAX_STOP_TIME_MS + 1;
    EXPECT_FALSE(fwGyroAssistConfigIsValid(&config));
    config = defaultConfig();
    config.gyroGain[ROLL] = -0.01f;
    EXPECT_FALSE(fwGyroAssistConfigIsValid(&config));
    config = defaultConfig();
    config.gyroGain[ROLL] = 0.011f;
    EXPECT_FALSE(fwGyroAssistConfigIsValid(&config));
    config = defaultConfig();
    config.transitionTimeMs = 0;
    EXPECT_FALSE(fwGyroAssistConfigIsValid(&config));
    config.transitionTimeMs = FW_GYRO_ASSIST_MIN_TRANSITION_TIME_MS;
    EXPECT_TRUE(fwGyroAssistConfigIsValid(&config));
    config = defaultConfig();
    config.gyroTimeoutMs = 0;
    EXPECT_FALSE(fwGyroAssistConfigIsValid(&config));
    config = defaultConfig();
    config.gyroLpfHz = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(fwGyroAssistConfigIsValid(&config));
}

TEST(FwGyroAssist, DormantControllerIgnoresUnrelatedSensorState)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    fwGyroAssistInput_t input = defaultInput();
    input.conditions.requested = false;
    input.conditions.gyroValid = false;
    input.fallbackCommand[ROLL] = 0.35f;
    input.deltaTimeSeconds = 0.0f;

    const fwGyroAssistOutput_t output = fwGyroAssistUpdate(&controller, nullptr, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_PHASE_INACTIVE, output.phase);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_NOT_REQUESTED, output.reason);
    EXPECT_FALSE(output.hardInhibit);
    EXPECT_FLOAT_EQ(0.35f, output.axis[ROLL].command);
}

TEST(FwGyroAssist, EntryStartsAtExistingControllerOutput)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.pilotCommand[ROLL] = -0.3f;
    input.fallbackCommand[ROLL] = 0.6f;
    input.gyroRateDps[ROLL] = 10.0f;

    const fwGyroAssistOutput_t output = update(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_PHASE_ENTERING, output.phase);
    EXPECT_FLOAT_EQ(0.0f, output.blend);
    EXPECT_FLOAT_EQ(0.6f, output.axis[ROLL].command);
    EXPECT_TRUE(output.usingGyroAssist);
}

TEST(FwGyroAssist, CrossFadesToPilotDeflection)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.pilotCommand[ROLL] = 0.2f;
    input.fallbackCommand[ROLL] = 0.6f;

    update(&controller, &config, &input);
    const fwGyroAssistOutput_t output = update(&controller, &config, &input);
    EXPECT_NEAR(0.1f, output.blend, 1e-6f);
    EXPECT_NEAR(0.56f, output.axis[ROLL].command, 1e-6f);

    const fwGyroAssistOutput_t active = advanceToActive(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_PHASE_ACTIVE, active.phase);
    EXPECT_FLOAT_EQ(1.0f, active.blend);
    EXPECT_NEAR(0.2f, active.axis[ROLL].command, 1e-6f);
}

TEST(FwGyroAssist, NeutralPilotAndStillAirRemainNeutral)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();

    const fwGyroAssistOutput_t output = advanceToActive(&controller, &config, &input);
    for (unsigned axis = 0; axis < FW_GYRO_ASSIST_AXIS_COUNT; axis++) {
        EXPECT_FLOAT_EQ(0.0f, output.axis[axis].correction);
        EXPECT_FLOAT_EQ(0.0f, output.axis[axis].command);
        EXPECT_FALSE(output.axis[axis].correctionLimited);
    }
}

TEST(FwGyroAssist, GyroCorrectionOpposesRotation)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    fwGyroAssistConfig_t config = defaultConfig();
    config.correctionLimit[ROLL] = 0.5f;
    config.correctionLimit[PITCH] = 0.5f;
    config.stickPriority[ROLL] = 1.0f;
    config.stickPriority[PITCH] = 1.0f;
    fwGyroAssistInput_t input = defaultInput();
    input.gyroRateDps[ROLL] = 20.0f;
    input.gyroRateDps[PITCH] = -30.0f;

    const fwGyroAssistOutput_t output = advanceToActive(&controller, &config, &input);
    EXPECT_NEAR(-0.2f, output.axis[ROLL].correction, 1e-6f);
    EXPECT_NEAR(0.3f, output.axis[PITCH].correction, 1e-6f);
}

TEST(FwGyroAssist, GustCorrectionIsLimited)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.gyroRateDps[ROLL] = 200.0f;

    const fwGyroAssistOutput_t output = advanceToActive(&controller, &config, &input);
    EXPECT_FLOAT_EQ(-0.25f, output.axis[ROLL].correction);
    EXPECT_FLOAT_EQ(-0.25f, output.axis[ROLL].command);
    EXPECT_TRUE(output.axis[ROLL].correctionLimited);
}

TEST(FwGyroAssist, FullStickAlwaysPreservesPilotThrow)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    fwGyroAssistConfig_t config = defaultConfig();
    config.correctionLimit[ROLL] = 0.5f;
    config.correctionLimit[PITCH] = 0.5f;
    config.stickPriority[ROLL] = 1.0f;
    config.stickPriority[PITCH] = 1.0f;
    fwGyroAssistInput_t input = defaultInput();
    input.pilotCommand[ROLL] = 1.0f;
    input.pilotCommand[PITCH] = -1.0f;
    input.gyroRateDps[ROLL] = 100.0f;
    input.gyroRateDps[PITCH] = -100.0f;

    const fwGyroAssistOutput_t output = advanceToActive(&controller, &config, &input);
    EXPECT_FLOAT_EQ(0.0f, output.axis[ROLL].correction);
    EXPECT_FLOAT_EQ(1.0f, output.axis[ROLL].command);
    EXPECT_FLOAT_EQ(0.0f, output.axis[PITCH].correction);
    EXPECT_FLOAT_EQ(-1.0f, output.axis[PITCH].command);
}

TEST(FwGyroAssist, NearFullStickPriorityReducesEffectiveGain)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    fwGyroAssistConfig_t config = defaultConfig();
    config.correctionLimit[ROLL] = 0.5f;
    config.stickPriority[ROLL] = 1.0f;
    fwGyroAssistInput_t input = defaultInput();
    input.pilotCommand[ROLL] = 0.9f;
    input.gyroRateDps[ROLL] = -100.0f;

    const fwGyroAssistOutput_t output = advanceToActive(&controller, &config, &input);
    EXPECT_NEAR(0.1f, output.axis[ROLL].correction, 1e-6f);
    EXPECT_FLOAT_EQ(1.0f, output.axis[ROLL].command);
}

TEST(FwGyroAssist, StickPriorityReducesEffectiveGainWithoutChangingCorrectionLimit)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    fwGyroAssistConfig_t config = defaultConfig();
    config.correctionLimit[ROLL] = 0.5f;
    config.stickPriority[ROLL] = 1.0f;
    fwGyroAssistInput_t input = defaultInput();
    input.pilotCommand[ROLL] = 0.5f;
    input.gyroRateDps[ROLL] = 20.0f;

    const fwGyroAssistOutput_t output = advanceToActive(&controller, &config, &input);
    EXPECT_NEAR(0.5f, output.axis[ROLL].priorityGainScale, 1e-6f);
    EXPECT_NEAR(0.005f, output.axis[ROLL].effectiveGyroGain, 1e-6f);
    EXPECT_NEAR(-0.1f, output.axis[ROLL].correction, 1e-6f);
    EXPECT_FALSE(output.axis[ROLL].correctionLimited);
}

TEST(FwGyroAssist, PriorityAboveOneMovesZeroGainPointInward)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    fwGyroAssistConfig_t config = defaultConfig();
    config.stickPriority[ROLL] = 1.4f;
    fwGyroAssistInput_t input = defaultInput();
    input.pilotCommand[ROLL] = 0.6f;
    input.gyroRateDps[ROLL] = 20.0f;

    const fwGyroAssistOutput_t output = advanceToActive(&controller, &config, &input);
    EXPECT_NEAR(0.0f, output.axis[ROLL].priorityGainScale, 1e-6f);
    EXPECT_NEAR(0.0f, output.axis[ROLL].correction, 1e-6f);
}

TEST(FwGyroAssist, StopReleaseAndLockUseIndependentTimeConstants)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    fwGyroAssistConfig_t config = defaultConfig();
    config.stickPriority[ROLL] = 1.0f;
    config.stopReleaseTimeMs[ROLL] = 100;
    config.stopLockTimeMs[ROLL] = 1000;
    fwGyroAssistInput_t input = defaultInput();
    input.gyroRateDps[ROLL] = 20.0f;
    advanceToActive(&controller, &config, &input);

    input.pilotCommand[ROLL] = 1.0f;
    const fwGyroAssistOutput_t released = update(&controller, &config, &input);
    EXPECT_NEAR(100.0f / 110.0f, released.axis[ROLL].priorityGainScale, 1e-6f);

    input.pilotCommand[ROLL] = 0.0f;
    const fwGyroAssistOutput_t locking = update(&controller, &config, &input);
    EXPECT_GT(locking.axis[ROLL].priorityGainScale, released.axis[ROLL].priorityGainScale);
    EXPECT_LT(locking.axis[ROLL].priorityGainScale, 0.92f);
}

TEST(FwGyroAssist, AllFinalCommandsRemainNormalized)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    fwGyroAssistConfig_t config = defaultConfig();
    config.correctionLimit[YAW] = 0.5f;
    fwGyroAssistInput_t input = defaultInput();
    input.pilotCommand[YAW] = 4.0f;
    input.fallbackCommand[YAW] = -4.0f;
    input.gyroRateDps[YAW] = -1000.0f;

    const fwGyroAssistOutput_t output = advanceToActive(&controller, &config, &input);
    EXPECT_FLOAT_EQ(1.0f, output.axis[YAW].pilotCommand);
    EXPECT_FLOAT_EQ(1.0f, output.axis[YAW].command);
}

TEST(FwGyroAssist, DeliberateExitStartsWithoutOutputStep)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.pilotCommand[ROLL] = 0.2f;
    input.fallbackCommand[ROLL] = -0.6f;
    advanceToActive(&controller, &config, &input);

    input.conditions.requested = false;
    const fwGyroAssistOutput_t firstExit = update(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_PHASE_EXITING, firstExit.phase);
    EXPECT_FLOAT_EQ(1.0f, firstExit.blend);
    EXPECT_NEAR(0.2f, firstExit.axis[ROLL].command, 1e-6f);

    const fwGyroAssistOutput_t fading = update(&controller, &config, &input);
    EXPECT_NEAR(0.9f, fading.blend, 1e-6f);
    EXPECT_NEAR(0.12f, fading.axis[ROLL].command, 1e-6f);

    fwGyroAssistOutput_t inactive = fading;
    for (unsigned i = 0; i < 10; i++) {
        inactive = update(&controller, &config, &input);
    }
    EXPECT_EQ(FW_GYRO_ASSIST_PHASE_INACTIVE, inactive.phase);
    EXPECT_FLOAT_EQ(0.0f, inactive.blend);
    EXPECT_NEAR(-0.6f, inactive.axis[ROLL].command, 1e-6f);
}

TEST(FwGyroAssist, ReentryDuringExitKeepsCurrentBlend)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.pilotCommand[ROLL] = 0.2f;
    input.fallbackCommand[ROLL] = -0.6f;
    advanceToActive(&controller, &config, &input);

    input.conditions.requested = false;
    update(&controller, &config, &input);
    const fwGyroAssistOutput_t fading = update(&controller, &config, &input);
    input.conditions.requested = true;
    const fwGyroAssistOutput_t reentering = update(&controller, &config, &input);

    EXPECT_EQ(FW_GYRO_ASSIST_PHASE_ENTERING, reentering.phase);
    EXPECT_FLOAT_EQ(fading.blend, reentering.blend);
    EXPECT_FLOAT_EQ(fading.axis[ROLL].command, reentering.axis[ROLL].command);
}

TEST(FwGyroAssist, DisarmImmediatelyDropsCorrection)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.fallbackCommand[ROLL] = 0.4f;
    input.gyroRateDps[ROLL] = 20.0f;
    advanceToActive(&controller, &config, &input);

    input.conditions.armed = false;
    const fwGyroAssistOutput_t output = update(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_PHASE_INHIBITED, output.phase);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_DISARMED, output.reason);
    EXPECT_TRUE(output.hardInhibit);
    EXPECT_FLOAT_EQ(0.4f, output.axis[ROLL].command);
}

TEST(FwGyroAssist, FailsafeImmediatelyUsesFallback)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.fallbackCommand[PITCH] = -0.7f;
    advanceToActive(&controller, &config, &input);

    input.conditions.failsafe = true;
    const fwGyroAssistOutput_t output = update(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_FAILSAFE, output.reason);
    EXPECT_FLOAT_EQ(-0.7f, output.axis[PITCH].command);
    EXPECT_FALSE(output.usingGyroAssist);
}

TEST(FwGyroAssist, InvalidAndStaleGyroImmediatelyUseFallback)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.fallbackCommand[YAW] = 0.3f;
    advanceToActive(&controller, &config, &input);

    input.conditions.gyroValid = false;
    fwGyroAssistOutput_t output = fwGyroAssistUpdate(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_GYRO_INVALID, output.reason);
    EXPECT_FLOAT_EQ(0.3f, output.axis[YAW].command);

    input.conditions.gyroValid = true;
    input.nowMs = 2000;
    input.gyroSampleTimeMs = 1979;
    output = fwGyroAssistUpdate(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_GYRO_STALE, output.reason);
    EXPECT_FLOAT_EQ(0.3f, output.axis[YAW].command);
}

TEST(FwGyroAssist, GyroAgeCheckHandlesClockWrap)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.nowMs = 5;
    input.gyroSampleTimeMs = UINT32_MAX - 10;

    fwGyroAssistOutput_t output = fwGyroAssistUpdate(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_NONE, output.reason);

    input.nowMs = 11;
    output = fwGyroAssistUpdate(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_GYRO_STALE, output.reason);
}

TEST(FwGyroAssist, NavigationLaunchAndAutotrimRequestControlledExit)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    advanceToActive(&controller, &config, &input);

    input.conditions.navigationActive = true;
    fwGyroAssistOutput_t output = update(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_PHASE_EXITING, output.phase);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_NAVIGATION, output.reason);
    EXPECT_FALSE(output.hardInhibit);

    input.conditions.navigationActive = false;
    input.conditions.launchActive = true;
    output = update(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_LAUNCH, output.reason);

    input.conditions.launchActive = false;
    input.conditions.autotrimActive = true;
    output = update(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_AUTOTRIM, output.reason);
}

TEST(FwGyroAssist, DisallowedPilotModeRequestsControlledExit)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    advanceToActive(&controller, &config, &input);

    input.conditions.modeAllowed = false;
    const fwGyroAssistOutput_t output = update(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_PHASE_EXITING, output.phase);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_MODE_NOT_ALLOWED, output.reason);
}

TEST(FwGyroAssist, LowPassFilterAttenuatesStepResponse)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    fwGyroAssistConfig_t config = defaultConfig();
    config.gyroLpfHz = 1.0f;
    config.correctionLimit[ROLL] = 0.5f;
    fwGyroAssistInput_t input = defaultInput();
    advanceToActive(&controller, &config, &input);

    input.gyroRateDps[ROLL] = 10.0f;
    const fwGyroAssistOutput_t output = update(&controller, &config, &input);
    EXPECT_GT(output.axis[ROLL].filteredGyroRateDps, 0.0f);
    EXPECT_LT(output.axis[ROLL].filteredGyroRateDps, 10.0f);
    EXPECT_LT(output.axis[ROLL].correction, 0.0f);
    EXPECT_GT(output.axis[ROLL].correction, -0.1f);
}

TEST(FwGyroAssist, InvalidTimingAndInputAreHardInhibits)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.deltaTimeSeconds = 0.0f;

    fwGyroAssistOutput_t output = fwGyroAssistUpdate(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_INVALID_TIMESTEP, output.reason);
    EXPECT_TRUE(output.hardInhibit);

    input.deltaTimeSeconds = 0.01f;
    input.pilotCommand[ROLL] = std::numeric_limits<float>::infinity();
    output = fwGyroAssistUpdate(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_INVALID_INPUT, output.reason);
    EXPECT_FLOAT_EQ(0.0f, output.axis[ROLL].pilotCommand);
}

TEST(FwGyroAssist, InvalidConfigurationUsesExistingController)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.fallbackCommand[ROLL] = 0.45f;
    config.correctionLimit[ROLL] = -0.1f;

    const fwGyroAssistOutput_t output = fwGyroAssistUpdate(&controller, &config, &input);
    EXPECT_EQ(FW_GYRO_ASSIST_REASON_INVALID_CONFIGURATION, output.reason);
    EXPECT_FLOAT_EQ(0.45f, output.axis[ROLL].command);
}

TEST(FwGyroAssist, MixerReversalFlipsCompleteLogicalCommand)
{
    fwGyroAssistController_t controller;
    fwGyroAssistInit(&controller);
    const fwGyroAssistConfig_t config = defaultConfig();
    fwGyroAssistInput_t input = defaultInput();
    input.pilotCommand[ROLL] = 0.4f;
    input.gyroRateDps[ROLL] = 10.0f;

    const fwGyroAssistOutput_t output = advanceToActive(&controller, &config, &input);
    const float normalMixerRate = 1.0f;
    const float reversedMixerRate = -1.0f;
    const float normalPhysicalCommand = output.axis[ROLL].command * normalMixerRate;
    const float reversedPhysicalCommand = output.axis[ROLL].command * reversedMixerRate;

    EXPECT_NEAR(0.3f, normalPhysicalCommand, 1e-6f);
    EXPECT_FLOAT_EQ(-normalPhysicalCommand, reversedPhysicalCommand);
}
