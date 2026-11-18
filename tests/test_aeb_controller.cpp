// Unit tests for aeb::AebController.
// Test names start with the requirement ID they verify (docs/REQUIREMENTS.md).

#include <gtest/gtest.h>

#include <limits>

#include "aeb/aeb_controller.hpp"

using aeb::AebController;
using aeb::Config;
using aeb::ObjectInput;
using aeb::Output;
using aeb::State;
using aeb::VehicleInput;

namespace {

constexpr uint32_t kCycleMs = 10U;
constexpr float kClosingMps = 10.0f;

class AebControllerTest : public ::testing::Test {
protected:
    Config cfg{};
    AebController ctrl{cfg};
    uint32_t nowMs = 0U;
    VehicleInput vehicle{50.0f / 3.6f, 0.0f, true};
    float objectAccelMps2 = 0.0f;

    /// One 10 ms cycle with a fresh radar frame.
    Output step(float distanceM, float closingMps, bool present = true) {
        nowMs += kCycleMs;
        const ObjectInput object{nowMs, present, distanceM, closingMps, objectAccelMps2};
        return ctrl.step(vehicle, object, nowMs);
    }

    Output stepFor(int cycles, float distanceM, float closingMps) {
        Output out{};
        for (int i = 0; i < cycles; ++i) {
            out = step(distanceM, closingMps);
        }
        return out;
    }

    /// Runs `cycles` cycles with an object at the given TTC.
    Output stepTtc(float ttcS, int cycles = 1) {
        Output out{};
        for (int i = 0; i < cycles; ++i) {
            out = step(ttcS * kClosingMps, kClosingMps);
        }
        return out;
    }

    Output stepNoObject(int cycles = 1) {
        Output out{};
        for (int i = 0; i < cycles; ++i) {
            out = step(0.0f, 0.0f, false);
        }
        return out;
    }

    /// Drives the controller into FullBrake with the request fully built up.
    void enterFullBrake() {
        stepTtc(0.5f, 50);
        ASSERT_EQ(ctrl.state(), State::FullBrake);
    }
};

}  // namespace

TEST_F(AebControllerTest, REQ_AEB_001_SwitchedOffNeverWarnsOrBrakes) {
    vehicle.aebSwitchOn = false;
    const Output out = stepTtc(0.5f, 20);
    EXPECT_EQ(out.state, State::Off);
    EXPECT_FALSE(out.warning);
    EXPECT_EQ(out.brakeDecelMps2, 0.0f);
}

TEST_F(AebControllerTest, REQ_AEB_002_NoObjectStaysInStandby) {
    const Output out = stepNoObject(20);
    EXPECT_EQ(out.state, State::Standby);
    EXPECT_FALSE(out.warning);
    EXPECT_EQ(out.brakeDecelMps2, 0.0f);
}

TEST_F(AebControllerTest, REQ_AEB_010_NoActivationBelowMinimumSpeed) {
    vehicle.egoSpeedMps = 4.0f / 3.6f;
    EXPECT_EQ(stepTtc(0.5f, 20).state, State::Standby);
}

TEST_F(AebControllerTest, REQ_AEB_010_NoActivationAboveMaximumSpeed) {
    vehicle.egoSpeedMps = 140.0f / 3.6f;
    EXPECT_EQ(stepTtc(0.5f, 20).state, State::Standby);
}

TEST_F(AebControllerTest, REQ_AEB_010_ActivatesInsideSpeedWindow) {
    vehicle.egoSpeedMps = 6.0f / 3.6f;
    EXPECT_EQ(stepTtc(2.0f, 2).state, State::Warning);
}

TEST_F(AebControllerTest, REQ_AEB_020_WarnsBelowWarningTtc) {
    EXPECT_EQ(stepTtc(2.7f, 5).state, State::Standby);
    const Output out = stepTtc(2.5f, 2);
    EXPECT_EQ(out.state, State::Warning);
    EXPECT_TRUE(out.warning);
    EXPECT_EQ(out.brakeDecelMps2, 0.0f);
}

TEST_F(AebControllerTest, REQ_AEB_021_PartialBrakeBelowPartialTtc) {
    stepTtc(2.0f, 2);
    const Output out = stepTtc(1.5f, 2);
    EXPECT_EQ(out.state, State::PartialBrake);
    EXPECT_TRUE(out.warning);
    EXPECT_FLOAT_EQ(stepTtc(1.5f, 20).brakeDecelMps2, cfg.partialDecelMps2);
}

TEST_F(AebControllerTest, REQ_AEB_022_FullBrakeBelowFullTtc) {
    stepTtc(1.5f, 2);
    const Output out = stepTtc(0.9f, 2);
    EXPECT_EQ(out.state, State::FullBrake);
    EXPECT_FLOAT_EQ(stepTtc(0.9f, 30).brakeDecelMps2, cfg.fullDecelMps2);
}

TEST_F(AebControllerTest, REQ_AEB_023_DirectEscalationOnCutIn) {
    stepNoObject(5);
    EXPECT_EQ(stepTtc(0.4f, 2).state, State::FullBrake);
}

TEST_F(AebControllerTest, REQ_AEB_024_SingleCycleSpikeIsIgnored) {
    stepNoObject(5);
    EXPECT_EQ(stepTtc(0.4f, 1).state, State::Standby);  // spike
    EXPECT_EQ(stepNoObject(1).state, State::Standby);   // gone again
    EXPECT_EQ(stepTtc(0.4f, 1).state, State::Standby);  // counter restarted
    EXPECT_EQ(stepTtc(0.4f, 1).state, State::FullBrake);
}

TEST_F(AebControllerTest, REQ_AEB_025_WarnsWhenLeadBrakesAtEqualSpeed) {
    // Same speed (closing 0): constant-speed TTC is infinite, but the lead is
    // braking at 6 m/s^2 only 12 m ahead -> impact in 2 s if ego does nothing.
    objectAccelMps2 = -6.0f;
    const Output out = stepFor(2, 12.0f, 0.0f);
    EXPECT_EQ(out.state, State::Warning);
    EXPECT_NEAR(out.ttcS, 2.0f, 1e-3f);
}

TEST_F(AebControllerTest, REQ_AEB_025_SmallObjectDecelerationIsIgnored) {
    objectAccelMps2 = -0.8f;  // tracker noise / gentle lift-off, above -1 m/s^2
    EXPECT_EQ(stepFor(20, 12.0f, 0.0f).state, State::Standby);
}

TEST_F(AebControllerTest, REQ_AEB_030_WarningUsesHysteresis) {
    stepTtc(2.5f, 2);
    ASSERT_EQ(ctrl.state(), State::Warning);
    EXPECT_EQ(stepTtc(2.8f, 5).state, State::Warning);  // < 2.6 + 0.3
    EXPECT_EQ(stepTtc(3.0f, 1).state, State::Standby);
}

TEST_F(AebControllerTest, REQ_AEB_031_NoStepDownFromFullToPartial) {
    enterFullBrake();
    EXPECT_EQ(stepTtc(1.4f, 10).state, State::FullBrake);
    EXPECT_EQ(stepTtc(2.5f, 10).state, State::FullBrake);
}

// Regression for a bug found in closed-loop simulation (CCRs-10..30): while
// slowing down towards a stopped car TTC grows again; releasing on TTC made the
// vehicle coast into the target.
TEST_F(AebControllerTest, REQ_AEB_031_KeepsBrakingWhileStillClosing) {
    enterFullBrake();
    EXPECT_EQ(stepTtc(8.0f, 20).state, State::FullBrake);
}

TEST_F(AebControllerTest, REQ_AEB_031_ReleasesWhenThreatIsGone) {
    enterFullBrake();
    const Output out = step(10.0f, 0.0f);  // speeds matched: TTC infinite
    EXPECT_EQ(out.state, State::Standby);
    EXPECT_FALSE(out.warning);
}

TEST_F(AebControllerTest, REQ_AEB_032_BrakingContinuesBelowActivationSpeed) {
    enterFullBrake();
    vehicle.egoSpeedMps = 1.0f;  // below 5 km/h, above standstill
    EXPECT_EQ(stepTtc(0.5f, 5).state, State::FullBrake);
}

TEST_F(AebControllerTest, REQ_AEB_033_BrakeRequestRiseIsRateLimited) {
    float previous = 0.0f;
    for (int i = 0; i < 40; ++i) {
        const float decel = stepTtc(0.4f).brakeDecelMps2;
        EXPECT_LE(decel - previous, cfg.brakeRiseRateMps3 * 0.010f + 1e-5f) << "cycle " << i;
        previous = decel;
    }
    EXPECT_FLOAT_EQ(previous, cfg.fullDecelMps2);
}

TEST_F(AebControllerTest, REQ_AEB_033_BrakeRequestReleaseIsRateLimited) {
    enterFullBrake();
    float previous = cfg.fullDecelMps2;
    for (int i = 0; i < 60; ++i) {
        const float decel = step(10.0f, 0.0f).brakeDecelMps2;
        EXPECT_LE(previous - decel, cfg.brakeReleaseRateMps3 * 0.010f + 1e-5f) << "cycle " << i;
        previous = decel;
    }
    EXPECT_FLOAT_EQ(previous, 0.0f);
}

TEST_F(AebControllerTest, REQ_AEB_040_HoldsBrakeAtStandstill) {
    enterFullBrake();
    vehicle.egoSpeedMps = 0.0f;
    EXPECT_EQ(stepTtc(0.5f).state, State::Hold);
    EXPECT_FLOAT_EQ(stepTtc(0.5f, 50).brakeDecelMps2, cfg.holdDecelMps2);
}

// Regression for a race found in simulation (CCRs-10, CCRb-12m-6mps2): close to
// standstill behind a stopped object, "speeds matched" must not release the
// brake before Hold is reached.
TEST_F(AebControllerTest, REQ_AEB_040_BrakesToStandstillBehindStoppedObject) {
    enterFullBrake();
    vehicle.egoSpeedMps = 0.5f;          // crawling, below the activation window
    EXPECT_EQ(step(2.0f, 0.05f).state, State::FullBrake);  // closing < 0.1 m/s
    vehicle.egoSpeedMps = 0.0f;
    EXPECT_EQ(step(2.0f, 0.0f).state, State::Hold);
}

TEST_F(AebControllerTest, REQ_AEB_041_HoldEndsAfterHoldTime) {
    enterFullBrake();
    vehicle.egoSpeedMps = 0.0f;
    stepTtc(0.5f);  // enters Hold
    const int holdCycles = static_cast<int>(cfg.holdTimeMs / kCycleMs);
    EXPECT_EQ(stepTtc(0.5f, holdCycles - 1).state, State::Hold);
    EXPECT_EQ(stepTtc(0.5f).state, State::Standby);
}

TEST_F(AebControllerTest, REQ_AEB_050_KickdownCancelsIntervention) {
    enterFullBrake();
    vehicle.acceleratorPct = 95.0f;
    const Output out = stepTtc(0.5f);
    EXPECT_EQ(out.state, State::Standby);
    EXPECT_FALSE(out.warning);
    EXPECT_LT(out.brakeDecelMps2, cfg.fullDecelMps2);
    EXPECT_EQ(stepTtc(0.5f, 50).brakeDecelMps2, 0.0f);
}

TEST_F(AebControllerTest, REQ_AEB_050_NormalPedalDoesNotOverride) {
    vehicle.acceleratorPct = 30.0f;
    EXPECT_EQ(stepTtc(0.5f, 2).state, State::FullBrake);
}

TEST_F(AebControllerTest, REQ_AEB_060_StaleRadarFrameIsFault) {
    stepTtc(3.0f, 5);
    const uint32_t lastFrameMs = nowMs;
    Output out{};
    for (int i = 0; i < 25; ++i) {
        nowMs += kCycleMs;
        out = ctrl.step(vehicle, ObjectInput{lastFrameMs, false, 0.0f, 0.0f, 0.0f}, nowMs);
    }
    EXPECT_EQ(out.state, State::Fault);
    EXPECT_TRUE(out.faultLamp);
    EXPECT_FALSE(out.warning);
    EXPECT_EQ(out.brakeDecelMps2, 0.0f);
}

TEST_F(AebControllerTest, REQ_AEB_060_FrameWithinTimeoutIsAccepted) {
    const uint32_t frameMs = nowMs + kCycleMs;
    nowMs += cfg.sensorTimeoutMs;
    const Output out = ctrl.step(vehicle, ObjectInput{frameMs, false, 0.0f, 0.0f, 0.0f}, nowMs);
    EXPECT_EQ(out.state, State::Standby);
}

TEST_F(AebControllerTest, REQ_AEB_061_NanDistanceIsFault) {
    EXPECT_EQ(step(std::numeric_limits<float>::quiet_NaN(), kClosingMps).state, State::Fault);
}

TEST_F(AebControllerTest, REQ_AEB_061_InfiniteClosingSpeedIsFault) {
    EXPECT_EQ(step(10.0f, std::numeric_limits<float>::infinity()).state, State::Fault);
}

TEST_F(AebControllerTest, REQ_AEB_061_OutOfRangeValuesAreFault) {
    EXPECT_EQ(step(-0.5f, kClosingMps).state, State::Fault);
    EXPECT_EQ(step(300.0f, kClosingMps).state, State::Fault);
    EXPECT_EQ(step(30.0f, 80.0f).state, State::Fault);
    objectAccelMps2 = -20.0f;
    EXPECT_EQ(step(30.0f, 5.0f).state, State::Fault);
}

TEST_F(AebControllerTest, REQ_AEB_062_FaultDuringBrakingRampsDown) {
    enterFullBrake();
    const Output out = step(std::numeric_limits<float>::quiet_NaN(), kClosingMps);
    EXPECT_EQ(out.state, State::Fault);
    EXPECT_NEAR(out.brakeDecelMps2, cfg.fullDecelMps2 - cfg.brakeReleaseRateMps3 * 0.010f, 1e-4f);
}

TEST_F(AebControllerTest, REQ_AEB_063_RecoversFromFault) {
    step(-1.0f, kClosingMps);
    ASSERT_EQ(ctrl.state(), State::Fault);
    EXPECT_EQ(stepNoObject().state, State::Standby);
    EXPECT_EQ(stepTtc(2.0f, 2).state, State::Warning);
}

TEST_F(AebControllerTest, TimestampWrapAroundIsHandled) {
    nowMs = 0xFFFFFFFFU - 25U;
    stepTtc(2.0f, 5);  // crosses the 32-bit wrap
    EXPECT_EQ(ctrl.state(), State::Warning);
}

TEST_F(AebControllerTest, ResetReturnsToInitialState) {
    enterFullBrake();
    ctrl.reset();
    EXPECT_EQ(ctrl.state(), State::Standby);
    EXPECT_EQ(stepNoObject().brakeDecelMps2, 0.0f);
}

TEST(AebState, ToStringNamesEveryState) {
    EXPECT_STREQ(aeb::toString(State::Off), "Off");
    EXPECT_STREQ(aeb::toString(State::FullBrake), "FullBrake");
    EXPECT_STREQ(aeb::toString(State::Fault), "Fault");
}
