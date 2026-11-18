// Closed-loop test catalogue, modelled on the Euro NCAP AEB Car-to-Car protocol:
//   CCRs - Car-to-Car Rear stationary
//   CCRm - Car-to-Car Rear moving (target at constant 20 km/h)
//   CCRb - Car-to-Car Rear braking (target brakes in front of ego)
// plus false-positive, driver-override and sensor-failure cases.

#include "sim/simulation.hpp"

namespace sim {

namespace {

constexpr float kNoDropout = -1.0f;
constexpr float kStartTtcS = 4.0f;  // every approach starts 4 s before impact

constexpr float startGap(float egoKph, float leadKph) {
    return (egoKph - leadKph) / 3.6f * kStartTtcS;
}

constexpr Scenario ccrs(const char* name, float egoKph, Expectation expectation,
                        float minReductionPct = 0.0f) {
    return Scenario{name, egoKph, 0.0f, startGap(egoKph, 0.0f), 0.0f, 0.0f, 0.0f,
                    kNoDropout, expectation, minReductionPct};
}

constexpr Scenario ccrm(const char* name, float egoKph) {
    return Scenario{name, egoKph, 20.0f, startGap(egoKph, 20.0f), 0.0f, 0.0f, 0.0f,
                    kNoDropout, Expectation::AvoidCollision, 0.0f};
}

constexpr Scenario ccrb(const char* name, float gapM, float leadDecelMps2) {
    return Scenario{name, 50.0f, 50.0f, gapM, leadDecelMps2, 1.0f, 0.0f,
                    kNoDropout, Expectation::AvoidCollision, 0.0f};
}

}  // namespace

const Scenario kScenarios[] = {
    // REQ-AEB-070 / -071
    ccrs("CCRs-10", 10.0f, Expectation::AvoidCollision),
    ccrs("CCRs-20", 20.0f, Expectation::AvoidCollision),
    ccrs("CCRs-30", 30.0f, Expectation::AvoidCollision),
    ccrs("CCRs-40", 40.0f, Expectation::AvoidCollision),
    ccrs("CCRs-50", 50.0f, Expectation::AvoidCollision),
    ccrs("CCRs-60", 60.0f, Expectation::MitigateCollision, 50.0f),
    ccrs("CCRs-70", 70.0f, Expectation::MitigateCollision, 50.0f),
    ccrs("CCRs-80", 80.0f, Expectation::MitigateCollision, 50.0f),
    // REQ-AEB-072
    ccrm("CCRm-30", 30.0f),
    ccrm("CCRm-50", 50.0f),
    ccrm("CCRm-70", 70.0f),
    // REQ-AEB-073
    ccrb("CCRb-12m-2mps2", 12.0f, 2.0f),
    ccrb("CCRb-12m-6mps2", 12.0f, 6.0f),
    ccrb("CCRb-40m-2mps2", 40.0f, 2.0f),
    ccrb("CCRb-40m-6mps2", 40.0f, 6.0f),
    // REQ-AEB-074: false-positive checks
    Scenario{"Follow-50", 50.0f, 50.0f, 15.0f, 0.0f, 0.0f, 0.0f, kNoDropout,
             Expectation::NoIntervention, 0.0f},
    Scenario{"PullAway-50", 50.0f, 70.0f, 20.0f, 0.0f, 0.0f, 0.0f, kNoDropout,
             Expectation::NoIntervention, 0.0f},
    // REQ-AEB-050: driver kick-down overrides AEB
    Scenario{"Kickdown-40", 40.0f, 0.0f, startGap(40.0f, 0.0f), 0.0f, 0.0f, 100.0f, kNoDropout,
             Expectation::NoIntervention, 0.0f},
    // REQ-AEB-060: radar stops sending frames -> fail-silent
    Scenario{"RadarLoss-40", 40.0f, 0.0f, startGap(40.0f, 0.0f), 0.0f, 0.0f, 0.0f, 0.5f,
             Expectation::FailSilent, 0.0f},
};

const size_t kScenarioCount = sizeof(kScenarios) / sizeof(kScenarios[0]);

}  // namespace sim
