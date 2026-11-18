#pragma once

#include <stddef.h>
#include <stdint.h>

#include "aeb/config.hpp"
#include "aeb/types.hpp"

namespace sim {

enum class Expectation : uint8_t {
    AvoidCollision,     ///< Must stop before hitting the target
    MitigateCollision,  ///< Impact speed must drop by at least minSpeedReductionPct
    NoIntervention,     ///< No warning and no braking (false-positive check)
    FailSilent,         ///< Must report a fault and never brake
};

/// One closed-loop test case, modelled on the Euro NCAP AEB Car-to-Car tests.
struct Scenario {
    const char* name;
    float egoSpeedKph;
    float leadSpeedKph;
    float initialGapM;
    float leadDecelMps2;        ///< 0 = lead keeps its speed
    float leadBrakeStartS;      ///< When the lead starts braking
    float acceleratorPct;       ///< Driver accelerator pedal during the test
    float radarDropoutS;        ///< Radar stops sending frames at this time; < 0 = never
    Expectation expectation;
    float minSpeedReductionPct; ///< Only used for MitigateCollision
};

/// Vehicle and sensor model parameters.
struct PlantParams {
    float brakeTimeConstantS = 0.15f;  ///< First-order lag of the hydraulic brake
    float maxDecelMps2 = 9.3f;         ///< Tyre/road friction limit (dry asphalt)
    float radarDistanceNoiseM = 0.05f;
    float radarSpeedNoiseMps = 0.05f;
    float radarAccelNoiseMps2 = 0.1f;
    float radarAccelLagS = 0.1f;       ///< Tracker filter lag on object acceleration
    float radarRangeM = 160.0f;
    uint32_t seed = 12345U;
};

/// Signals recorded every cycle (used for CSV export and plotting).
struct Sample {
    float timeS;
    float gapM;
    float egoSpeedMps;
    float leadSpeedMps;
    float ttcS;
    float brakeRequestMps2;
    float brakeActualMps2;
    aeb::State state;
};

struct Result {
    bool passed;
    bool collision;
    bool faultSeen;
    float impactSpeedKph;      ///< Relative speed at impact, 0 if no collision
    float speedReductionPct;   ///< 100 % = collision avoided
    float minGapM;
    float firstWarningS;       ///< < 0 = never
    float firstBrakeS;         ///< < 0 = never
    float maxBrakeRequestMps2;
    float durationS;
};

using SampleCallback = void (*)(const Sample& sample, void* context);

extern const Scenario kScenarios[];
extern const size_t kScenarioCount;

/// Runs one scenario in closed loop: radar model -> AebController -> brake
/// actuator -> vehicle dynamics, at a fixed 10 ms step.
Result runScenario(const Scenario& scenario, const aeb::Config& config, const PlantParams& plant,
                   SampleCallback callback = nullptr, void* context = nullptr);

/// Formats a one-line result report ("RESULT <name> ..."). Shared by the
/// host and the firmware so both print byte-identical text.
size_t formatResult(char* buffer, size_t size, const Scenario& scenario, const Result& result);

const char* toString(Expectation expectation);

}  // namespace sim
