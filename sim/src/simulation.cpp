#include "sim/simulation.hpp"

#include "aeb/aeb_controller.hpp"
#include "sim/rng.hpp"
#include "sim/text_buffer.hpp"

namespace sim {

namespace {

constexpr uint32_t kCycleMs = 10U;
constexpr float kDtS = 0.01f;
constexpr uint32_t kMaxDurationMs = 20000U;
constexpr float kMpsToKph = 3.6f;

constexpr float kphToMps(float kph) { return kph / kMpsToKph; }

float clamp(float value, float low, float high) {
    if (value < low) {
        return low;
    }
    return value > high ? high : value;
}

bool evaluate(const Scenario& scenario, const Result& r) {
    switch (scenario.expectation) {
        case Expectation::AvoidCollision:
            return !r.collision;
        case Expectation::MitigateCollision:
            return r.speedReductionPct >= scenario.minSpeedReductionPct;
        case Expectation::NoIntervention:
            return r.firstWarningS < 0.0f && r.maxBrakeRequestMps2 <= 0.0f;
        case Expectation::FailSilent:
            return r.faultSeen && r.maxBrakeRequestMps2 <= 0.0f;
    }
    return false;
}

}  // namespace

const char* toString(Expectation expectation) {
    switch (expectation) {
        case Expectation::AvoidCollision:    return "avoid";
        case Expectation::MitigateCollision: return "mitigate";
        case Expectation::NoIntervention:    return "no-action";
        case Expectation::FailSilent:        return "fail-silent";
    }
    return "?";
}

Result runScenario(const Scenario& scenario, const aeb::Config& config, const PlantParams& plant,
                   SampleCallback callback, void* context) {
    aeb::AebController controller(config);
    Rng rng(plant.seed);

    float egoPosM = 0.0f;
    float egoSpeedMps = kphToMps(scenario.egoSpeedKph);
    float leadPosM = scenario.initialGapM;
    float leadSpeedMps = kphToMps(scenario.leadSpeedKph);
    float brakeActualMps2 = 0.0f;
    float trackedLeadAccelMps2 = 0.0f;
    const float initialRelSpeedKph = scenario.egoSpeedKph - scenario.leadSpeedKph;
    uint32_t lastFrameMs = 0U;

    Result r{};
    r.minGapM = scenario.initialGapM;
    r.firstWarningS = -1.0f;
    r.firstBrakeS = -1.0f;

    uint32_t nowMs = 0U;
    for (; nowMs <= kMaxDurationMs; nowMs += kCycleMs) {
        const float timeS = static_cast<float>(nowMs) * 0.001f;
        const float gapM = leadPosM - egoPosM;

        const bool radarAlive = scenario.radarDropoutS < 0.0f || timeS < scenario.radarDropoutS;
        if (radarAlive) {
            lastFrameMs = nowMs;
        }
        aeb::ObjectInput object{};
        object.frameTimestampMs = lastFrameMs;
        object.objectPresent = gapM <= plant.radarRangeM;
        const float measuredGapM = gapM + plant.radarDistanceNoiseM * rng.gaussian();
        object.distanceM = measuredGapM > 0.0f ? measuredGapM : 0.0f;
        object.closingSpeedMps =
            (egoSpeedMps - leadSpeedMps) + plant.radarSpeedNoiseMps * rng.gaussian();
        // The radar tracker estimates object acceleration with some lag.
        const bool leadBraking = scenario.leadDecelMps2 > 0.0f &&
                                 timeS >= scenario.leadBrakeStartS && leadSpeedMps > 0.0f;
        const float leadAccelMps2 = leadBraking ? -scenario.leadDecelMps2 : 0.0f;
        trackedLeadAccelMps2 +=
            (leadAccelMps2 - trackedLeadAccelMps2) * (kDtS / plant.radarAccelLagS);
        object.objectAccelMps2 = trackedLeadAccelMps2 + plant.radarAccelNoiseMps2 * rng.gaussian();

        const aeb::VehicleInput vehicle{egoSpeedMps, scenario.acceleratorPct, true};
        const aeb::Output out = controller.step(vehicle, object, nowMs);

        if (out.warning && r.firstWarningS < 0.0f) {
            r.firstWarningS = timeS;
        }
        if (out.brakeDecelMps2 > 0.0f && r.firstBrakeS < 0.0f) {
            r.firstBrakeS = timeS;
        }
        if (out.brakeDecelMps2 > r.maxBrakeRequestMps2) {
            r.maxBrakeRequestMps2 = out.brakeDecelMps2;
        }
        r.faultSeen = r.faultSeen || out.state == aeb::State::Fault;

        // Brake actuator: first-order lag, limited by tyre friction.
        brakeActualMps2 += (out.brakeDecelMps2 - brakeActualMps2) * (kDtS / plant.brakeTimeConstantS);
        brakeActualMps2 = clamp(brakeActualMps2, 0.0f, plant.maxDecelMps2);

        if (callback != nullptr) {
            const Sample sample{timeS,   gapM,           egoSpeedMps,     leadSpeedMps,
                                out.ttcS, out.brakeDecelMps2, brakeActualMps2, out.state};
            callback(sample, context);
        }

        // Semi-implicit Euler: speed is updated first, position uses the new speed.
        egoSpeedMps = clamp(egoSpeedMps - brakeActualMps2 * kDtS, 0.0f, 100.0f);
        egoPosM += egoSpeedMps * kDtS;
        if (scenario.leadDecelMps2 > 0.0f && timeS >= scenario.leadBrakeStartS) {
            leadSpeedMps = clamp(leadSpeedMps - scenario.leadDecelMps2 * kDtS, 0.0f, 100.0f);
        }
        leadPosM += leadSpeedMps * kDtS;

        const float newGapM = leadPosM - egoPosM;
        if (newGapM < r.minGapM) {
            r.minGapM = newGapM;
        }
        if (newGapM <= 0.0f) {
            r.collision = true;
            r.impactSpeedKph = (egoSpeedMps - leadSpeedMps) * kMpsToKph;
            break;
        }
        // Done once AEB has stopped the car and released the hold.
        if (egoSpeedMps <= 0.0f && out.state == aeb::State::Standby) {
            break;
        }
    }
    r.durationS = static_cast<float>(nowMs) * 0.001f;

    if (!r.collision) {
        r.speedReductionPct = 100.0f;
    } else if (initialRelSpeedKph > 0.0f) {
        r.speedReductionPct = 100.0f * (1.0f - r.impactSpeedKph / initialRelSpeedKph);
    }
    r.passed = evaluate(scenario, r);
    return r;
}

size_t formatResult(char* buffer, size_t size, const Scenario& scenario, const Result& r) {
    TextBuffer t(buffer, size);
    t.str("RESULT ").str(scenario.name).padTo(22);
    t.str(toString(scenario.expectation)).padTo(34);
    t.str(r.passed ? "PASS" : "FAIL").padTo(40);
    t.str("impact=").fixed(r.impactSpeedKph, 1).str("km/h").padTo(58);
    t.str("min_gap=").fixed(r.minGapM, 2).str("m").padTo(75);
    t.str("warn@");
    if (r.firstWarningS < 0.0f) {
        t.str("-");
    } else {
        t.fixed(r.firstWarningS, 2).str("s");
    }
    t.padTo(87).str("brake@");
    if (r.firstBrakeS < 0.0f) {
        t.str("-");
    } else {
        t.fixed(r.firstBrakeS, 2).str("s");
    }
    t.padTo(100).str("peak=").fixed(r.maxBrakeRequestMps2, 2).str("m/s2");
    return t.size();
}

}  // namespace sim
