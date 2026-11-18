#pragma once

#include <stdint.h>

#include "aeb/config.hpp"
#include "aeb/types.hpp"

namespace aeb {

/// Automatic Emergency Braking decision logic.
///
/// Call step() once per control cycle (nominally every 10 ms). Uses no heap,
/// exceptions or standard library so it also builds for the bare-metal target.
class AebController {
public:
    explicit AebController(const Config& config = Config{});

    Output step(const VehicleInput& vehicle, const ObjectInput& object, uint32_t nowMs);

    void reset();

    State state() const { return state_; }
    const Config& config() const { return cfg_; }

private:
    enum class Threat : uint8_t { None, Warning, Partial, Full };

    bool isSensorHealthy(const ObjectInput& object, uint32_t nowMs) const;
    Threat assessThreat(float ttcS, Threat current) const;
    State nextState(const VehicleInput& vehicle, bool sensorHealthy, float ttcS, uint32_t nowMs);
    float targetDecel(State target) const;
    float rampDecel(float targetMps2, uint32_t dtMs) const;

    static Threat threatOf(State from);
    static State stateOf(Threat threat);

    Config cfg_;
    State state_ = State::Standby;
    uint8_t confirmCount_ = 0U;
    uint32_t holdStartMs_ = 0U;
    uint32_t lastStepMs_ = 0U;
    bool hasStepped_ = false;
    float brakeDecelMps2_ = 0.0f;
};

}  // namespace aeb
