#include "aeb/aeb_controller.hpp"

#include "aeb/ttc.hpp"

namespace aeb {

namespace {

/// Upper bound for the time step used by the brake ramp, so a late call
/// cannot cause a large jump in the brake request.
constexpr uint32_t kMaxRampStepMs = 100U;

constexpr bool isBraking(State state) {
    return state == State::PartialBrake || state == State::FullBrake;
}

constexpr float absf(float value) { return value < 0.0f ? -value : value; }

/// NaN != NaN, and Inf - Inf == NaN, so this rejects both without <cmath>.
constexpr bool isFinite(float value) { return value == value && (value - value) == 0.0f; }

}  // namespace

const char* toString(State state) {
    switch (state) {
        case State::Off:          return "Off";
        case State::Standby:      return "Standby";
        case State::Warning:      return "Warning";
        case State::PartialBrake: return "PartialBrake";
        case State::FullBrake:    return "FullBrake";
        case State::Hold:         return "Hold";
        case State::Fault:        return "Fault";
    }
    return "?";
}

AebController::AebController(const Config& config) : cfg_(config) {}

void AebController::reset() {
    state_ = State::Standby;
    confirmCount_ = 0U;
    holdStartMs_ = 0U;
    lastStepMs_ = 0U;
    hasStepped_ = false;
    brakeDecelMps2_ = 0.0f;
}

Output AebController::step(const VehicleInput& vehicle, const ObjectInput& object,
                           uint32_t nowMs) {
    const uint32_t dtMs = hasStepped_ ? (nowMs - lastStepMs_) : 0U;
    lastStepMs_ = nowMs;
    hasStepped_ = true;

    const bool sensorHealthy = isSensorHealthy(object, nowMs);
    const float ttcS = (sensorHealthy && object.objectPresent)
                           ? computeTtcWithObjectBraking(
                                 object.distanceM, vehicle.egoSpeedMps, object.closingSpeedMps,
                                 object.objectAccelMps2, cfg_.objectBrakingThresholdMps2)
                           : kTtcInfiniteS;

    state_ = nextState(vehicle, sensorHealthy, ttcS, nowMs);
    brakeDecelMps2_ = rampDecel(targetDecel(state_), dtMs);

    Output out{};
    out.state = state_;
    out.warning = (state_ == State::Warning) || isBraking(state_);
    out.faultLamp = (state_ == State::Fault);
    out.brakeDecelMps2 = brakeDecelMps2_;
    out.ttcS = ttcS;
    return out;
}

// REQ-AEB-060, REQ-AEB-061
bool AebController::isSensorHealthy(const ObjectInput& object, uint32_t nowMs) const {
    // Unsigned subtraction is wrap-around safe. A timestamp from the "future"
    // produces a huge age and is therefore also rejected.
    if (nowMs - object.frameTimestampMs > cfg_.sensorTimeoutMs) {
        return false;
    }
    if (!object.objectPresent) {
        return true;
    }
    return isFinite(object.distanceM) && isFinite(object.closingSpeedMps) &&
           isFinite(object.objectAccelMps2) && object.distanceM >= 0.0f &&
           object.distanceM <= cfg_.maxPlausibleDistanceM &&
           absf(object.closingSpeedMps) <= cfg_.maxPlausibleClosingSpeedMps &&
           absf(object.objectAccelMps2) <= cfg_.maxPlausibleAccelMps2;
}

// REQ-AEB-020..022 with hysteresis (REQ-AEB-030): a level that is already
// active is kept until TTC rises above its threshold + hysteresis.
AebController::Threat AebController::assessThreat(float ttcS, Threat current) const {
    const auto below = [&](float thresholdS, Threat level) {
        const float limit = (current >= level) ? thresholdS + cfg_.ttcHysteresisS : thresholdS;
        return ttcS < limit;
    };
    if (below(cfg_.ttcFullBrakeS, Threat::Full)) {
        return Threat::Full;
    }
    if (below(cfg_.ttcPartialBrakeS, Threat::Partial)) {
        return Threat::Partial;
    }
    if (below(cfg_.ttcWarningS, Threat::Warning)) {
        return Threat::Warning;
    }
    return Threat::None;
}

State AebController::nextState(const VehicleInput& vehicle, bool sensorHealthy, float ttcS,
                               uint32_t nowMs) {
    if (!vehicle.aebSwitchOn) {  // REQ-AEB-001
        confirmCount_ = 0U;
        return State::Off;
    }
    if (!sensorHealthy) {  // REQ-AEB-060, -061
        confirmCount_ = 0U;
        return State::Fault;
    }
    if (vehicle.acceleratorPct >= cfg_.kickdownPct) {  // REQ-AEB-050
        confirmCount_ = 0U;
        return State::Standby;
    }

    const bool braking = isBraking(state_);
    if (braking && vehicle.egoSpeedMps <= cfg_.standstillSpeedMps) {  // REQ-AEB-040
        confirmCount_ = 0U;
        holdStartMs_ = nowMs;
        return State::Hold;
    }
    if (state_ == State::Hold) {  // REQ-AEB-041
        return (nowMs - holdStartMs_ >= cfg_.holdTimeMs) ? State::Standby : State::Hold;
    }

    // Off and Fault fall back to Standby once they are no longer active (REQ-AEB-063).
    const State settled = (state_ == State::Off || state_ == State::Fault) ? State::Standby : state_;
    const Threat current = threatOf(settled);

    Threat requested = assessThreat(ttcS, current);
    const bool inWindow = vehicle.egoSpeedMps >= cfg_.minActiveSpeedMps &&
                          vehicle.egoSpeedMps <= cfg_.maxActiveSpeedMps;
    if (!inWindow && !braking) {  // REQ-AEB-010, REQ-AEB-032
        requested = Threat::None;
    }

    if (requested > current) {  // REQ-AEB-023, REQ-AEB-024
        if (confirmCount_ < 0xFFU) {
            ++confirmCount_;
        }
        if (confirmCount_ >= cfg_.confirmCycles) {
            confirmCount_ = 0U;
            return stateOf(requested);
        }
        return settled;
    }

    confirmCount_ = 0U;
    if (braking) {
        // REQ-AEB-031: never step down between brake levels and keep braking until
        // the speeds match. Releasing when TTC rises is wrong: TTC grows again while
        // slowing towards a stopped car, and the vehicle would coast into it.
        // At crawling speed, finish the stop and go to Hold rather than release,
        // otherwise radar noise decides between the two.
        const bool speedsMatched = ttcS >= kTtcInfiniteS;
        const bool stillDriving = vehicle.egoSpeedMps > cfg_.minActiveSpeedMps;
        return (speedsMatched && stillDriving) ? State::Standby : settled;
    }
    return stateOf(requested);
}

float AebController::targetDecel(State target) const {
    switch (target) {
        case State::PartialBrake: return cfg_.partialDecelMps2;
        case State::FullBrake:    return cfg_.fullDecelMps2;
        case State::Hold:         return cfg_.holdDecelMps2;
        case State::Off:
        case State::Standby:
        case State::Warning:
        case State::Fault:        return 0.0f;
    }
    return 0.0f;
}

// REQ-AEB-033, REQ-AEB-062
float AebController::rampDecel(float targetMps2, uint32_t dtMs) const {
    const uint32_t clampedMs = dtMs < kMaxRampStepMs ? dtMs : kMaxRampStepMs;
    const float dtS = static_cast<float>(clampedMs) * 0.001f;
    const float current = brakeDecelMps2_;

    if (targetMps2 > current) {
        const float next = current + cfg_.brakeRiseRateMps3 * dtS;
        return next < targetMps2 ? next : targetMps2;
    }
    const float next = current - cfg_.brakeReleaseRateMps3 * dtS;
    return next > targetMps2 ? next : targetMps2;
}

AebController::Threat AebController::threatOf(State from) {
    switch (from) {
        case State::Warning:      return Threat::Warning;
        case State::PartialBrake: return Threat::Partial;
        case State::FullBrake:    return Threat::Full;
        case State::Off:
        case State::Standby:
        case State::Hold:
        case State::Fault:        return Threat::None;
    }
    return Threat::None;
}

State AebController::stateOf(Threat threat) {
    switch (threat) {
        case Threat::Warning: return State::Warning;
        case Threat::Partial: return State::PartialBrake;
        case Threat::Full:    return State::FullBrake;
        case Threat::None:    return State::Standby;
    }
    return State::Standby;
}

}  // namespace aeb
