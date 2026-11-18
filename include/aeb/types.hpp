#pragma once

#include <stdint.h>

namespace aeb {

/// Ego vehicle signals, sampled once per control cycle.
struct VehicleInput {
    float egoSpeedMps;     ///< Longitudinal speed, >= 0 [m/s]
    float acceleratorPct;  ///< Accelerator pedal position [0..100 %]
    bool aebSwitchOn;      ///< Driver-selectable AEB on/off switch
};

/// Most relevant (closest in-path) object reported by the forward radar.
struct ObjectInput {
    uint32_t frameTimestampMs;  ///< Time at which the latest radar frame was received [ms]
    bool objectPresent;         ///< True if an in-path object is being tracked
    float distanceM;            ///< Bumper-to-bumper distance [m]
    float closingSpeedMps;      ///< Ego speed minus object speed; > 0 means approaching [m/s]
    float objectAccelMps2;      ///< Object longitudinal acceleration from the radar tracker; < 0 = braking [m/s^2]
};

enum class State : uint8_t {
    Off,           ///< Switched off by the driver
    Standby,       ///< Monitoring, no threat
    Warning,       ///< Forward collision warning to the driver
    PartialBrake,  ///< Moderate autonomous braking
    FullBrake,     ///< Maximum autonomous braking
    Hold,          ///< Vehicle stopped by AEB, holding the brake
    Fault,         ///< Sensor data missing or implausible (fail-silent)
};

struct Output {
    State state;
    bool warning;          ///< Audible/visual forward collision warning
    bool faultLamp;        ///< AEB malfunction indicator
    float brakeDecelMps2;  ///< Deceleration request to the brake system, >= 0 [m/s^2]
    float ttcS;            ///< Time to collision evaluated this cycle [s]
};

const char* toString(State state);

}  // namespace aeb
