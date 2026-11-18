#pragma once

namespace aeb {

/// TTC value used when no collision is predicted.
constexpr float kTtcInfiniteS = 1.0e6f;

/// Closing speeds below this are treated as "not approaching" to avoid
/// dividing by (almost) zero.
constexpr float kMinClosingSpeedMps = 0.1f;

/// Square root by Newton iteration. The bare-metal target has no libm, and a
/// fixed iteration count keeps the result identical on host and target.
constexpr float sqrtNewton(float x) {
    if (!(x > 0.0f)) {
        return 0.0f;
    }
    float guess = x > 1.0f ? x : 1.0f;
    for (int i = 0; i < 24; ++i) {
        guess = 0.5f * (guess + x / guess);
    }
    return guess;
}

/// Time to collision assuming both vehicles keep their current speed.
constexpr float computeTtc(float distanceM, float closingSpeedMps) {
    if (closingSpeedMps < kMinClosingSpeedMps) {
        return kTtcInfiniteS;
    }
    if (distanceM <= 0.0f) {
        return 0.0f;
    }
    const float ttc = distanceM / closingSpeedMps;
    return ttc < kTtcInfiniteS ? ttc : kTtcInfiniteS;
}

/// "Enhanced" TTC: the ego vehicle keeps its speed (what happens if the driver
/// does nothing) while the object keeps braking with `objectAccelMps2` until it
/// stops. Falls back to the constant-speed TTC when the object is not braking
/// harder than `brakingThresholdMps2` (e.g. -1 m/s^2).
constexpr float computeTtcWithObjectBraking(float distanceM, float egoSpeedMps,
                                            float closingSpeedMps, float objectAccelMps2,
                                            float brakingThresholdMps2) {
    if (objectAccelMps2 > brakingThresholdMps2) {
        return computeTtc(distanceM, closingSpeedMps);
    }
    if (distanceM <= 0.0f) {
        return 0.0f;
    }
    const float decel = -objectAccelMps2;  // > 0
    const float objectSpeed = egoSpeedMps - closingSpeedMps > 0.0f ? egoSpeedMps - closingSpeedMps : 0.0f;

    // Case 1 - impact while the object is still moving. Solve
    //   distance - closing * t - 0.5 * decel * t^2 = 0
    // using the cancellation-free form t = 2d / (v + sqrt(v^2 + 2 a d)).
    const float root = sqrtNewton(closingSpeedMps * closingSpeedMps + 2.0f * decel * distanceM);
    const float tImpact = 2.0f * distanceM / (closingSpeedMps + root);
    const float tObjectStops = objectSpeed / decel;
    if (tImpact <= tObjectStops) {
        return tImpact < kTtcInfiniteS ? tImpact : kTtcInfiniteS;
    }

    // Case 2 - the object stops first; ego reaches its stopping point.
    if (egoSpeedMps < kMinClosingSpeedMps) {
        return kTtcInfiniteS;
    }
    const float objectStopDistanceM = objectSpeed * objectSpeed / (2.0f * decel);
    const float ttc = (distanceM + objectStopDistanceM) / egoSpeedMps;
    return ttc < kTtcInfiniteS ? ttc : kTtcInfiniteS;
}

}  // namespace aeb
