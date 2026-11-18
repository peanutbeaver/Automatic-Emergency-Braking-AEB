#include <gtest/gtest.h>

#include <cmath>

#include "aeb/ttc.hpp"

using aeb::computeTtc;
using aeb::kTtcInfiniteS;

TEST(Ttc, DistanceDividedByClosingSpeed) {
    EXPECT_FLOAT_EQ(computeTtc(20.0f, 10.0f), 2.0f);
    EXPECT_FLOAT_EQ(computeTtc(5.0f, 20.0f), 0.25f);
}

TEST(Ttc, InfiniteWhenNotApproaching) {
    EXPECT_EQ(computeTtc(20.0f, 0.0f), kTtcInfiniteS);
    EXPECT_EQ(computeTtc(20.0f, -5.0f), kTtcInfiniteS);
}

TEST(Ttc, InfiniteBelowMinimumClosingSpeed) {
    EXPECT_EQ(computeTtc(20.0f, 0.05f), kTtcInfiniteS);
}

TEST(Ttc, ZeroWhenAlreadyInContact) {
    EXPECT_EQ(computeTtc(0.0f, 5.0f), 0.0f);
    EXPECT_EQ(computeTtc(-1.0f, 5.0f), 0.0f);
}

TEST(SqrtNewton, MatchesStdSqrt) {
    for (float x : {0.0001f, 0.25f, 1.0f, 2.0f, 144.0f, 9900.0f, 1.0e6f}) {
        EXPECT_NEAR(aeb::sqrtNewton(x), std::sqrt(x), std::sqrt(x) * 1e-6f) << x;
    }
    EXPECT_EQ(aeb::sqrtNewton(0.0f), 0.0f);
    EXPECT_EQ(aeb::sqrtNewton(-4.0f), 0.0f);
}

using aeb::computeTtcWithObjectBraking;
constexpr float kThreshold = -1.0f;

TEST(EnhancedTtc, FallsBackToConstantSpeedWhenObjectIsNotBraking) {
    EXPECT_FLOAT_EQ(computeTtcWithObjectBraking(20.0f, 15.0f, 10.0f, 0.0f, kThreshold), 2.0f);
    EXPECT_FLOAT_EQ(computeTtcWithObjectBraking(20.0f, 15.0f, 10.0f, -0.5f, kThreshold), 2.0f);
}

TEST(EnhancedTtc, ImpactWhileObjectIsStillBraking) {
    // Equal speeds (14 m/s), lead brakes at 6 m/s^2, 12 m gap:
    // 0.5 * 6 * t^2 = 12  ->  t = 2 s (lead would stop only after 2.33 s).
    EXPECT_NEAR(computeTtcWithObjectBraking(12.0f, 14.0f, 0.0f, -6.0f, kThreshold), 2.0f, 1e-4f);
}

TEST(EnhancedTtc, ObjectStopsBeforeImpact) {
    // Ego 10 m/s, lead 4 m/s braking at 8 m/s^2 -> lead stops after 0.5 s / 1 m.
    // Ego covers 30 + 1 = 31 m in 3.1 s.
    EXPECT_NEAR(computeTtcWithObjectBraking(30.0f, 10.0f, 6.0f, -8.0f, kThreshold), 3.1f, 1e-4f);
}

TEST(EnhancedTtc, StoppedEgoNeverCollidesWithStoppedObject) {
    EXPECT_EQ(computeTtcWithObjectBraking(5.0f, 0.0f, 0.0f, -3.0f, kThreshold), kTtcInfiniteS);
}

TEST(Ttc, IsUsableAtCompileTime) {
    static_assert(computeTtc(30.0f, 15.0f) == 2.0f, "TTC must be constexpr");
    SUCCEED();
}
