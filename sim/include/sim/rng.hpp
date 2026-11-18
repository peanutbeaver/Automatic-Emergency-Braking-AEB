#pragma once

#include <stdint.h>

namespace sim {

/// Deterministic xorshift32 PRNG. Produces the same sequence on the host and
/// on the Cortex-M3, which is what makes SIL and PIL results comparable.
class Rng {
public:
    explicit Rng(uint32_t seed) : state_(seed != 0U ? seed : 0x9E3779B9U) {}

    uint32_t next() {
        uint32_t x = state_;
        x ^= x << 13U;
        x ^= x >> 17U;
        x ^= x << 5U;
        state_ = x;
        return x;
    }

    /// Uniform in [0, 1).
    float uniform() { return static_cast<float>(next() >> 8U) * (1.0f / 16777216.0f); }

    /// Approximately standard normal (Irwin-Hall: sum of 12 uniforms - 6).
    /// Needs no sqrt/log, so it works without a math library.
    float gaussian() {
        float sum = 0.0f;
        for (int i = 0; i < 12; ++i) {
            sum += uniform();
        }
        return sum - 6.0f;
    }

private:
    uint32_t state_;
};

}  // namespace sim
