#pragma once

#include <stdint.h>

namespace aeb {

/// Calibration parameters. Defaults implement docs/REQUIREMENTS.md.
struct Config {
    // Activation window (REQ-AEB-010)
    float minActiveSpeedMps = 5.0f / 3.6f;
    float maxActiveSpeedMps = 130.0f / 3.6f;

    // TTC thresholds (REQ-AEB-020..022, REQ-AEB-030)
    float ttcWarningS = 2.6f;
    float ttcPartialBrakeS = 1.6f;
    float ttcFullBrakeS = 1.0f;
    float ttcHysteresisS = 0.3f;
    uint8_t confirmCycles = 2;  // REQ-AEB-024

    // Object braking is only taken into account below this acceleration, so
    // tracker noise on a steady lead vehicle cannot cause warnings (REQ-AEB-025).
    float objectBrakingThresholdMps2 = -1.0f;

    // Brake levels and ramps (REQ-AEB-021, -022, -033, -040)
    float partialDecelMps2 = 4.0f;
    float fullDecelMps2 = 9.0f;
    float holdDecelMps2 = 4.0f;
    float brakeRiseRateMps3 = 50.0f;
    float brakeReleaseRateMps3 = 25.0f;

    // Standstill hold (REQ-AEB-040, -041)
    float standstillSpeedMps = 0.1f;
    uint32_t holdTimeMs = 2000U;

    // Driver override (REQ-AEB-050)
    float kickdownPct = 90.0f;

    // Sensor monitoring (REQ-AEB-060, -061)
    uint32_t sensorTimeoutMs = 200U;
    float maxPlausibleDistanceM = 250.0f;
    float maxPlausibleClosingSpeedMps = 70.0f;
    float maxPlausibleAccelMps2 = 15.0f;
};

}  // namespace aeb
