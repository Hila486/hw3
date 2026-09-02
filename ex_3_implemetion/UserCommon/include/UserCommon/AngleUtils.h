#pragma once

#include <Common/Units.h>
#include <cmath>

namespace user_common_207610130_215664087 {

using namespace common;

constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] inline double horizontalDegrees(HorizontalAngle angle) {
    return angle.force_numerical_value_in(deg);
}

[[nodiscard]] inline double altitudeDegrees(AltitudeAngle angle) {
    return angle.force_numerical_value_in(deg);
}

[[nodiscard]] inline double physicalCm(PhysicalLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] inline double xCm(XLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] inline double yCm(YLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] inline double zCm(ZLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] inline double normalizeDegrees(double degrees) {
    double normalized = std::fmod(degrees, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized;
}

[[nodiscard]] inline double signedAngleDeltaDegrees(double from_degrees, double to_degrees) {
    double delta = normalizeDegrees(to_degrees) - normalizeDegrees(from_degrees);
    if (delta > 180.0) {
        delta -= 360.0;
    }
    if (delta < -180.0) {
        delta += 360.0;
    }
    return delta;
}

[[nodiscard]] inline double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

} // namespace user_common_207610130_215664087
