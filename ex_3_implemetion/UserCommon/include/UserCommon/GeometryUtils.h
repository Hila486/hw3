#pragma once

#include <Common/Units.h>
#include <UserCommon/AngleUtils.h>
#include <mp-units/systems/si/math.h>

namespace user_common_207610130_215664087 {

using namespace common;

[[nodiscard]] inline Orientation absoluteBeamOrientation(
    const Orientation& drone_heading,
    const Orientation& relative_beam) {
    return Orientation{
        relative_beam.horizontal + drone_heading.horizontal,
        relative_beam.altitude + drone_heading.altitude,
    };
}

[[nodiscard]] inline Position3D pointAlongBeam(
    const Position3D& origin,
    const Orientation& beam_orientation,
    PhysicalLength distance) {

    const auto cos_altitude = si::cos(beam_orientation.altitude);
    const auto dx = cos_altitude * si::cos(beam_orientation.horizontal);
    const auto dy = cos_altitude * si::sin(beam_orientation.horizontal);
    const auto dz = si::sin(beam_orientation.altitude);

    const double distance_cm = distance.force_numerical_value_in(cm);
    const double dir_x = dx.force_numerical_value_in(mp::one);
    const double dir_y = dy.force_numerical_value_in(mp::one);
    const double dir_z = dz.force_numerical_value_in(mp::one);

    return Position3D{
        origin.x + dir_x * distance_cm * x_extent[cm],
        origin.y + dir_y * distance_cm * y_extent[cm],
        origin.z + dir_z * distance_cm * z_extent[cm],
    };
}

} // namespace user_common_207610130_215664087
