#include <UserCommon/MockMovement.h>

#include <cmath>
#include <mp-units/systems/si/math.h>
#include <stdexcept>

namespace user_common_207610130_215664087 {

namespace {

constexpr double kTrigEpsilon = 1.0e-9;

/// Extracts degree value from HorizontalAngle type.
[[nodiscard]] double horizontalDegrees(HorizontalAngle angle) {
    return angle.force_numerical_value_in(deg);
}

/// Normalizes degree angle to [0, 360) range.
[[nodiscard]] double normalizeDegrees(double degrees) {
    double normalized = std::fmod(degrees, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized;
}

/// Snaps small trigonometric values close to 0 within epsilon.
[[nodiscard]] double snapped(double value) {
    return std::abs(value) < kTrigEpsilon ? 0.0 : value;
}

} // namespace

MockMovement::MockMovement(MockGPS& gps, const common::IMap3D* hidden_map)
    : gps_(gps), hidden_map_(hidden_map) {}

void MockMovement::checkRealMapCollision(const Position3D& position) const {
    if (hidden_map_ != nullptr) {
        if (hidden_map_->atVoxel(position) == common::types::VoxelOccupancy::Occupied) {
            throw std::runtime_error("Drone collided with an occupied voxel on the actual map.");
        }
    }
}

/**
 * @brief Rotates drone left or right and updates simulated GPS heading.
 */
common::types::MovementResult MockMovement::rotate(common::types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == common::types::RotationDirection::Left) ? angle : -angle;

    const double next_degrees =
        normalizeDegrees(horizontalDegrees(current.horizontal + signed_angle));

    gps_.setHeading(Orientation{next_degrees * horizontal_angle[deg], current.altitude});
    return common::types::MovementResult{true, {}};
}

/**
 * @brief Advances drone forward in 2D horizontal direction and updates simulated GPS position.
 */
common::types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D current_pos = gps_.position();
    const Orientation heading = gps_.heading();
    const double distance_cm = distance.force_numerical_value_in(cm);

    const double dx_cm =
        distance_cm * snapped(si::cos(heading.horizontal).force_numerical_value_in(mp::one));
    const double dy_cm =
        distance_cm * snapped(si::sin(heading.horizontal).force_numerical_value_in(mp::one));

    Position3D new_pos{
        current_pos.x + dx_cm * x_extent[cm],
        current_pos.y + dy_cm * y_extent[cm],
        current_pos.z,
    };

    checkRealMapCollision(new_pos);

    gps_.setPosition(new_pos);
    return common::types::MovementResult{true, {}};
}

/**
 * @brief Elevates drone vertically and updates simulated GPS position.
 */
common::types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D current_pos = gps_.position();
    const double distance_cm = distance.force_numerical_value_in(cm);

    Position3D new_pos{
        current_pos.x,
        current_pos.y,
        current_pos.z + distance_cm * z_extent[cm],
    };

    checkRealMapCollision(new_pos);

    gps_.setPosition(new_pos);
    return common::types::MovementResult{true, {}};
}

} // namespace user_common_207610130_215664087
