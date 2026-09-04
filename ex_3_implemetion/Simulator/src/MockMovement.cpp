#include <Simulator/MockMovement.h>

#include <UserCommon/AngleUtils.h>

#include <algorithm>
#include <cmath>
#include <mp-units/systems/si/math.h>
#include <stdexcept>

namespace simulator_207610130_215664087 {

using namespace user_common_207610130_215664087;

namespace {

// Small threshold used to remove tiny floating-point errors from trig calculations.
constexpr double kTrigEpsilon = 1.0e-9;

// Replaces very small values with zero.
// Useful for results like cos(90°) that may be 0.0000000001 instead of exactly 0.
[[nodiscard]] double snapped(double value) {
    return std::abs(value) < kTrigEpsilon ? 0.0 : value;
}

} // namespace

// Initializes the movement mock with the GPS, hidden real map,
// and physical radius of the drone.
MockMovement::MockMovement(MockGPS& gps, const common::IMap3D* hidden_map, PhysicalLength drone_radius)
    : gps_(gps), hidden_map_(hidden_map), drone_radius_(drone_radius) {}

// Checks whether the drone collides with an occupied voxel in the hidden real map.
void MockMovement::checkRealMapCollision(const Position3D& position) const {
    // If no real map was provided, collision checking is disabled.
    if (hidden_map_ == nullptr) {
        return;
    }

    // First check the drone's center point.
    if (hidden_map_->atVoxel(position) == common::types::VoxelOccupancy::Occupied) {
        throw std::runtime_error("Drone collided with an occupied voxel on the actual map.");
    }

    const double radius_cm = drone_radius_.force_numerical_value_in(cm);

    // If the drone has a physical radius, also check the space around its center.
    if (radius_cm > 0.0) {
        const double res_cm = hidden_map_->getMapConfig().resolution.force_numerical_value_in(cm);
        // Sampling step is based on map resolution.
        const double step_cm = res_cm > 0.0 ? std::max(res_cm * 0.5, 1.0) : 5.0;
        const double r2 = radius_cm * radius_cm;

         // Sample points inside the drone's spherical volume.
        for (double rx = -radius_cm; rx <= radius_cm; rx += step_cm) {
            for (double ry = -radius_cm; ry <= radius_cm; ry += step_cm) {
                for (double rz = -radius_cm; rz <= radius_cm; rz += step_cm) {
                    // Ignore points outside the drone radius.
                    if (rx * rx + ry * ry + rz * rz <= r2) {
                        const Position3D probe{
                            position.x + rx * x_extent[cm],
                            position.y + ry * y_extent[cm],
                            position.z + rz * z_extent[cm]
                        };
                        if (hidden_map_->atVoxel(probe) == common::types::VoxelOccupancy::Occupied) {
                            throw std::runtime_error("Drone collided with an occupied voxel on the actual map.");
                        }
                    }
                }
            }
        }
    }
}

// Rotates the drone left or right and updates the heading stored in the GPS.
common::types::MovementResult MockMovement::rotate(
    common::types::RotationDirection direction,
    HorizontalAngle angle) {

    const Orientation current = gps_.heading();

    // Left rotation is positive, right rotation is negative.
    const HorizontalAngle signed_angle =
        (direction == common::types::RotationDirection::Left) ? angle : -angle;

    // Calculate and normalize the new heading to the valid degree range.
    const double next_degrees =
        normalizeDegrees(horizontalDegrees(current.horizontal + signed_angle));

    gps_.setHeading(Orientation{next_degrees * horizontal_angle[deg], current.altitude});
    return common::types::MovementResult{true, {}};
}

// Moves the drone forward or backward according to its current heading.
common::types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D current_pos = gps_.position();
    const Orientation heading = gps_.heading();
    const double distance_cm = distance.force_numerical_value_in(cm);

    // Convert movement distance and heading into X and Y changes.
    const double dx_cm =
        distance_cm * snapped(si::cos(heading.horizontal).force_numerical_value_in(mp::one));
    const double dy_cm =
        distance_cm * snapped(si::sin(heading.horizontal).force_numerical_value_in(mp::one));

    // Calculate the final destination.
    Position3D new_pos{
        current_pos.x + dx_cm * x_extent[cm],
        current_pos.y + dy_cm * y_extent[cm],
        current_pos.z,
    };

    // Choose how frequently to check the movement path for collisions.
    double step_size_cm = 1.0;
    if (hidden_map_ != nullptr) {
        step_size_cm = std::max(0.5, hidden_map_->getMapConfig().resolution.force_numerical_value_in(cm) * 0.5);
    }
    const std::size_t num_samples = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(std::abs(distance_cm) / step_size_cm)));

    // Check points along the entire movement path,
    // not only the final destination.
    for (std::size_t i = 1; i <= num_samples; ++i) {
        const double frac = static_cast<double>(i) / static_cast<double>(num_samples);
        Position3D sample_pos{
            current_pos.x + frac * dx_cm * x_extent[cm],
            current_pos.y + frac * dy_cm * y_extent[cm],
            current_pos.z,
        };
        checkRealMapCollision(sample_pos);
    }

    // Only update the GPS after the full path was confirmed safe.
    gps_.setPosition(new_pos);
    return common::types::MovementResult{true, {}};
}

// Moves the drone vertically up or down.
common::types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D current_pos = gps_.position();
    const double distance_cm = distance.force_numerical_value_in(cm);

    // Calculate the final vertical position.
    Position3D new_pos{
        current_pos.x,
        current_pos.y,
        current_pos.z + distance_cm * z_extent[cm],
    };

    // Choose how frequently to check the movement path for collisions.
    double step_size_cm = 1.0;
    if (hidden_map_ != nullptr) {
        step_size_cm = std::max(0.5, hidden_map_->getMapConfig().resolution.force_numerical_value_in(cm) * 0.5);
    }
    const std::size_t num_samples = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(std::abs(distance_cm) / step_size_cm)));

    // Check the entire vertical path for collisions.
    for (std::size_t i = 1; i <= num_samples; ++i) {
        const double frac = static_cast<double>(i) / static_cast<double>(num_samples);
        Position3D sample_pos{
            current_pos.x,
            current_pos.y,
            current_pos.z + frac * distance_cm * z_extent[cm],
        };
        checkRealMapCollision(sample_pos);
    }

    // Update the GPS only after the movement was confirmed safe.
    gps_.setPosition(new_pos);
    return common::types::MovementResult{true, {}};
}

} // namespace simulator_207610130_215664087
