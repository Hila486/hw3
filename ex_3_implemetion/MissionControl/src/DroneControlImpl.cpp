#include <MissionControl/DroneControlImpl.h>
#include <MissionControl/ScanResultToVoxels.h>

#include <UserCommon/AngleUtils.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <utility>

namespace mission_control_207610130_215664087 {

using namespace user_common_207610130_215664087;

namespace {
// Minimum allowed sampling step to avoid zero or extremely small values.
constexpr double kMinimumPositiveStepCm = 1.0e-6;
// Default safety step if the map resolution is invalid.
constexpr double kFallbackSafetyStepCm = 1.0;

// Returns the squared distance between two 3D positions in centimeters.
// Squared distance is enough for comparisons and avoids an unnecessary sqrt.
[[nodiscard]] double squaredDistanceCm(const Position3D& a, const Position3D& b) {
    const double dx_cm = xCm(a.x) - xCm(b.x);
    const double dy_cm = yCm(a.y) - yCm(b.y);
    const double dz_cm = zCm(a.z) - zCm(b.z);
    return dx_cm * dx_cm + dy_cm * dy_cm + dz_cm * dz_cm;
}

// Checks whether a point is inside a sphere.
[[nodiscard]] bool isInsideSphere(const Position3D& position,
                                  const Position3D& center,
                                  double radius_cm) {
    return squaredDistanceCm(position, center) <= radius_cm * radius_cm;
}

// Creates a DroneStepResult telling the simulator to continue.
[[nodiscard]] types::DroneStepResult continueResult(std::string message = {}) {
    return types::DroneStepResult{types::DroneStepStatus::Continue, std::move(message)};
}
// Creates a DroneStepResult telling the simulator that mapping is complete.
[[nodiscard]] types::DroneStepResult completedResult(std::string message = {}) {
    return types::DroneStepResult{types::DroneStepStatus::Completed, std::move(message)};
}
// Creates a DroneStepResult representing an error.
[[nodiscard]] types::DroneStepResult errorResult(std::string message) {
    return types::DroneStepResult{types::DroneStepStatus::Error, std::move(message)};
}

} // namespace

// Initializes the controller with the drone configuration, mission,
// simulated hardware interfaces, output map and mapping algorithm.
DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}


// Executes one step of the mapping mission.
types::DroneStepResult DroneControlImpl::step() {
    try {
        // The space currently occupied by the drone must be empty.
        markCurrentDroneBodyAsEmpty();

        // Give the mapping algorithm the most recent lidar scan, if one exists.
        const types::LidarScanResult* latest_scan =
            latest_scan_ ? &(*latest_scan_) : nullptr;
         // Ask the mapping algorithm what the drone should do next.
        const types::MappingStepCommand command =
            mapping_algorithm_.nextStep(state(), latest_scan);

        // A scan is consumed by one algorithm step.
        latest_scan_.reset();

        // Check whether the algorithm finished mapping.
        if (command.status == types::AlgorithmStatus::Finished) {
            return completedResult("Mapping algorithm finished.");
        }
        if (command.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            return completedResult("Mapping algorithm finished with unmappable voxels.");
        }
        // Execute the requested movement, if there is one.
        if (command.movement) {
            std::string movement_message;
            if (!executeMovementCommand(*command.movement, movement_message)) {
                ++step_index_;
                return continueResult(std::move(movement_message));
            }
        }
        
        // Execute a lidar scan if the algorithm requested one.
        if (command.scan_orientation) {
            latest_scan_ = lidar_.scan(*command.scan_orientation);
            // Convert the lidar scan into voxel information
            // and update the drone's output map.
            ScanResultToVoxels::applyToMap(
                output_map_,
                gps_.position(),
                gps_.heading(),
                *latest_scan_,
                lidar_.config());
        }

        ++step_index_;
        return continueResult();
    } catch (const std::exception& exception) {
         // Prevent exceptions from crashing the simulation.
        return errorResult(std::string("DroneControlImpl::step failed: ") + exception.what());
    }
}

// Returns the current state of the drone for the mapping algorithm.
types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

// Validates and executes one movement command.
bool DroneControlImpl::executeMovementCommand(const types::MovementCommand& command,
                                              std::string& message) {
    // Hover means no movement is required.
    if (command.type == types::MovementCommandType::Hover) {
        return true;
    }

    // Make sure the requested movement does not exceed
    // the drone's configured movement limits.
    if (!isMovementCommandWithinLimits(command, message)) {
        return false;
    }

    const Position3D start = gps_.position();
    const Orientation heading = gps_.heading();
    // Calculate where the movement would place the drone.
    const Position3D target = targetPositionForMovement(start, heading, command);

    // Before moving, make sure the entire volume that the drone
    // will travel through is already known to be empty.
    if (command.type != types::MovementCommandType::Rotate &&
        !isSweptSphereKnownEmpty(start, target)) {
        message = "Movement rejected: swept drone volume is not fully known empty.";
        return false;
    }

    types::MovementResult result;
    // Send the movement to the correct hardware interface function.
    switch (command.type) {
        case types::MovementCommandType::Rotate:
            result = movement_.rotate(command.rotation, command.angle);
            break;
        case types::MovementCommandType::Advance:
            result = movement_.advance(command.distance);
            break;
        case types::MovementCommandType::Elevate:
            result = movement_.elevate(command.distance);
            break;
        case types::MovementCommandType::Hover:
            return true;
    }
    // Handle hardware movement failure.
    if (!result) {
        message = result.message.empty() ? "Movement command failed." : result.message;
        return false;
    }

    return true;
}

// Checks whether a movement command stays within the drone's configured limits.
bool DroneControlImpl::isMovementCommandWithinLimits(
    const types::MovementCommand& command,
    std::string& message) const {
    switch (command.type) {
        case types::MovementCommandType::Hover:
            return true;
        case types::MovementCommandType::Rotate: {
            const double angle_degrees = std::abs(horizontalDegrees(command.angle));
            const double max_degrees = horizontalDegrees(drone_.max_rotate);
            if (angle_degrees > max_degrees) {
                message = "Movement rejected: rotate command exceeds max_rotate.";
                return false;
            }
            return true;
        }
        case types::MovementCommandType::Advance: {
            const double distance_cm = physicalCm(command.distance);
            if (std::abs(distance_cm) > physicalCm(drone_.max_advance)) {
                message = "Movement rejected: advance command exceeds max_advance.";
                return false;
            }
            return true;
        }
        case types::MovementCommandType::Elevate: {
            const double distance_cm = physicalCm(command.distance);
            if (std::abs(distance_cm) > physicalCm(drone_.max_elevate)) {
                message = "Movement rejected: elevate command exceeds max_elevate.";
                return false;
            }
            return true;
        }
    }

    message = "Movement rejected: unsupported movement command type.";
    return false;
}

// Calculates the expected target position for a movement command.
// This does not actually move the drone.
Position3D DroneControlImpl::targetPositionForMovement(
    const Position3D& start,
    const Orientation& heading,
    const types::MovementCommand& command) const {
    switch (command.type) {
        case types::MovementCommandType::Hover:
        case types::MovementCommandType::Rotate:
            return start;
        case types::MovementCommandType::Advance: {
            const double distance_cm = physicalCm(command.distance);
            const double heading_radians = degreesToRadians(horizontalDegrees(heading.horizontal));
            const double dx_cm = distance_cm * std::cos(heading_radians);
            const double dy_cm = distance_cm * std::sin(heading_radians);
            return Position3D{
                start.x + dx_cm * x_extent[cm],
                start.y + dy_cm * y_extent[cm],
                start.z
            };
        }
        case types::MovementCommandType::Elevate:
            return Position3D{
                start.x,
                start.y,
                start.z + command.distance.force_numerical_value_in(cm) * z_extent[cm]
            };
    }

    return start;
}

// Checks that the entire volume swept by the drone while moving
// is already marked as Empty in the output map.
bool DroneControlImpl::isSweptSphereKnownEmpty(const Position3D& start,
                                              const Position3D& target) const {
    const double radius_cm = physicalCm(drone_.radius);
    const double resolution_cm = physicalCm(output_map_.getMapConfig().resolution);
    // Determine how frequently to sample along the movement path.
    const double safety_step_cm =
        resolution_cm > 0.0 ? std::max(resolution_cm * 0.5, kMinimumPositiveStepCm)
                            : kFallbackSafetyStepCm;

    const double start_x = xCm(start.x);
    const double start_y = yCm(start.y);
    const double start_z = zCm(start.z);

    const double target_x = xCm(target.x);
    const double target_y = yCm(target.y);
    const double target_z = zCm(target.z);

    const double dx = target_x - start_x;
    const double dy = target_y - start_y;
    const double dz = target_z - start_z;
    
    // Total length of the requested movement.
    const double total_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    const std::size_t sample_count =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(total_distance / safety_step_cm)));

    // Step used to sample points inside the drone's sphere.
    const double sample_radius_step_cm =
        resolution_cm > 0.0 ? std::max(resolution_cm * 0.5, kMinimumPositiveStepCm)
                            : kFallbackSafetyStepCm;

    // Sample positions along the entire movement path.
    for (std::size_t step = 0; step <= sample_count; ++step) {
        const double progress = static_cast<double>(step) / static_cast<double>(sample_count);
        // Current center of the drone along the path.
        const Position3D center{
            (start_x + progress * dx) * x_extent[cm],
            (start_y + progress * dy) * y_extent[cm],
            (start_z + progress * dz) * z_extent[cm]
        };
        // Check points throughout the drone's spherical body.
        for (double rx = -radius_cm; rx <= radius_cm; rx += sample_radius_step_cm) {
            for (double ry = -radius_cm; ry <= radius_cm; ry += sample_radius_step_cm) {
                for (double rz = -radius_cm; rz <= radius_cm; rz += sample_radius_step_cm) {
                    const Position3D point{
                        center.x + rx * x_extent[cm],
                        center.y + ry * y_extent[cm],
                        center.z + rz * z_extent[cm]
                    };
                    
                    // Ignore points outside the spherical drone body.
                    if (!isInsideSphere(point, center, radius_cm)) {
                        continue;
                    }

                    // The drone may only move through voxels
                    // that are already known to be empty.
                    if (output_map_.atVoxel(point) != types::VoxelOccupancy::Empty) {
                        return false;
                    }
                }
            }
        }
    }

    return true;
}


// Marks the volume currently occupied by the drone as Empty.
// Since the drone is physically there, that volume is known to contain no obstacle.
void DroneControlImpl::markCurrentDroneBodyAsEmpty() {
    const Position3D center = gps_.position();
    const double radius_cm = physicalCm(drone_.radius);
    const double resolution_cm = physicalCm(output_map_.getMapConfig().resolution);
    const double step_cm =
        resolution_cm > 0.0 ? std::max(resolution_cm * 0.5, kMinimumPositiveStepCm)
                            : kFallbackSafetyStepCm;
                            
    // Sample the spherical volume occupied by the drone.
    for (double rx = -radius_cm; rx <= radius_cm; rx += step_cm) {
        for (double ry = -radius_cm; ry <= radius_cm; ry += step_cm) {
            for (double rz = -radius_cm; rz <= radius_cm; rz += step_cm) {
                const Position3D point{
                    center.x + rx * x_extent[cm],
                    center.y + ry * y_extent[cm],
                    center.z + rz * z_extent[cm]
                };

                if (isInsideSphere(point, center, radius_cm)) {
                    output_map_.set(point, types::VoxelOccupancy::Empty);
                }
            }
        }
    }
}

} // namespace mission_control_207610130_215664087
