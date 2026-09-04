#include <Algorithm/MappingAlgorithmImpl_207610130_215664087.h>

#include <Common/IMap3D.h>
#include <Common/MappingAlgorithmRegistration.h>
#include <UserCommon/AngleUtils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

using algorithm_207610130_215664087::MappingAlgorithmImpl_207610130_215664087;
using namespace user_common_207610130_215664087;

// Registers this algorithm so the simulator can load it dynamically.
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_207610130_215664087);

namespace algorithm_207610130_215664087 {

namespace {

// Small tolerances used for floating-point comparisons.
constexpr double kEpsilonDegrees = 1.0e-6;
constexpr double kPositionEpsilonCm = 1.0e-6;
// Minimum exploration step size.
constexpr double kMinimumStepCm = 1.0;
// Each cell is scanned in 6 directions: front, right, back, left, up, down.
constexpr std::size_t kScanDirectionCount = 6;
// Safety limit to prevent endless failed movement attempts.
constexpr std::size_t kMaxFailedTranslations = 20000;

// Checks whether a physical distance is positive
[[nodiscard]] bool isPositive(PhysicalLength length) {
    return physicalCm(length) > 0.0;
}

// Checks whether an angle is positive.
[[nodiscard]] bool isPositiveAngle(HorizontalAngle angle) {
    return angle > 0.0 * horizontal_angle[deg];
}

// Returns true if the drone should not enter this type of voxel.
[[nodiscard]] bool isBlockedOccupancy(common::types::VoxelOccupancy occupancy) {
    return occupancy == common::types::VoxelOccupancy::Occupied ||
           occupancy == common::types::VoxelOccupancy::OutOfBounds ||
           occupancy == common::types::VoxelOccupancy::PotentiallyOccupied;
}

// Calculates a new position after moving horizontally in a given direction.
[[nodiscard]] Position3D offsetHorizontal(
    const Position3D& position,
    double heading_degrees,
    PhysicalLength distance) {
    const double radians = degreesToRadians(heading_degrees);
    const double distance_cm = physicalCm(distance);

    return Position3D{
        position.x + distance_cm * std::cos(radians) * x_extent[cm],
        position.y + distance_cm * std::sin(radians) * y_extent[cm],
        position.z,
    };
}

// Calculates a new position after moving vertically.
[[nodiscard]] Position3D offsetVertical(
    const Position3D& position,
    PhysicalLength distance) {
    const double distance_cm = physicalCm(distance);

    return Position3D{
        position.x,
        position.y,
        position.z + distance_cm * z_extent[cm],
    };
}

// Wraps a movement request inside a MappingStepCommand.
[[nodiscard]] common::types::MappingStepCommand movementCommand(common::types::MovementCommand movement) {
    common::types::MappingStepCommand command;
    command.movement = movement;
    command.status = common::types::AlgorithmStatus::Working;
    return command;
}

} // namespace

// Ordering operator so GridCell can be stored inside std::set.
bool MappingAlgorithmImpl_207610130_215664087::GridCell::operator<(const GridCell& other) const {
    if (x != other.x) {
        return x < other.x;
    }
    if (y != other.y) {
        return y < other.y;
    }
    return z < other.z;
}

// Checks whether two grid cells represent the same location.
bool MappingAlgorithmImpl_207610130_215664087::GridCell::operator==(const GridCell& other) const {
    return x == other.x && y == other.y && z == other.z;
}


// Main algorithm state machine.
// Scans the current cell, explores new DFS neighbors,
// and backtracks when no new neighbor is available.
common::types::MappingStepCommand MappingAlgorithmImpl_207610130_215664087::nextStep(
    const common::types::DroneState& state,
    const common::types::LidarScanResult* latest_scan) {

    // Convert current physical position into a discrete DFS grid cell.
    const GridCell current_cell = cellFromPosition(state.position);

     // Check whether the previously requested movement succeeded.
    handlePendingTranslationResult(current_cell);

    // Initialize DFS state on the first call.
    initializeAtCurrentCell(current_cell, state.position);

     // Safety stop after too many failed movement attempts.
    if (failed_translation_count_ >= kMaxFailedTranslations) {
        return finishCommand();
    }

     // Start a fresh scan sequence after entering a new cell.
    if (scan_cell_ != current_cell) {
        resetScanSequenceForCell(current_cell);
    }

     // If we previously requested a scan, wait for its result.
    if (waiting_for_scan_result_) {
        if (latest_scan == nullptr) {
            return scanCommand(*pending_scan_index_);
        }

        waiting_for_scan_result_ = false;
        ++next_scan_index_;
        pending_scan_index_.reset();
    }

     // Scan all six directions before trying to move.
    if (next_scan_index_ < kScanDirectionCount) {
        return scanCommand(next_scan_index_);
    }

    // Find an unexplored neighboring cell.
    if (!pending_target_) {
        pending_target_ = findExplorationTarget(state, current_cell);
    }

    // Move toward the chosen target.
    if (pending_target_) {
        return commandForTarget(state, current_cell, *pending_target_);
    }

    // No target and no previous DFS node means exploration is finished.
    if (dfs_path_.size() <= 1) {
        return finishCommand();
    }

     // No new neighbor exists, so return to the previous DFS node.
    const PathNode previous_node = dfs_path_[dfs_path_.size() - 2];

    const double delta_x_cm = xCm(previous_node.position.x) - xCm(state.position.x);
    const double delta_y_cm = yCm(previous_node.position.y) - yCm(state.position.y);
    const double delta_z_cm = zCm(previous_node.position.z) - zCm(state.position.z);

    // Determine whether backtracking is vertical or horizontal.
    const bool backtrack_is_vertical =
        std::abs(delta_x_cm) <= kPositionEpsilonCm &&
        std::abs(delta_y_cm) <= kPositionEpsilonCm &&
        std::abs(delta_z_cm) > kPositionEpsilonCm;

    const double backtrack_distance_cm =
        backtrack_is_vertical ? std::abs(delta_z_cm) : std::hypot(delta_x_cm, delta_y_cm);

    pending_target_ = NavigationTarget{
        previous_node,
        true,
        backtrack_is_vertical,
        horizontalDegrees(state.heading.horizontal),
        backtrack_distance_cm * cm,
    };

    return commandForTarget(state, current_cell, *pending_target_);
}

// Converts a real-world position into a discrete DFS grid coordinate.
MappingAlgorithmImpl_207610130_215664087::GridCell MappingAlgorithmImpl_207610130_215664087::cellFromPosition(
    const Position3D& position) const {

    const common::types::MapConfig map_config = output_map_.getMapConfig();
    const double step_cm = std::max(kMinimumStepCm, physicalCm(planningStep()));

    return GridCell{
        static_cast<int>(
            std::llround((xCm(position.x) - xCm(map_config.boundaries.min_x)) / step_cm)),
        static_cast<int>(
            std::llround((yCm(position.y) - yCm(map_config.boundaries.min_y)) / step_cm)),
        static_cast<int>(
            std::llround((zCm(position.z) - zCm(map_config.boundaries.min_height)) / step_cm)),
    };
}

// Determines the distance between neighboring DFS cells.
// It respects map resolution and the drone's movement limits.
PhysicalLength MappingAlgorithmImpl_207610130_215664087::planningStep() const {

    const common::types::MapConfig map_config = output_map_.getMapConfig();
    double step_cm = physicalCm(map_config.resolution);

    if (step_cm <= 0.0) {
        step_cm = physicalCm(mission_config_.gps_resolution);
    }
    if (step_cm <= 0.0) {
        step_cm = kMinimumStepCm;
    }

    // Never request a step larger than the drone can advance.
    if (isPositive(drone_config_.max_advance)) {
        step_cm = std::min(step_cm, physicalCm(drone_config_.max_advance));
    }
    // Also respect the maximum vertical movement.
    if (isPositive(drone_config_.max_elevate)) {
        step_cm = std::min(step_cm, physicalCm(drone_config_.max_elevate));
    }

    return std::max(kMinimumStepCm, step_cm) * cm;
}

// Checks whether the entire drone body can safely fit
// around the given center position according to the known output map.
bool MappingAlgorithmImpl_207610130_215664087::isLegalDroneCenter(const Position3D& position) const {
    if (!output_map_.isInBounds(position)) {
        return false;
    }

    if (isBlockedOccupancy(output_map_.atVoxel(position))) {
        return false;
    }

    const double radius_cm = physicalCm(drone_config_.radius);
    // A zero-radius drone only needs its center checked.
    if (radius_cm <= 0.0) {
        return true;
    }

    const double sample_step_cm =
        std::max(kMinimumStepCm, std::min(radius_cm, physicalCm(planningStep())));

    // Sample points throughout the drone's spherical body.
    for (double dz_cm = -radius_cm; dz_cm <= radius_cm; dz_cm += sample_step_cm) {
        for (double dy_cm = -radius_cm; dy_cm <= radius_cm; dy_cm += sample_step_cm) {
            for (double dx_cm = -radius_cm; dx_cm <= radius_cm; dx_cm += sample_step_cm) {

                const double distance_squared =
                    dx_cm * dx_cm + dy_cm * dy_cm + dz_cm * dz_cm;
                
                // Ignore points outside the sphere.
                if (distance_squared > radius_cm * radius_cm) {
                    continue;
                }

                const Position3D checked_position{
                    position.x + dx_cm * x_extent[cm],
                    position.y + dy_cm * y_extent[cm],
                    position.z + dz_cm * z_extent[cm],
                };

                // Reject if any part of the drone would be blocked.
                if (!output_map_.isInBounds(checked_position) ||
                    isBlockedOccupancy(output_map_.atVoxel(checked_position))) {
                    return false;
                }
            }
        }
    }

    return true;
}


// Searches neighboring cells in DFS order and returns
// the first legal, unvisited target.
std::optional<MappingAlgorithmImpl_207610130_215664087::NavigationTarget>
MappingAlgorithmImpl_207610130_215664087::findExplorationTarget(
    const common::types::DroneState& state,
    const GridCell& current_cell) const {

    struct CandidateDirection {
        double relative_heading_degrees = 0.0;
        bool vertical = false;
        int vertical_sign = 0;
    };

    const PhysicalLength step = planningStep();
    const double current_heading_degrees = horizontalDegrees(state.heading.horizontal);

    // Exploration order:
    // forward, right, backward, left, up, down.
    const std::array<CandidateDirection, 6> directions{{
        CandidateDirection{0.0, false, 0},
        CandidateDirection{-90.0, false, 0},
        CandidateDirection{180.0, false, 0},
        CandidateDirection{90.0, false, 0},
        CandidateDirection{0.0, true, 1},
        CandidateDirection{0.0, true, -1},
    }};

    for (const CandidateDirection& direction : directions) {
        const double desired_heading_degrees =
            normalizeDegrees(current_heading_degrees + direction.relative_heading_degrees);
        
         // Calculate the candidate physical position.
        const Position3D candidate_position =
            direction.vertical
                ? offsetVertical(state.position, direction.vertical_sign * step)
                : offsetHorizontal(state.position, desired_heading_degrees, step);
        const GridCell candidate_cell = cellFromPosition(candidate_position);

        // Skip cells that were already visited, failed,
        // unchanged, or unsafe.
        if (candidate_cell == current_cell ||
            visited_cells_.find(candidate_cell) != visited_cells_.end() ||
            attempted_cells_.find(candidate_cell) != attempted_cells_.end() ||
            !isLegalDroneCenter(candidate_position)) {
            continue;
        }

        return NavigationTarget{
            PathNode{candidate_cell, candidate_position},
            false,
            direction.vertical,
            desired_heading_degrees,
            step,
        };
    }

    return std::nullopt;
}

// Turns a target into the next movement command.
// Horizontal targets may require rotation first, then advance.
common::types::MappingStepCommand MappingAlgorithmImpl_207610130_215664087::commandForTarget(
    const common::types::DroneState& state,
    const GridCell& current_cell,
    const NavigationTarget& target) {

     // Vertical target: directly elevate.
    if (target.vertical) {
        const double delta_z_cm = zCm(target.node.position.z) - zCm(state.position.z);
        common::types::MovementCommand movement;
        movement.type = common::types::MovementCommandType::Elevate;
        movement.distance = delta_z_cm * cm;

        pending_translation_ = target;
        return movementCommand(movement);
    }

     // Calculate horizontal direction and distance to the target.
    const double dx_cm = xCm(target.node.position.x) - xCm(state.position.x);
    const double dy_cm = yCm(target.node.position.y) - yCm(state.position.y);
    const double horizontal_distance_cm = std::hypot(dx_cm, dy_cm);
    const double desired_heading_degrees =
        horizontal_distance_cm > 0.0
            ? normalizeDegrees(std::atan2(dy_cm, dx_cm) * 180.0 / kPi)
            : target.desired_heading_degrees;
    const double current_heading_degrees = horizontalDegrees(state.heading.horizontal);
    const double turn_degrees =
        signedAngleDeltaDegrees(current_heading_degrees, desired_heading_degrees);

    // Rotate first if the drone is not facing the target.
    if (std::abs(turn_degrees) > kEpsilonDegrees) {
        double rotation_degrees = std::abs(turn_degrees);
        if (isPositiveAngle(drone_config_.max_rotate)) {
            rotation_degrees =
                std::min(rotation_degrees, horizontalDegrees(drone_config_.max_rotate));
        }

        
        common::types::MovementCommand movement;
        movement.type = common::types::MovementCommandType::Rotate;
        movement.rotation = turn_degrees >= 0.0
                                ? common::types::RotationDirection::Left
                                : common::types::RotationDirection::Right;
        movement.angle = rotation_degrees * horizontal_angle[deg];
        return movementCommand(movement);
    }
    // Already facing the target, so move forward.
    common::types::MovementCommand movement;
    movement.type = common::types::MovementCommandType::Advance;
    movement.distance = horizontal_distance_cm * cm;

    // Remember this movement so its result can be checked next step.
    pending_translation_ = NavigationTarget{
        target.node,
        target.backtracking,
        target.vertical,
        desired_heading_degrees,
        horizontal_distance_cm * cm,
    };

    (void)current_cell;
    return movementCommand(movement);
}

// Creates one of the six lidar scan commands for the current cell.
common::types::MappingStepCommand MappingAlgorithmImpl_207610130_215664087::scanCommand(std::size_t scan_index) {
    const std::array<Orientation, kScanDirectionCount> scans{{
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},// Forward
        Orientation{-90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},// Right
        Orientation{180.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},// Backward
        Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},// Left
        Orientation{0.0 * horizontal_angle[deg], 90.0 * altitude_angle[deg]},// Up
        Orientation{0.0 * horizontal_angle[deg], -90.0 * altitude_angle[deg]},// Down
    }};

    common::types::MappingStepCommand command;
    command.scan_orientation = scans[scan_index];
    command.status = common::types::AlgorithmStatus::Working;

    // Remember that we are waiting for this scan result.
    waiting_for_scan_result_ = true;
    pending_scan_index_ = scan_index;
    return command;
}

// Creates the command indicating that mapping is complete.
common::types::MappingStepCommand MappingAlgorithmImpl_207610130_215664087::finishCommand() const {
    common::types::MappingStepCommand command;
    command.status = common::types::AlgorithmStatus::Finished;
    return command;
}

// Initializes DFS at the drone's starting cell.
void MappingAlgorithmImpl_207610130_215664087::initializeAtCurrentCell(
    const GridCell& current_cell,
    const Position3D& position) {
    visited_cells_.insert(current_cell);

    if (dfs_path_.empty()) {
        dfs_path_.push_back(PathNode{current_cell, position});
    }
}

// Checks whether the movement requested in the previous step succeeded.
void MappingAlgorithmImpl_207610130_215664087::handlePendingTranslationResult(const GridCell& current_cell) {
    if (!pending_translation_) {
        return;
    }

    const NavigationTarget completed_target = *pending_translation_;
    pending_translation_.reset();

    // Reaching the expected cell means the movement succeeded.
    if (current_cell == completed_target.node.cell) {
        failed_translation_count_ = 0;
        visited_cells_.insert(current_cell);

        if (completed_target.backtracking) {
            // Remove DFS nodes until the current cell becomes
            // the top of the backtracking path.
            while (dfs_path_.size() > 1 && !(dfs_path_.back().cell == current_cell)) {
                dfs_path_.pop_back();
            }
        } else if (dfs_path_.empty() || !(dfs_path_.back().cell == current_cell)) {
             // Add newly explored cell to the DFS path.
            dfs_path_.push_back(completed_target.node);
        }

        pending_target_.reset();
        return;
    }

    // Movement did not reach the target.
    ++failed_translation_count_;
    // Remember the failed cell so we do not repeatedly try it.
    attempted_cells_.insert(completed_target.node.cell);
    pending_target_.reset();
}

// Resets the six-direction scan sequence when entering a new cell.
void MappingAlgorithmImpl_207610130_215664087::resetScanSequenceForCell(const GridCell& current_cell) {
    scan_cell_ = current_cell;
    next_scan_index_ = 0;
    waiting_for_scan_result_ = false;
    pending_scan_index_.reset();
}

} // namespace algorithm_207610130_215664087
