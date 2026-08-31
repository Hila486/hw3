#include <MissionControl/DroneControlImpl.h>
#include <MissionControl/ScanResultToVoxels.h>

#include <cmath>
#include <mp-units/systems/si/math.h>
#include <sstream>
#include <utility>

namespace mission_control_207610130_215664087 {

namespace {

constexpr double kTrigEpsilon = 1.0e-9;

[[nodiscard]] double horizontalDegrees(HorizontalAngle angle) {
    return angle.force_numerical_value_in(deg);
}

[[nodiscard]] double altitudeDegrees(AltitudeAngle angle) {
    return angle.force_numerical_value_in(deg);
}

[[nodiscard]] double snapped(double value) {
    return std::abs(value) < kTrigEpsilon ? 0.0 : value;
}

} // namespace

DroneControlImpl::DroneControlImpl(
    const common::types::DroneConfigData& drone_config,
    const common::types::MissionConfigData& mission_config,
    common::ILidar& lidar,
    common::IGPS& gps,
    common::IDroneMovement& movement,
    common::IMutableMap3D& map,
    common::IMappingAlgorithm& algorithm)
    : drone_config_(drone_config),
      mission_config_(mission_config),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      map_(map),
      algorithm_(algorithm) {}

bool DroneControlImpl::validateAdvance(PhysicalLength distance, std::string& error_message) const {
    const double distance_cm = distance.force_numerical_value_in(cm);
    const double max_advance_cm = drone_config_.max_horizontal_speed.force_numerical_value_in(cm);

    if (std::abs(distance_cm) > max_advance_cm) {
        std::ostringstream ss;
        ss << "Advance distance " << distance_cm << " exceeds max speed " << max_advance_cm;
        error_message = ss.str();
        return false;
    }
    return true;
}

bool DroneControlImpl::validateElevate(PhysicalLength distance, std::string& error_message) const {
    const double distance_cm = distance.force_numerical_value_in(cm);
    const double max_elevate_cm = drone_config_.max_vertical_speed.force_numerical_value_in(cm);

    if (std::abs(distance_cm) > max_elevate_cm) {
        std::ostringstream ss;
        ss << "Elevate distance " << distance_cm << " exceeds max vertical speed " << max_elevate_cm;
        error_message = ss.str();
        return false;
    }
    return true;
}

bool DroneControlImpl::validateRotate(HorizontalAngle angle, std::string& error_message) const {
    const double angle_deg = horizontalDegrees(angle);
    const double max_rotate_deg = horizontalDegrees(drone_config_.max_rotation_speed);

    if (angle_deg < 0.0 || angle_deg > max_rotate_deg) {
        std::ostringstream ss;
        ss << "Rotate angle " << angle_deg << " is invalid or exceeds max rotation " << max_rotate_deg;
        error_message = ss.str();
        return false;
    }
    return true;
}

bool DroneControlImpl::validateScan(const Orientation& orientation, std::string& error_message) const {
    const double horiz_deg = horizontalDegrees(orientation.horizontal);
    const double alt_deg = altitudeDegrees(orientation.altitude);

    if (horiz_deg < 0.0 || horiz_deg >= 360.0) {
        error_message = "Scan horizontal angle must be in [0, 360).";
        return false;
    }
    if (alt_deg < -90.0 || alt_deg > 90.0) {
        error_message = "Scan altitude angle must be in [-90, 90].";
        return false;
    }
    return true;
}

void DroneControlImpl::updateMapWithScan(
    const common::types::LidarScanResult& scan,
    const Position3D& drone_pos,
    const Orientation& drone_orient) {

    // First mark ray paths as empty
    const PhysicalLength resolution = map_.getMapConfig().resolution;
    const double step_size_cm = std::max(0.5, resolution.force_numerical_value_in(cm) * 0.5);

    for (const auto& hit : scan.hits) {
        const double distance_cm = hit.distance.force_numerical_value_in(cm);
        const HorizontalAngle total_azimuth = drone_orient.horizontal + hit.relative_azimuth;
        const AltitudeAngle total_elevation = drone_orient.altitude + hit.relative_elevation;

        const double cos_elev = snapped(si::cos(total_elevation).force_numerical_value_in(mp::one));
        const double sin_elev = snapped(si::sin(total_elevation).force_numerical_value_in(mp::one));
        const double cos_azim = snapped(si::cos(total_azimuth).force_numerical_value_in(mp::one));
        const double sin_azim = snapped(si::sin(total_azimuth).force_numerical_value_in(mp::one));

        const double dx_unit = cos_elev * cos_azim;
        const double dy_unit = cos_elev * sin_azim;
        const double dz_unit = sin_elev;

        // Trace empty voxels up to hit distance minus small buffer
        const double trace_limit_cm = std::max(0.0, distance_cm - step_size_cm);
        const std::size_t num_samples = static_cast<std::size_t>(std::ceil(trace_limit_cm / step_size_cm));

        for (std::size_t s = 1; s <= num_samples; ++s) {
            const double cur_dist_cm = std::min(trace_limit_cm, s * step_size_cm);
            Position3D empty_pos{
                drone_pos.x + cur_dist_cm * dx_unit * x_extent[cm],
                drone_pos.y + cur_dist_cm * dy_unit * y_extent[cm],
                drone_pos.z + cur_dist_cm * dz_unit * z_extent[cm],
            };
            if (map_.atVoxel(empty_pos) == common::types::VoxelOccupancy::Unmapped) {
                map_.setVoxel(empty_pos, common::types::VoxelOccupancy::Empty);
            }
        }
    }

    // Now mark occupied hit voxels
    const auto hit_voxels = scanResultToHitVoxels(scan, drone_pos, drone_orient, resolution);
    for (const auto& hit_pos : hit_voxels) {
        map_.setVoxel(hit_pos, common::types::VoxelOccupancy::Occupied);
    }
}

common::types::DroneStepResult DroneControlImpl::step() {
    ++step_count_;

    const Position3D current_pos = gps_.position();
    const Orientation current_orient = gps_.heading();

    common::types::DroneState drone_state{
        current_pos,
        current_orient,
        step_count_
    };

    common::types::MappingStepCommand command;
    try {
        command = algorithm_.nextStep(drone_state, latest_scan_result_.get());
    } catch (const std::exception& ex) {
        return common::types::DroneStepResult{
            common::types::DroneStepStatus::Error,
            std::string("Algorithm threw exception in nextStep: ") + ex.what()
        };
    } catch (...) {
        return common::types::DroneStepResult{
            common::types::DroneStepStatus::Error,
            "Algorithm threw unknown exception in nextStep."
        };
    }

    // Clear previous scan result pointer after supplying it to nextStep
    latest_scan_result_.reset();

    switch (command.action) {
        case common::types::StepAction::Scan: {
            std::string err;
            if (!validateScan(command.scan_orientation, err)) {
                return common::types::DroneStepResult{common::types::DroneStepStatus::Error, err};
            }
            try {
                auto scan_res = lidar_.scan(command.scan_orientation);
                updateMapWithScan(scan_res, current_pos, current_orient);
                latest_scan_result_ = std::make_unique<common::types::LidarScanResult>(std::move(scan_res));
                return common::types::DroneStepResult{common::types::DroneStepStatus::Continue, {}};
            } catch (const std::exception& ex) {
                return common::types::DroneStepResult{common::types::DroneStepStatus::Error, ex.what()};
            }
        }

        case common::types::StepAction::Advance: {
            std::string err;
            if (!validateAdvance(command.distance, err)) {
                return common::types::DroneStepResult{common::types::DroneStepStatus::Error, err};
            }
            try {
                auto move_res = movement_.advance(command.distance);
                if (!move_res.success) {
                    return common::types::DroneStepResult{common::types::DroneStepStatus::Error, "Advance movement failed."};
                }
                return common::types::DroneStepResult{common::types::DroneStepStatus::Continue, {}};
            } catch (const std::exception& ex) {
                return common::types::DroneStepResult{common::types::DroneStepStatus::Error, ex.what()};
            }
        }

        case common::types::StepAction::Elevate: {
            std::string err;
            if (!validateElevate(command.distance, err)) {
                return common::types::DroneStepResult{common::types::DroneStepStatus::Error, err};
            }
            try {
                auto move_res = movement_.elevate(command.distance);
                if (!move_res.success) {
                    return common::types::DroneStepResult{common::types::DroneStepStatus::Error, "Elevate movement failed."};
                }
                return common::types::DroneStepResult{common::types::DroneStepStatus::Continue, {}};
            } catch (const std::exception& ex) {
                return common::types::DroneStepResult{common::types::DroneStepStatus::Error, ex.what()};
            }
        }

        case common::types::StepAction::Rotate: {
            std::string err;
            if (!validateRotate(command.angle, err)) {
                return common::types::DroneStepResult{common::types::DroneStepStatus::Error, err};
            }
            try {
                auto move_res = movement_.rotate(command.rotation_direction, command.angle);
                if (!move_res.success) {
                    return common::types::DroneStepResult{common::types::DroneStepStatus::Error, "Rotate movement failed."};
                }
                return common::types::DroneStepResult{common::types::DroneStepStatus::Continue, {}};
            } catch (const std::exception& ex) {
                return common::types::DroneStepResult{common::types::DroneStepStatus::Error, ex.what()};
            }
        }

        case common::types::StepAction::Finish: {
            return common::types::DroneStepResult{common::types::DroneStepStatus::Completed, {}};
        }
    }

    return common::types::DroneStepResult{common::types::DroneStepStatus::Error, "Unknown command action."};
}

} // namespace mission_control_207610130_215664087
