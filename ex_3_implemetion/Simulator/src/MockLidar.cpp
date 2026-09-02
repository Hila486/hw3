#include <Simulator/MockLidar.h>

#include <UserCommon/GeometryUtils.h>
#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <limits>

namespace simulator_207610130_215664087 {

using namespace user_common_207610130_215664087;

namespace {

[[nodiscard]] std::size_t beams_on_circle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t i = 0; i < circle_index; ++i) {
        count *= 4;
    }
    return count;
}

[[nodiscard]] HorizontalAngle horizontal_delta(PhysicalLength offset, PhysicalLength distance) {
    return HorizontalAngle{si::atan2(offset, distance)};
}

[[nodiscard]] AltitudeAngle altitude_delta(PhysicalLength offset, PhysicalLength distance) {
    return AltitudeAngle{si::atan2(offset, distance)};
}

} // namespace

MockLidar::MockLidar(
    common::types::LidarConfigData lidar_config,
    const common::IMap3D& hidden_map,
    const MockGPS& gps)
    : lidar_config_(lidar_config),
      hidden_map_(hidden_map),
      gps_(gps) {}

common::types::LidarConfigData MockLidar::config() const {
    return lidar_config_;
}

common::types::LidarScanResult MockLidar::scan(Orientation scan_orientation) const {
    common::types::LidarScanResult results;
    if (lidar_config_.fov_circles == 0) {
        return results;
    }

    const Orientation sensor_heading = gps_.heading();
    const Orientation center_beam_abs =
        absoluteBeamOrientation(sensor_heading, scan_orientation);

    const PhysicalLength center_distance = traceBeam(center_beam_abs);
    results.push_back(common::types::LidarHit{center_distance, scan_orientation});

    for (std::size_t circle = 1; circle < lidar_config_.fov_circles; ++circle) {
        const std::size_t beam_count = beams_on_circle(circle);
        const PhysicalLength radius = static_cast<double>(circle) * lidar_config_.d;

        for (std::size_t i = 0; i < beam_count; ++i) {
            const auto theta = (360.0 * static_cast<double>(i) / static_cast<double>(beam_count)) * deg;
            const PhysicalLength horizontal_offset = radius * si::cos(theta);
            const PhysicalLength altitude_offset = radius * si::sin(theta);

            const Orientation offset{
                horizontal_delta(horizontal_offset, lidar_config_.z_min),
                altitude_delta(altitude_offset, lidar_config_.z_min),
            };
            const Orientation relative_beam{
                scan_orientation.horizontal + offset.horizontal,
                scan_orientation.altitude + offset.altitude,
            };
            const Orientation absolute_beam =
                absoluteBeamOrientation(sensor_heading, relative_beam);
            const PhysicalLength distance = traceBeam(absolute_beam);
            results.push_back(common::types::LidarHit{distance, relative_beam});
        }
    }

    return results;
}

PhysicalLength MockLidar::traceBeam(const Orientation& beam_orientation) const {
    const Position3D origin = gps_.position();
    const PhysicalLength step = 0.1 * hidden_map_.getMapConfig().resolution;

    for (PhysicalLength distance = 0.0 * cm; distance <= lidar_config_.z_max; distance += step) {
        const Position3D sample = pointAlongBeam(origin, beam_orientation, distance);
        if (hidden_map_.atVoxel(sample) == common::types::VoxelOccupancy::Occupied) {
            if (distance < lidar_config_.z_min) {
                return 0.0 * cm;
            }
            return distance;
        }
    }

    return std::numeric_limits<double>::max() * cm;
}

} // namespace simulator_207610130_215664087
