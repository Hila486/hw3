#include <Simulator/MockLidar.h>

#include <cmath>
#include <mp-units/systems/si/math.h>
#include <utility>

namespace simulator_207610130_215664087 {

namespace {

constexpr double kTrigEpsilon = 1.0e-9;

[[nodiscard]] double snapped(double value) {
    return std::abs(value) < kTrigEpsilon ? 0.0 : value;
}

} // namespace

MockLidar::MockLidar(
    common::types::LidarConfigData lidar_config,
    const common::IMap3D& hidden_map,
    const MockGPS& gps)
    : lidar_config_(std::move(lidar_config)),
      hidden_map_(hidden_map),
      gps_(gps) {}

common::types::LidarScanResult MockLidar::scan(const Orientation& scan_orientation) {
    common::types::LidarScanResult result;
    const Position3D drone_pos = gps_.position();
    const Orientation drone_heading = gps_.heading();

    const double max_range_cm = lidar_config_.max_distance.force_numerical_value_in(cm);
    const double step_size_cm = std::max(0.5, hidden_map_.getMapConfig().resolution.force_numerical_value_in(cm) * 0.5);

    for (const auto& beam : lidar_config_.beams) {
        const HorizontalAngle total_azimuth =
            drone_heading.horizontal + scan_orientation.horizontal + beam.relative_azimuth;
        const AltitudeAngle total_elevation =
            drone_heading.altitude + scan_orientation.altitude + beam.relative_elevation;

        const double cos_elev = snapped(si::cos(total_elevation).force_numerical_value_in(mp::one));
        const double sin_elev = snapped(si::sin(total_elevation).force_numerical_value_in(mp::one));
        const double cos_azim = snapped(si::cos(total_azimuth).force_numerical_value_in(mp::one));
        const double sin_azim = snapped(si::sin(total_azimuth).force_numerical_value_in(mp::one));

        const double dx = cos_elev * cos_azim;
        const double dy = cos_elev * sin_azim;
        const double dz = sin_elev;

        double distance_cm = step_size_cm;
        bool hit_found = false;

        while (distance_cm <= max_range_cm) {
            Position3D probe{
                drone_pos.x + distance_cm * dx * x_extent[cm],
                drone_pos.y + distance_cm * dy * y_extent[cm],
                drone_pos.z + distance_cm * dz * z_extent[cm],
            };

            const auto occupancy = hidden_map_.atVoxel(probe);
            if (occupancy == common::types::VoxelOccupancy::Occupied) {
                result.hits.push_back(common::types::LidarHit{
                    beam.relative_azimuth,
                    beam.relative_elevation,
                    distance_cm * x_extent[cm]
                });
                hit_found = true;
                break;
            }

            distance_cm += step_size_cm;
        }

        if (!hit_found) {
            result.hits.push_back(common::types::LidarHit{
                beam.relative_azimuth,
                beam.relative_elevation,
                max_range_cm * x_extent[cm]
            });
        }
    }

    return result;
}

} // namespace simulator_207610130_215664087
