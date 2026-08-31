#include <MissionControl/ScanResultToVoxels.h>

#include <cmath>
#include <mp-units/systems/si/math.h>

namespace mission_control_207610130_215664087 {

namespace {

constexpr double kTrigEpsilon = 1.0e-9;

[[nodiscard]] double snapped(double value) {
    return std::abs(value) < kTrigEpsilon ? 0.0 : value;
}

} // namespace

std::vector<Position3D> scanResultToHitVoxels(
    const common::types::LidarScanResult& scan_result,
    const Position3D& drone_position,
    const Orientation& drone_orientation,
    PhysicalLength voxel_size) {

    (void)voxel_size;
    std::vector<Position3D> hit_voxels;
    hit_voxels.reserve(scan_result.hits.size());

    for (const auto& hit : scan_result.hits) {
        const double distance_cm = hit.distance.force_numerical_value_in(cm);
        const HorizontalAngle total_azimuth = drone_orientation.horizontal + hit.relative_azimuth;
        const AltitudeAngle total_elevation = drone_orientation.altitude + hit.relative_elevation;

        const double cos_elev = snapped(si::cos(total_elevation).force_numerical_value_in(mp::one));
        const double sin_elev = snapped(si::sin(total_elevation).force_numerical_value_in(mp::one));
        const double cos_azim = snapped(si::cos(total_azimuth).force_numerical_value_in(mp::one));
        const double sin_azim = snapped(si::sin(total_azimuth).force_numerical_value_in(mp::one));

        const double dx_cm = distance_cm * cos_elev * cos_azim;
        const double dy_cm = distance_cm * cos_elev * sin_azim;
        const double dz_cm = distance_cm * sin_elev;

        hit_voxels.push_back(Position3D{
            drone_position.x + dx_cm * x_extent[cm],
            drone_position.y + dy_cm * y_extent[cm],
            drone_position.z + dz_cm * z_extent[cm],
        });
    }

    return hit_voxels;
}

} // namespace mission_control_207610130_215664087
