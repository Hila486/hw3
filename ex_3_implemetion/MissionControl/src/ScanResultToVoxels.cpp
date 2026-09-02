#include <MissionControl/ScanResultToVoxels.h>

#include <UserCommon/GeometryUtils.h>
#include <mp-units/systems/si/math.h>

#include <cmath>
#include <limits>

namespace mission_control_207610130_215664087 {

using namespace user_common_207610130_215664087;

namespace {

[[nodiscard]] bool isZeroDistance(PhysicalLength distance) {
    return distance == 0.0 * cm;
}

[[nodiscard]] bool isMissDistance(PhysicalLength distance) {
    return distance.force_numerical_value_in(cm) == std::numeric_limits<double>::max();
}

[[nodiscard]] int occupancyPriority(common::types::VoxelOccupancy occupancy) {
    switch (occupancy) {
        case common::types::VoxelOccupancy::Occupied:
            return 3;
        case common::types::VoxelOccupancy::Empty:
            return 2;
        case common::types::VoxelOccupancy::PotentiallyOccupied:
            return 1;
        case common::types::VoxelOccupancy::Unmapped:
        case common::types::VoxelOccupancy::OutOfBounds:
            return 0;
    }
    return 0;
}

void setIfStronger(
    IMutableMap3D& output_map,
    const Position3D& position,
    common::types::VoxelOccupancy value) {

    const common::types::VoxelOccupancy current_value = output_map.atVoxel(position);
    if (occupancyPriority(value) > occupancyPriority(current_value)) {
        output_map.set(position, value);
    }
}

void markBeamSegment(
    IMutableMap3D& output_map,
    const Position3D& scan_origin,
    const Orientation& beam_orientation,
    PhysicalLength start_distance,
    PhysicalLength end_distance,
    PhysicalLength step,
    common::types::VoxelOccupancy value) {

    for (PhysicalLength distance = start_distance; distance <= end_distance; distance += step) {
        const Position3D current_point = pointAlongBeam(scan_origin, beam_orientation, distance);
        setIfStronger(output_map, current_point, value);
    }
}

} // namespace

void ScanResultToVoxels::applyToMap(
    IMutableMap3D& output_map,
    const Position3D& scan_origin,
    const Orientation& drone_heading,
    const common::types::LidarScanResult& scan,
    const common::types::LidarConfigData& lidar_config) {

    const PhysicalLength step = 0.1 * output_map.getMapConfig().resolution;
    if (step <= 0.0 * cm) {
        return;
    }

    for (const common::types::LidarHit& hit : scan) {
        const Orientation beam_orientation = absoluteBeamOrientation(drone_heading, hit.angle);

        if (isZeroDistance(hit.distance)) {
            markBeamSegment(
                output_map,
                scan_origin,
                beam_orientation,
                0.0 * cm,
                lidar_config.z_min,
                step,
                common::types::VoxelOccupancy::PotentiallyOccupied);
            continue;
        }

        if (isMissDistance(hit.distance)) {
            markBeamSegment(
                output_map,
                scan_origin,
                beam_orientation,
                0.0 * cm,
                lidar_config.z_max,
                step,
                common::types::VoxelOccupancy::Empty);
            continue;
        }

        if (hit.distance > 0.0 * cm) {
            markBeamSegment(
                output_map,
                scan_origin,
                beam_orientation,
                0.0 * cm,
                hit.distance - step,
                step,
                common::types::VoxelOccupancy::Empty);
            setIfStronger(
                output_map,
                pointAlongBeam(scan_origin, beam_orientation, hit.distance),
                common::types::VoxelOccupancy::Occupied);
        }
    }
}

} // namespace mission_control_207610130_215664087
