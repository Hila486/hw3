#pragma once

#include <Common/IMap3D.h>
#include <Common/Types.h>

#include <vector>

namespace mission_control_207610130_215664087 {

using namespace common;

/**
 * @brief Converts LiDAR scan polar hit results to discrete 3D world voxel coordinates.
 */
std::vector<Position3D> scanResultToHitVoxels(
    const common::types::LidarScanResult& scan_result,
    const Position3D& drone_position,
    const Orientation& drone_orientation,
    PhysicalLength voxel_size);

} // namespace mission_control_207610130_215664087
