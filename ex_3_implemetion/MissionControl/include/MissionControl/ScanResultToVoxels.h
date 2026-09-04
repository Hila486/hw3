#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>

namespace mission_control_207610130_215664087 {

using namespace common;

// Utility class that converts lidar scan results into voxel information
// and updates the drone's output map.
class ScanResultToVoxels {
public:
    // Applies one lidar scan to the output map.
    // Marks voxels along the beam as empty and detected hit voxels as occupied.
    static void applyToMap(
        IMutableMap3D& output_map,
        const Position3D& scan_origin,
        const Orientation& drone_heading,
        const common::types::LidarScanResult& scan,
        const common::types::LidarConfigData& lidar_config);
};

} // namespace mission_control_207610130_215664087
