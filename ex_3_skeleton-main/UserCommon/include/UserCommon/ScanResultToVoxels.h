#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>

namespace user_common_207610130_215664087 {

/**
 * @class ScanResultToVoxels
 * @brief Helper utility converting LiDAR scan hit results to 3D map voxel occupancy updates.
 * 
 * Ray-traces LiDAR scan hits, updating voxel occupancy states (Occupied, Empty,
 * PotentiallyOccupied) on a 3D mutable map.
 */
class ScanResultToVoxels {
public:
    /**
     * @brief Applies a LiDAR scan result directly onto a 3D mutable output map.
     * @param output_map The map to be updated with voxel occupancy information.
     * @param scan_origin Current 3D position of the LiDAR sensor (scan origin).
     * @param drone_heading Current heading orientation of the drone.
     * @param scan LiDAR scan hits result array.
     * @param lidar_config LiDAR hardware config (z_min, z_max).
     */
    static void applyToMap(common::IMutableMap3D& output_map,
                           const Position3D& scan_origin,
                           const Orientation& drone_heading,
                           const common::types::LidarScanResult& scan,
                           const common::types::LidarConfigData& lidar_config);
};

} // namespace user_common_207610130_215664087
