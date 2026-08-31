#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>

namespace mission_control_207610130_215664087 {

using namespace common;

class ScanResultToVoxels {
public:
    static void applyToMap(
        IMutableMap3D& output_map,
        const Position3D& scan_origin,
        const Orientation& drone_heading,
        const common::types::LidarScanResult& scan,
        const common::types::LidarConfigData& lidar_config);
};

} // namespace mission_control_207610130_215664087
