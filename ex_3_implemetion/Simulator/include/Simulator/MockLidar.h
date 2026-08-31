#pragma once

#include <Common/ILidar.h>
#include <Common/IMap3D.h>
#include <Common/Types.h>
#include <Simulator/MockGPS.h>

namespace simulator_207610130_215664087 {

using namespace common;

class MockLidar final : public common::ILidar {
public:
    MockLidar(
        common::types::LidarConfigData lidar_config,
        const common::IMap3D& hidden_map,
        const MockGPS& gps);

    [[nodiscard]] common::types::LidarScanResult scan(
        const Orientation& scan_orientation) override;

private:
    common::types::LidarConfigData lidar_config_;
    const common::IMap3D& hidden_map_;
    const MockGPS& gps_;
};

} // namespace simulator_207610130_215664087
