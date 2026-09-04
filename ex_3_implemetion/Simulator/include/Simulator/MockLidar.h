#pragma once

#include <Common/ILidar.h>
#include <Common/IMap3D.h>
#include <Common/Types.h>
#include <Simulator/MockGPS.h>

namespace simulator_207610130_215664087 {

using namespace common;
// Mock implementation of the lidar sensor used by the simulator.
// Uses the hidden real map and the simulated GPS to produce lidar scan results.
class MockLidar final : public common::ILidar {
public:
    // Creates the lidar mock with its configuration, the hidden real map,
    // and the GPS used to know the drone's current position.
    MockLidar(
        common::types::LidarConfigData lidar_config,
        const common::IMap3D& hidden_map,
        const MockGPS& gps);

    
    // Performs a lidar scan in the requested orientation
    // and returns the detected hits.
    [[nodiscard]] common::types::LidarScanResult scan(
        Orientation scan_orientation) const override;
    // Returns the lidar configuration.
    [[nodiscard]] common::types::LidarConfigData config() const override;

private:
    // Traces a single lidar beam through the hidden map
    // and returns the distance to the first detected obstacle.
    [[nodiscard]] PhysicalLength traceBeam(const Orientation& beam_orientation) const;

    // Lidar capabilities such as range and field-of-view configuration.
    common::types::LidarConfigData lidar_config_;
    // The real map used only by the simulator to determine what the lidar sees.
    const common::IMap3D& hidden_map_;
    // GPS used to get the drone's current simulated position.
    const MockGPS& gps_;
};

} // namespace simulator_207610130_215664087
