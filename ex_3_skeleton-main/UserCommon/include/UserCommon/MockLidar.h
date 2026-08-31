#pragma once

#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>

namespace user_common_207610130_215664087 {

/**
 * @class MockLidar
 * @brief Simulated LiDAR sensor implementation.
 * 
 * Ray-casts laser beams into the hidden ground-truth 3D map based on the drone's
 * current position (via IGPS) to calculate beam hit distances and obstacle returns,
 * implementing common::ILidar.
 */
class MockLidar final : public common::ILidar {
public:
    /**
     * @brief Constructs a MockLidar sensor instance.
     * @param config Lidar hardware configuration (FOV circles, range z_min/z_max, offset d).
     * @param map Reference to the hidden ground-truth 3D map.
     * @param gps Reference to the simulated GPS sensor for current drone position & orientation.
     */
    MockLidar(common::types::LidarConfigData config, const common::IMap3D& map, const common::IGPS& gps);

    /**
     * @brief Performs a multi-beam LiDAR scan at a specified sensor orientation.
     * @param scan_orientation Relative sensor orientation relative to drone heading.
     * @return LidarScanResult containing list of LidarHit distance measurements.
     */
    [[nodiscard]] common::types::LidarScanResult scan(Orientation scan_orientation) const override;

    /**
     * @brief Returns the LiDAR hardware configuration data.
     */
    [[nodiscard]] common::types::LidarConfigData config() const override;

private:
    /**
     * @brief Ray-traces a single laser beam direction to find obstacle intersection.
     * @param beam Absolute orientation of the laser beam in 3D world space.
     * @return Distance to nearest occupied voxel, or max limits if unblocked.
     */
    [[nodiscard]] PhysicalLength traceBeam(const Orientation& beam) const;

    common::types::LidarConfigData config_;  ///< Lidar sensor specifications
    const common::IMap3D& map_;              ///< Ground-truth 3D map
    const common::IGPS& gps_;                ///< Simulated GPS sensor reference
};

} // namespace user_common_207610130_215664087
