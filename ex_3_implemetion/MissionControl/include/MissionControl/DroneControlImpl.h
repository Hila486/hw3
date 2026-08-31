#pragma once

#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IDroneMovement.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>
#include <Common/Types.h>
#include <MissionControl/IDroneControl.h>

#include <memory>
#include <vector>

namespace mission_control_207610130_215664087 {

using namespace common;

/**
 * @class DroneControlImpl
 * @brief Concrete implementation of IDroneControl for MissionControl.
 */
class DroneControlImpl final : public mission_control::IDroneControl {
public:
    DroneControlImpl(
        const common::types::DroneConfigData& drone_config,
        const common::types::MissionConfigData& mission_config,
        common::ILidar& lidar,
        common::IGPS& gps,
        common::IDroneMovement& movement,
        common::IMutableMap3D& map,
        common::IMappingAlgorithm& algorithm);

    [[nodiscard]] common::types::DroneStepResult step() override;

private:
    [[nodiscard]] bool validateAdvance(PhysicalLength distance, std::string& error_message) const;
    [[nodiscard]] bool validateElevate(PhysicalLength distance, std::string& error_message) const;
    [[nodiscard]] bool validateRotate(HorizontalAngle angle, std::string& error_message) const;
    [[nodiscard]] bool validateScan(const Orientation& orientation, std::string& error_message) const;

    void updateMapWithScan(const common::types::LidarScanResult& scan,
                           const Position3D& drone_pos,
                           const Orientation& drone_orient);

    common::types::DroneConfigData drone_config_;
    common::types::MissionConfigData mission_config_;
    common::ILidar& lidar_;
    common::IGPS& gps_;
    common::IDroneMovement& movement_;
    common::IMutableMap3D& map_;
    common::IMappingAlgorithm& algorithm_;

    std::size_t step_count_ = 0;
    std::unique_ptr<common::types::LidarScanResult> latest_scan_result_{nullptr};
};

} // namespace mission_control_207610130_215664087
