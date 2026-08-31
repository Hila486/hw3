#pragma once

#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IDroneMovement.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>
#include <Common/Types.h>
#include <MissionControl/IDroneControl.h>

#include <memory>
#include <optional>
#include <string>

namespace mission_control_207610130_215664087 {

using namespace common;

class DroneControlImpl final : public mission_control::IDroneControl {
public:
    DroneControlImpl(
        types::DroneConfigData drone,
        types::MissionConfigData mission,
        ILidar& lidar,
        IGPS& gps,
        IDroneMovement& movement,
        IMutableMap3D& output_map,
        IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] types::DroneStepResult step() override;
    [[nodiscard]] types::DroneState state() const;

private:
    [[nodiscard]] bool executeMovementCommand(
        const types::MovementCommand& command,
        std::string& message);

    [[nodiscard]] bool isMovementCommandWithinLimits(
        const types::MovementCommand& command,
        std::string& message) const;

    [[nodiscard]] Position3D targetPositionForMovement(
        const Position3D& start,
        const Orientation& heading,
        const types::MovementCommand& command) const;

    [[nodiscard]] bool isSweptSphereKnownEmpty(
        const Position3D& start,
        const Position3D& target) const;

    void markCurrentDroneBodyAsEmpty();

    types::DroneConfigData drone_;
    types::MissionConfigData mission_;
    ILidar& lidar_;
    IGPS& gps_;
    IDroneMovement& movement_;
    IMutableMap3D& output_map_;
    IMappingAlgorithm& mapping_algorithm_;

    std::size_t step_index_ = 0;
    std::optional<types::LidarScanResult> latest_scan_{std::nullopt};
};

} // namespace mission_control_207610130_215664087
