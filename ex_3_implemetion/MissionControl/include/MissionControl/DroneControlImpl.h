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

// Main mission-control implementation.
// Connects the mapping algorithm with the sensors, movement system,
// and the output map, and executes one drone step at a time.
class DroneControlImpl final : public mission_control::IDroneControl {
public:
    // Creates the mission controller with all required drone components
    // and mission/drone configuration.
    DroneControlImpl(
        types::DroneConfigData drone,
        types::MissionConfigData mission,
        ILidar& lidar,
        IGPS& gps,
        IDroneMovement& movement,
        IMutableMap3D& output_map,
        IMappingAlgorithm& mapping_algorithm);

    // Executes one control step of the drone mission.
    [[nodiscard]] types::DroneStepResult step() override;
    // Returns the current state that is provided to the mapping algorithm.
    [[nodiscard]] types::DroneState state() const;

private:
    // Executes a movement command after validating it.
    // Returns false and writes an error message if the command fails.
    [[nodiscard]] bool executeMovementCommand(
        const types::MovementCommand& command,
        std::string& message);

    // Checks whether the requested movement respects the drone's
    // configured movement limits.
    [[nodiscard]] bool isMovementCommandWithinLimits(
        const types::MovementCommand& command,
        std::string& message) const;

        // Calculates the position the drone would reach after a movement command.
    [[nodiscard]] Position3D targetPositionForMovement(
        const Position3D& start,
        const Orientation& heading,
        const types::MovementCommand& command) const;

    // Checks whether the full path of the drone body between two positions
    // is already known to be empty in the output map.
    [[nodiscard]] bool isSweptSphereKnownEmpty(
        const Position3D& start,
        const Position3D& target) const;

    // Marks the volume currently occupied by the drone as empty
    // in the output map.
    void markCurrentDroneBodyAsEmpty();

    // Drone hardware/capability configuration.
    types::DroneConfigData drone_;
    // Mission-specific configuration.
    types::MissionConfigData mission_;

    // Interfaces to the simulated drone hardware.
    ILidar& lidar_;
    IGPS& gps_;
    IDroneMovement& movement_;

    // Map that is gradually built by the mapping algorithm.
    IMutableMap3D& output_map_;
    // Mapping algorithm that decides what the drone should do next.
    IMappingAlgorithm& mapping_algorithm_;

    // Counts how many mission-control steps have been executed.
    std::size_t step_index_ = 0;
    // Stores the most recent lidar scan, if a scan has already occurred.
    std::optional<types::LidarScanResult> latest_scan_{std::nullopt};
};

} // namespace mission_control_207610130_215664087
