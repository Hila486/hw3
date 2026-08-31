#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>

#include <MissionControl/IDroneControl.h>

#include <optional>
#include <string>

namespace user_common_207610130_215664087 {

/**
 * @class DroneControlImpl
 * @brief Concrete implementation of mission_control::IDroneControl interface.
 * 
 * Coordinates the drone mapping mission step cycle: querying the algorithm for next step,
 * executing movement commands within drone physical constraints, performing LiDAR scans,
 * ray-casting scan results onto the output map, and updating step states.
 */
class DroneControlImpl final : public mission_control::IDroneControl {
public:
    /**
     * @brief Constructs a DroneControlImpl instance.
     * @param drone Drone physical configuration specs (radius, max_advance, max_rotate, max_elevate).
     * @param mission Mission configuration data.
     * @param lidar Reference to the simulated LiDAR hardware device.
     * @param gps Reference to the simulated GPS sensor.
     * @param movement Reference to the simulated drone movement hardware.
     * @param output_map Reference to the 3D mutable output map to be built.
     * @param mapping_algorithm Reference to the mapping algorithm driving navigation.
     */
    DroneControlImpl(common::types::DroneConfigData drone,
                     common::types::MissionConfigData mission,
                     common::ILidar& lidar,
                     common::IGPS& gps,
                     common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm);

    /**
     * @brief Executes one simulation step cycle.
     * @return DroneStepResult indicating Continue, Completed, or Error with message.
     */
    [[nodiscard]] common::types::DroneStepResult step() override;

    /**
     * @brief Returns current state snapshot of the drone (position, heading, step index).
     */
    [[nodiscard]] common::types::DroneState state() const override;

private:
    [[nodiscard]] bool executeMovementCommand(const common::types::MovementCommand& command,
                                              std::string& message);
    [[nodiscard]] bool isMovementCommandWithinLimits(const common::types::MovementCommand& command,
                                                     std::string& message) const;
    [[nodiscard]] Position3D targetPositionForMovement(
        const Position3D& start,
        const Orientation& heading,
        const common::types::MovementCommand& command) const;
    [[nodiscard]] bool isCenterInsideMapBounds(const Position3D& center) const;
    [[nodiscard]] bool isSphereKnownEmptyAt(const Position3D& center,
                                            const Position3D& already_free_center) const;
    [[nodiscard]] bool isSweptSphereKnownEmpty(const Position3D& start,
                                               const Position3D& end) const;
    [[nodiscard]] double safetySampleStepCm() const;
    void markCurrentDroneBodyAsEmpty();

    common::types::DroneConfigData drone_;                ///< Drone physical specs
    common::types::MissionConfigData mission_;            ///< Mission configuration
    common::ILidar& lidar_;                                ///< LiDAR sensor device
    common::IGPS& gps_;                                    ///< GPS sensor device
    common::IDroneMovement& movement_;                    ///< Drone movement actuator
    common::IMutableMap3D& output_map_;                    ///< Map grid built by drone
    common::IMappingAlgorithm& mapping_algorithm_;        ///< Mapping algorithm driving steps
    std::optional<common::types::LidarScanResult> latest_scan_; ///< Latest scan result buffered
    std::size_t step_index_ = 0;                          ///< Current step counter
};

} // namespace user_common_207610130_215664087
