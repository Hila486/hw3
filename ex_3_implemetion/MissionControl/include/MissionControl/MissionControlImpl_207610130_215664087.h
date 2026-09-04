#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <MissionControl/DroneControlImpl.h>

#include <filesystem>
#include <memory>

namespace mission_control_207610130_215664087 {

/**
 * Manages the execution loop of a single drone mapping mission: instantiates drone control,
 * steps through mission iterations, logs status, and outputs final map results.
 */

class MissionControlImpl_207610130_215664087 final : public common::IMissionControl {
public:

    // Creates the mission controller from the provided mission dependencies.
    explicit MissionControlImpl_207610130_215664087(common::MissionControlDependencies dependencies);

  
    // Runs the complete mapping mission until it finishes,
    // reaches the maximum number of steps, or encounters an error.
    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    // Configuration of the current mapping mission.
    common::types::MissionConfigData mission_config_;
    // Drone capabilities and movement limits.
    common::types::DroneConfigData drone_config_;
    // Map being built during the mission.
    common::IMutableMap3D& output_map_;
    // File path where the final output map will be saved.
    std::filesystem::path output_map_file_;
    // Controls whether detailed mission information is printed.
    bool verbose_ = false;
     // Controls the drone one step at a time during the mission.
    DroneControlImpl drone_control_;
};

} // namespace mission_control_207610130_215664087
