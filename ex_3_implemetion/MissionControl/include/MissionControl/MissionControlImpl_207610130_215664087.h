#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <MissionControl/DroneControlImpl.h>

#include <filesystem>
#include <memory>

namespace mission_control_207610130_215664087 {

/**
 * @class MissionControlImpl_207610130_215664087
 * @brief Concrete mission controller implementing common::IMissionControl interface.
 * 
 * Manages the execution loop of a single drone mapping mission: instantiates drone control,
 * steps through mission iterations, logs status, and outputs final map results.
 */
class MissionControlImpl_207610130_215664087 final : public common::IMissionControl {
public:
    /**
     * @brief Constructs a MissionControl instance from dependencies.
     * @param dependencies MissionControlDependencies struct containing hardware and algorithm references.
     */
    explicit MissionControlImpl_207610130_215664087(common::MissionControlDependencies dependencies);

    /**
     * @brief Executes the complete drone mapping mission loop.
     * @return MissionRunResult containing final status (Completed, MaxSteps, Error) and step count.
     */
    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    common::types::MissionConfigData mission_config_;
    common::types::DroneConfigData drone_config_;
    common::IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
    bool verbose_ = false;
    DroneControlImpl drone_control_;
};

} // namespace mission_control_207610130_215664087
