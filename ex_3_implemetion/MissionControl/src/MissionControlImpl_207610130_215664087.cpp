#include <MissionControl/MissionControlImpl_207610130_215664087.h>

#include <Common/MissionControlRegistration.h>

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

// Global auto-registration macro call so that dlopen loads this factory automatically.
REGISTER_MISSION_CONTROL(mission_control_207610130_215664087::MissionControlImpl_207610130_215664087);

namespace mission_control_207610130_215664087 {

namespace {

void addError(std::vector<common::types::ErrorRef>& errors,
              std::string code,
              std::string message) {
    errors.push_back(common::types::ErrorRef{std::move(code), std::move(message)});
}

} // namespace

MissionControlImpl_207610130_215664087::MissionControlImpl_207610130_215664087(
    common::MissionControlDependencies dependencies)
    : mission_config_(dependencies.mission_config),
      drone_config_(dependencies.drone_config),
      output_map_(dependencies.output_map),
      output_map_file_(std::move(dependencies.output_map_file)),
      verbose_(dependencies.verbose),
      drone_control_(dependencies.drone_config,
                     dependencies.mission_config,
                     dependencies.lidar,
                     dependencies.gps,
                     dependencies.movement,
                     dependencies.output_map,
                     dependencies.mapping_algorithm) {}

/**
 * @brief Runs the drone mission stepping loop up to mission_config_.max_steps.
 */
common::types::MissionRunResult MissionControlImpl_207610130_215664087::runMission() {
    auto saveOutputMap = [this](std::vector<common::types::ErrorRef>& errors) {
        try {
            output_map_.save(output_map_file_);
            return true;
        } catch (const std::exception& exception) {
            addError(errors, "OUTPUT_MAP_SAVE_FAILED", exception.what());
        } catch (...) {
            addError(errors, "OUTPUT_MAP_SAVE_FAILED", "Unknown error while saving output map.");
        }
        return false;
    };

    if (verbose_) {
        std::cout << "[MissionControl] Starting mission run for output map: "
                  << output_map_file_.string() << std::endl;
    }

    for (std::size_t executed_steps = 0; executed_steps < mission_config_.max_steps; ++executed_steps) {
        const common::types::DroneStepResult step_result = drone_control_.step();
        const std::size_t completed_steps = executed_steps + 1;

        if (verbose_) {
            std::cout << "[MissionControl] Step " << completed_steps << "/" << mission_config_.max_steps
                      << " completed." << std::endl;
        }

        if (step_result.status == common::types::DroneStepStatus::Completed) {
            std::vector<common::types::ErrorRef> errors;
            if (!saveOutputMap(errors)) {
                return common::types::MissionRunResult{common::types::MissionRunStatus::Error,
                                                       completed_steps,
                                                       std::move(errors)};
            }

            return common::types::MissionRunResult{common::types::MissionRunStatus::Completed,
                                                   completed_steps,
                                                   {}};
        }

        if (step_result.status == common::types::DroneStepStatus::Error) {
            std::vector<common::types::ErrorRef> errors;
            addError(errors,
                     "DRONE_STEP_FAILED",
                     step_result.message.empty() ? "Drone control step failed."
                                                 : step_result.message);
            saveOutputMap(errors);
            return common::types::MissionRunResult{common::types::MissionRunStatus::Error,
                                                   completed_steps,
                                                   std::move(errors)};
        }
    }

    std::vector<common::types::ErrorRef> errors;
    if (!saveOutputMap(errors)) {
        return common::types::MissionRunResult{common::types::MissionRunStatus::Error,
                                               mission_config_.max_steps,
                                               std::move(errors)};
    }

    return common::types::MissionRunResult{common::types::MissionRunStatus::MaxSteps,
                                           mission_config_.max_steps,
                                           {}};
}

} // namespace mission_control_207610130_215664087
