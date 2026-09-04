#pragma once

#include <Common/Types.h>
#include <Simulator/SimulationConfigTypes.h>

#include <filesystem>

namespace simulator_207610130_215664087 {

class ConfigParser {
public:
    [[nodiscard]] static common::types::DroneConfigData parseDroneConfig(
        const std::filesystem::path& file_path);

    [[nodiscard]] static common::types::LidarConfigData parseLidarConfig(
        const std::filesystem::path& file_path);

    [[nodiscard]] static common::types::MissionConfigData parseMissionConfig(
        const std::filesystem::path& file_path);

    [[nodiscard]] static SimulationConfigData parseSimulationConfig(
        const std::filesystem::path& file_path);

    [[nodiscard]] static SimulationCompositionData parseSimulationComposition(
        const std::filesystem::path& file_path);
};

} // namespace simulator_207610130_215664087
