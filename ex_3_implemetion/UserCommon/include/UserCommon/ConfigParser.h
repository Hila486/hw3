#pragma once

#include <Common/Types.h>
#include <UserCommon/SimulationConfigTypes.h>
#include <filesystem>

namespace user_common_207610130_215664087 {

/**
 * @class ConfigParser
 * @brief Utility for parsing simulation and mission YAML configuration files.
 * 
 * Provides static helper methods to parse drone, mission, LiDAR, simulation,
 * and composition YAML files into standard typed configuration structures.
 */
class ConfigParser {
public:
    /**
     * @brief Parses a simulation composition YAML file defining Cartesian groups.
     * @param filepath Path to the composition YAML file.
     * @return SimulationCompositionData structure containing all groups.
     */
    [[nodiscard]] static SimulationCompositionData
    parseSimulationComposition(const std::filesystem::path& filepath);

    /**
     * @brief Parses a simulation environment configuration YAML file.
     * @param filepath Path to the simulation YAML file.
     * @return SimulationConfigData structure.
     */
    [[nodiscard]] static SimulationConfigData
    parseSimulationConfig(const std::filesystem::path& filepath);

    /**
     * @brief Parses a mission configuration YAML file.
     * @param filepath Path to the mission YAML file.
     * @return MissionConfigData structure.
     */
    [[nodiscard]] static common::types::MissionConfigData
    parseMissionConfig(const std::filesystem::path& filepath);

    /**
     * @brief Parses a drone physical configuration YAML file.
     * @param filepath Path to the drone YAML file.
     * @return DroneConfigData structure.
     */
    [[nodiscard]] static common::types::DroneConfigData
    parseDroneConfig(const std::filesystem::path& filepath);

    /**
     * @brief Parses a LiDAR sensor configuration YAML file.
     * @param filepath Path to the LiDAR YAML file.
     * @return LidarConfigData structure.
     */
    [[nodiscard]] static common::types::LidarConfigData
    parseLidarConfig(const std::filesystem::path& filepath);
};

} // namespace user_common_207610130_215664087
