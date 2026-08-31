#pragma once

#include <Common/Types.h>

#include <filesystem>
#include <tuple>
#include <vector>

namespace simulator_207610130_215664087 {

using namespace common;

/**
 * @struct SimulationConfigData
 * @brief Holds environment-level configuration for one simulation scenario.
 */
struct SimulationConfigData {
    std::filesystem::path map_filename{};       ///< Path to the .npy ground-truth map file
    PhysicalLength map_resolution{};            ///< Voxel edge size in cm
    Position3D map_axes_offset{};               ///< Translation from mission coords to map coords
    Position3D initial_drone_position{};         ///< Drone spawn position
    HorizontalAngle initial_angle{};            ///< Drone initial heading
};

/**
 * @struct SimulationCompositionData
 * @brief Aggregated composition of all simulation scenarios, missions, drones, and lidars.
 */
struct SimulationCompositionData {
    std::filesystem::path composition_file{};   ///< Path to the composition YAML file

    /// Each entry pairs one SimulationConfigData with its associated mission configs
    std::vector<std::tuple<SimulationConfigData,
                           std::vector<common::types::MissionConfigData>>>
        simulation_mission_groups{};

    std::vector<common::types::DroneConfigData> drone_configs{};  ///< All drone configurations
    std::vector<common::types::LidarConfigData> lidar_configs{};  ///< All lidar configurations
};

} // namespace simulator_207610130_215664087
