#include <Simulator/ConfigParser.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace simulator_207610130_215664087 {

namespace {

YAML::Node loadYamlFile(const std::filesystem::path& file_path) {
    try {
        return YAML::LoadFile(file_path.string());
    } catch (const std::exception& exception) {
        throw std::runtime_error("Failed to parse YAML file " + file_path.string() + ": " + exception.what());
    }
}

template <typename T>
T requiredValue(const YAML::Node& node, const std::string& key, const std::string& context) {
    const YAML::Node child = node[key];
    if (!child) {
        throw std::runtime_error("Missing required key '" + key + "' in " + context);
    }
    try {
        return child.as<T>();
    } catch (const std::exception& exception) {
        throw std::runtime_error("Invalid value for key '" + key + "' in " + context + ": " + exception.what());
    }
}

template <typename T>
T optionalValue(const YAML::Node& node, const std::string& key, const T& fallback) {
    const YAML::Node child = node[key];
    if (!child) {
        return fallback;
    }
    try {
        return child.as<T>();
    } catch (...) {
        return fallback;
    }
}

std::filesystem::path resolveRelativePath(
    const std::filesystem::path& base_file_path,
    const std::string& relative_or_absolute_path) {
    const std::filesystem::path parsed_path(relative_or_absolute_path);
    if (parsed_path.is_absolute()) {
        return parsed_path;
    }
    return base_file_path.parent_path() / parsed_path;
}

} // namespace

common::types::DroneConfigData ConfigParser::parseDroneConfig(
    const std::filesystem::path& file_path) {
    const YAML::Node root = loadYamlFile(file_path);
    const YAML::Node config = root["drone_config"] ? root["drone_config"] : root;
    const std::string context = file_path.string();

    common::types::DroneConfigData drone_config;
    drone_config.radius =
        requiredValue<double>(config, "radius", context) * x_extent[cm];
    drone_config.max_horizontal_speed =
        requiredValue<double>(config, "max_horizontal_speed", context) * x_extent[cm];
    drone_config.max_vertical_speed =
        requiredValue<double>(config, "max_vertical_speed", context) * z_extent[cm];
    drone_config.max_rotation_speed =
        requiredValue<double>(config, "max_rotation_speed", context) * horizontal_angle[deg];

    return drone_config;
}

common::types::LidarConfigData ConfigParser::parseLidarConfig(
    const std::filesystem::path& file_path) {
    const YAML::Node root = loadYamlFile(file_path);
    const YAML::Node config = root["lidar_config"] ? root["lidar_config"] : root;
    const std::string context = file_path.string();

    common::types::LidarConfigData lidar_config;
    lidar_config.max_distance =
        requiredValue<double>(config, "max_distance", context) * x_extent[cm];

    const YAML::Node beams_node = config["beams"];
    if (!beams_node || !beams_node.IsSequence()) {
        throw std::runtime_error("Missing or invalid 'beams' list in " + context);
    }

    for (const auto& beam_node : beams_node) {
        const double azimuth = requiredValue<double>(beam_node, "relative_azimuth", context);
        const double elevation = requiredValue<double>(beam_node, "relative_elevation", context);
        lidar_config.beams.push_back(common::types::LidarBeam{
            azimuth * horizontal_angle[deg],
            elevation * altitude_angle[deg]
        });
    }

    return lidar_config;
}

common::types::MissionConfigData ConfigParser::parseMissionConfig(
    const std::filesystem::path& file_path) {
    const YAML::Node root = loadYamlFile(file_path);
    const YAML::Node config = root["mission_config"] ? root["mission_config"] : root;
    const std::string context = file_path.string();

    common::types::MissionConfigData mission_config;
    mission_config.max_steps = requiredValue<std::size_t>(config, "max_steps", context);
    mission_config.gps_resolution =
        requiredValue<double>(config, "gps_resolution", context) * x_extent[cm];

    // output_mapping_resolution_factor: defaults to 1.0
    mission_config.output_mapping_resolution_factor =
        optionalValue<double>(config, "output_mapping_resolution_factor", 1.0);

    const YAML::Node bounds_node = config["mission_bounds"];
    if (!bounds_node) {
        throw std::runtime_error("Missing 'mission_bounds' in " + context);
    }

    const auto parseInterval = [&](const std::string& axis) {
        const YAML::Node axis_node = bounds_node[axis];
        if (!axis_node) {
            throw std::runtime_error("Missing mission_bounds axis '" + axis + "' in " + context);
        }
        const double min_val = requiredValue<double>(axis_node, "min", context);
        const double max_val = requiredValue<double>(axis_node, "max", context);
        return std::make_pair(min_val, max_val);
    };

    const auto [x_min, x_max] = parseInterval("x");
    const auto [y_min, y_max] = parseInterval("y");
    const auto [z_min, z_max] = parseInterval("z");

    mission_config.mission_bounds = common::types::MappingBounds{
        Interval{x_min * x_extent[cm], x_max * x_extent[cm]},
        Interval{y_min * y_extent[cm], y_max * y_extent[cm]},
        Interval{z_min * z_extent[cm], z_max * z_extent[cm]}
    };

    return mission_config;
}

SimulationConfigData ConfigParser::parseSimulationConfig(
    const std::filesystem::path& file_path) {
    const YAML::Node root = loadYamlFile(file_path);
    const YAML::Node config = root["simulation_config"] ? root["simulation_config"] : root;
    const std::string context = file_path.string();

    SimulationConfigData sim_config;
    const std::string map_file_raw = requiredValue<std::string>(config, "map_filename", context);
    sim_config.map_filename = resolveRelativePath(file_path, map_file_raw);

    sim_config.map_resolution =
        requiredValue<double>(config, "map_resolution", context) * x_extent[cm];

    const YAML::Node offset_node = config["map_axes_offset"];
    if (offset_node) {
        const double ox = optionalValue<double>(offset_node, "x", 0.0);
        const double oy = optionalValue<double>(offset_node, "y", 0.0);
        const double oz = optionalValue<double>(offset_node, "z", 0.0);
        sim_config.map_axes_offset = Position3D{ox * x_extent[cm], oy * y_extent[cm], oz * z_extent[cm]};
    }

    const YAML::Node pos_node = config["initial_drone_position"];
    if (pos_node) {
        const double px = optionalValue<double>(pos_node, "x", 0.0);
        const double py = optionalValue<double>(pos_node, "y", 0.0);
        const double pz = optionalValue<double>(pos_node, "z", 0.0);
        sim_config.initial_drone_position = Position3D{px * x_extent[cm], py * y_extent[cm], pz * z_extent[cm]};
    }

    const double init_angle = optionalValue<double>(config, "initial_angle", 0.0);
    sim_config.initial_angle = init_angle * horizontal_angle[deg];

    return sim_config;
}

SimulationCompositionData ConfigParser::parseSimulationComposition(
    const std::filesystem::path& file_path) {
    const YAML::Node root = loadYamlFile(file_path);
    const YAML::Node composition_node = root["simulation_compositions"] ? root["simulation_compositions"] : root;
    const std::string context = file_path.string();

    SimulationCompositionData composition;
    composition.composition_file = file_path;

    const YAML::Node simulations_node = composition_node["simulations"];
    if (!simulations_node || !simulations_node.IsSequence()) {
        throw std::runtime_error("Missing or invalid 'simulations' sequence in " + context);
    }

    for (const auto& sim_entry : simulations_node) {
        const std::string sim_config_file_raw = requiredValue<std::string>(sim_entry, "simulation_config", context);
        const std::filesystem::path sim_config_path = resolveRelativePath(file_path, sim_config_file_raw);
        SimulationConfigData sim_config = parseSimulationConfig(sim_config_path);

        const YAML::Node mission_configs_node = sim_entry["mission_configs"];
        if (!mission_configs_node || !mission_configs_node.IsSequence()) {
            throw std::runtime_error("Missing or invalid 'mission_configs' sequence in " + context);
        }

        std::vector<common::types::MissionConfigData> missions;
        for (const auto& mission_file_node : mission_configs_node) {
            const std::string mission_file_raw = mission_file_node.as<std::string>();
            const std::filesystem::path mission_path = resolveRelativePath(file_path, mission_file_raw);
            missions.push_back(parseMissionConfig(mission_path));
        }

        composition.simulation_mission_groups.emplace_back(std::move(sim_config), std::move(missions));
    }

    const YAML::Node drones_node = composition_node["drone_configs"];
    if (!drones_node || !drones_node.IsSequence()) {
        throw std::runtime_error("Missing or invalid 'drone_configs' sequence in " + context);
    }
    for (const auto& drone_file_node : drones_node) {
        const std::string drone_file_raw = drone_file_node.as<std::string>();
        const std::filesystem::path drone_path = resolveRelativePath(file_path, drone_file_raw);
        composition.drone_configs.push_back(parseDroneConfig(drone_path));
    }

    const YAML::Node lidars_node = composition_node["lidar_configs"];
    if (!lidars_node || !lidars_node.IsSequence()) {
        throw std::runtime_error("Missing or invalid 'lidar_configs' sequence in " + context);
    }
    for (const auto& lidar_file_node : lidars_node) {
        const std::string lidar_file_raw = lidar_file_node.as<std::string>();
        const std::filesystem::path lidar_path = resolveRelativePath(file_path, lidar_file_raw);
        composition.lidar_configs.push_back(parseLidarConfig(lidar_path));
    }

    return composition;
}

} // namespace simulator_207610130_215664087
