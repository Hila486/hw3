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
T requireValue(const YAML::Node& node, const std::string& key, const std::string& context) {
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

template <typename T>
T findValue(const YAML::Node& node, const std::vector<std::string>& keys, const T& fallback) {
    for (const auto& key : keys) {
        if (node[key]) {
            try {
                return node[key].as<T>();
            } catch (...) {}
        }
    }
    return fallback;
}

template <typename T>
T requireAnyValue(const YAML::Node& node, const std::vector<std::string>& keys, const std::string& context) {
    for (const auto& key : keys) {
        if (node[key]) {
            try {
                return node[key].as<T>();
            } catch (const std::exception& ex) {
                throw std::runtime_error("Invalid value for key '" + key + "' in " + context + ": " + ex.what());
            }
        }
    }
    throw std::runtime_error("Missing required key in " + context);
}

std::filesystem::path resolveRelativePath(
    const std::filesystem::path& base_file_path,
    const std::string& relative_or_absolute_path) {
    const std::filesystem::path parsed_path(relative_or_absolute_path);
    if (parsed_path.is_absolute()) {
        return parsed_path.lexically_normal();
    }

    const auto cwd_candidate = std::filesystem::absolute(parsed_path).lexically_normal();
    if (std::filesystem::exists(cwd_candidate)) {
        return cwd_candidate;
    }

    const auto base_dir = std::filesystem::is_directory(base_file_path) ? base_file_path : base_file_path.parent_path();
    const auto candidate1 = (base_dir / parsed_path).lexically_normal();
    if (std::filesystem::exists(candidate1)) {
        return candidate1;
    }

    const auto candidate2 = (base_dir.parent_path() / parsed_path).lexically_normal();
    if (std::filesystem::exists(candidate2)) {
        return candidate2;
    }

    return candidate1;
}

} // namespace

common::types::DroneConfigData ConfigParser::parseDroneConfig(
    const std::filesystem::path& file_path) {
    const YAML::Node root = loadYamlFile(file_path);
    const YAML::Node config = root["drone_config"] ? root["drone_config"] : root;
    const std::string context = file_path.string();

    double radius_cm = 0.0;
    if (config["dimensions_cm"]) {
        radius_cm = config["dimensions_cm"].as<double>() / 2.0;
    } else if (config["radius_cm"]) {
        radius_cm = config["radius_cm"].as<double>();
    } else if (config["radius"]) {
        radius_cm = config["radius"].as<double>();
    } else {
        throw std::runtime_error("Missing drone dimension/radius in " + context);
    }

    const double max_rotate_deg =
        requireAnyValue<double>(config, {"max_rotate_deg", "max_rotate", "max_rotation_speed"}, context);
    const double max_advance_cm =
        requireAnyValue<double>(config, {"max_advance_cm", "max_advance", "max_horizontal_speed"}, context);
    const double max_elevate_cm =
        requireAnyValue<double>(config, {"max_elevate_cm", "max_elevate", "max_vertical_speed"}, context);

    common::types::DroneConfigData drone_config;
    drone_config.radius = radius_cm * x_extent[cm];
    drone_config.max_rotate = max_rotate_deg * horizontal_angle[deg];
    drone_config.max_advance = max_advance_cm * x_extent[cm];
    drone_config.max_elevate = max_elevate_cm * z_extent[cm];

    return drone_config;
}

common::types::LidarConfigData ConfigParser::parseLidarConfig(
    const std::filesystem::path& file_path) {
    const YAML::Node root = loadYamlFile(file_path);
    const YAML::Node config = root["lidar_config"] ? root["lidar_config"] : root;
    const std::string context = file_path.string();

    common::types::LidarConfigData lidar_config;
    const double z_min = requireAnyValue<double>(config, {"z_min_cm", "z_min"}, context);
    const double z_max = requireAnyValue<double>(config, {"z_max_cm", "z_max", "max_distance"}, context);
    const double d = findValue<double>(config, {"d_cm", "d"}, 1.0);
    const std::size_t fov_circles = findValue<std::size_t>(config, {"fov_circles"}, 1);

    lidar_config.z_min = z_min * x_extent[cm];
    lidar_config.z_max = z_max * x_extent[cm];
    lidar_config.d = d * x_extent[cm];
    lidar_config.fov_circles = fov_circles;

    return lidar_config;
}

common::types::MissionConfigData ConfigParser::parseMissionConfig(
    const std::filesystem::path& file_path) {
    const YAML::Node root = loadYamlFile(file_path);
    const YAML::Node config = root["mission_config"] ? root["mission_config"] : root;
    const std::string context = file_path.string();

    common::types::MissionConfigData mission_config;
    mission_config.max_steps = requireValue<std::size_t>(config, "max_steps", context);

    const double gps_res = requireAnyValue<double>(config, {"gps_resolution_cm", "gps_resolution"}, context);
    mission_config.gps_resolution = gps_res * x_extent[cm];

    mission_config.output_mapping_resolution_factor =
        optionalValue<double>(config, "output_mapping_resolution_factor", 1.0);

    YAML::Node bounds_node = config["boundaries"] ? config["boundaries"] : config["mission_bounds"];
    if (!bounds_node) {
        throw std::runtime_error("Missing boundaries in " + context);
    }

    auto parseAxis = [&](const std::vector<std::string>& axis_keys) {
        YAML::Node axis_node;
        for (const auto& k : axis_keys) {
            if (bounds_node[k]) {
                axis_node = bounds_node[k];
                break;
            }
        }
        if (!axis_node) {
            throw std::runtime_error("Missing axis boundary in " + context);
        }
        const double min_val = findValue<double>(axis_node, {"min_cm", "min"}, 0.0);
        const double max_val = findValue<double>(axis_node, {"max_cm", "max"}, 0.0);
        return std::make_pair(min_val, max_val);
    };

    const auto [x_min, x_max] = parseAxis({"x_boundary", "x"});
    const auto [y_min, y_max] = parseAxis({"y_boundary", "y"});
    const auto [z_min, z_max] = parseAxis({"height_boundary", "z_boundary", "z"});

    mission_config.mission_bounds = common::types::MappingBounds{
        x_min * x_extent[cm],
        x_max * x_extent[cm],
        y_min * y_extent[cm],
        y_max * y_extent[cm],
        z_min * z_extent[cm],
        z_max * z_extent[cm]
    };

    return mission_config;
}

SimulationConfigData ConfigParser::parseSimulationConfig(
    const std::filesystem::path& file_path) {
    const YAML::Node root = loadYamlFile(file_path);
    const YAML::Node config = root["simulation_config"] ? root["simulation_config"] : root;
    const std::string context = file_path.string();

    SimulationConfigData sim_config;
    const std::string map_file_raw = requireValue<std::string>(config, "map_filename", context);
    sim_config.map_filename = resolveRelativePath(file_path, map_file_raw);

    const double map_res = requireAnyValue<double>(config, {"map_resolution_cm", "map_resolution"}, context);
    sim_config.map_resolution = map_res * x_extent[cm];

    const YAML::Node offset_node = config["map_axes_offset"];
    if (offset_node) {
        const double ox = findValue<double>(offset_node, {"x_offset", "x"}, 0.0);
        const double oy = findValue<double>(offset_node, {"y_offset", "y"}, 0.0);
        const double oz = findValue<double>(offset_node, {"height_offset", "z_offset", "z"}, 0.0);
        sim_config.map_axes_offset = Position3D{ox * x_extent[cm], oy * y_extent[cm], oz * z_extent[cm]};
    }

    const YAML::Node pos_node = config["initial_drone_position"];
    if (pos_node) {
        const double px = findValue<double>(pos_node, {"x_cm", "x"}, 0.0);
        const double py = findValue<double>(pos_node, {"y_cm", "y"}, 0.0);
        const double pz = findValue<double>(pos_node, {"height_cm", "z_cm", "z"}, 0.0);
        sim_config.initial_drone_position = Position3D{px * x_extent[cm], py * y_extent[cm], pz * z_extent[cm]};
    }

    const double init_angle = findValue<double>(config, {"initial_angle_deg", "initial_angle"}, 0.0);
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
        const std::string sim_config_file_raw = requireValue<std::string>(sim_entry, "simulation_config", context);
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
