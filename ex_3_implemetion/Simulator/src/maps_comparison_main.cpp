#include <Common/Types.h>
#include <Simulator/Map3DImpl.h>
#include <Simulator/MapsComparison.h>
#include <Simulator/NpyMapIO.h>

#include <yaml-cpp/yaml.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace simulator_207610130_215664087 {

namespace {

using namespace common;

[[nodiscard]] types::MapConfig fullExtentConfig(const NpyMapShape& shape) {
    constexpr double resolution_cm = 1.0;
    return types::MapConfig{
        types::MappingBounds{
            0.0 * x_extent[cm],
            static_cast<double>(shape.dim_x) * resolution_cm * x_extent[cm],
            0.0 * y_extent[cm],
            static_cast<double>(shape.dim_y) * resolution_cm * y_extent[cm],
            0.0 * z_extent[cm],
            static_cast<double>(shape.dim_z) * resolution_cm * z_extent[cm],
        },
        Position3D{},
        resolution_cm * cm,
    };
}

[[nodiscard]] Position3D parseOffsetNode(const YAML::Node& node) {
    if (!node) {
        return Position3D{};
    }
    const auto value = [&node](const char* key) {
        return node[key] ? node[key].as<double>() : 0.0;
    };
    return Position3D{
        value("x_offset") * x_extent[cm],
        value("y_offset") * y_extent[cm],
        value("height_offset") * z_extent[cm],
    };
}

[[nodiscard]] types::MappingBounds parseBoundsNode(const YAML::Node& node) {
    if (!node) {
        throw std::runtime_error("comparison_config map is missing 'map_boundaries'.");
    }
    const auto axis = [&node](const char* axis_key, const char* bound_key) {
        const YAML::Node axis_node = node[axis_key];
        if (!axis_node || !axis_node[bound_key]) {
            throw std::runtime_error(std::string("comparison_config boundary missing ") +
                                     axis_key + "." + bound_key);
        }
        return axis_node[bound_key].as<double>();
    };
    return types::MappingBounds{
        axis("x_boundary", "min_cm") * x_extent[cm],
        axis("x_boundary", "max_cm") * x_extent[cm],
        axis("y_boundary", "min_cm") * y_extent[cm],
        axis("y_boundary", "max_cm") * y_extent[cm],
        axis("height_boundary", "min_cm") * z_extent[cm],
        axis("height_boundary", "max_cm") * z_extent[cm],
    };
}

[[nodiscard]] types::MapConfig parseMapConfigNode(const YAML::Node& node, const char* which) {
    if (!node) {
        throw std::runtime_error(std::string("comparison_config is missing '") + which + "'.");
    }
    if (!node["map_res_cm"]) {
        throw std::runtime_error(std::string("comparison_config.") + which +
                                 " is missing 'map_res_cm'.");
    }
    const double resolution_cm = node["map_res_cm"].as<double>();
    if (resolution_cm <= 0.0) {
        throw std::runtime_error(std::string("comparison_config.") + which +
                                 ".map_res_cm must be positive.");
    }
    return types::MapConfig{
        parseBoundsNode(node["map_boundaries"]),
        parseOffsetNode(node["map_offset"]),
        resolution_cm * cm,
    };
}

struct ComparisonConfigs {
    types::MapConfig original{};
    types::MapConfig target{};
};

[[nodiscard]] ComparisonConfigs parseComparisonConfig(const std::filesystem::path& path) {
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node config = root["comparison_config"];
    if (!config) {
        throw std::runtime_error("Missing 'comparison_config' root in " + path.string());
    }
    return ComparisonConfigs{
        parseMapConfigNode(config["original"], "original"),
        parseMapConfigNode(config["target"], "target"),
    };
}

[[nodiscard]] std::filesystem::path parseConfigArgument(const std::string& argument) {
    const std::string prefix = "comparison_config=";
    if (argument.rfind(prefix, 0) == 0) {
        return std::filesystem::path{argument.substr(prefix.size())};
    }
    return std::filesystem::path{argument};
}

} // namespace

} // namespace simulator_207610130_215664087

int main(int argc, char** argv) {
    using namespace simulator_207610130_215664087;

    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <origin_map> <target_map> "
                     "[comparison_config=<path>]\n";
        return 1;
    }

    try {
        const std::filesystem::path origin_path{argv[1]};
        const std::filesystem::path target_path{argv[2]};

        auto origin_array = loadNormalizedNpyMap(origin_path);
        auto target_array = loadNormalizedNpyMap(target_path);
        const NpyMapShape origin_shape = npyMapShape(*origin_array, origin_path);
        const NpyMapShape target_shape = npyMapShape(*target_array, target_path);

        common::types::MapConfig origin_config = fullExtentConfig(origin_shape);
        common::types::MapConfig target_config = fullExtentConfig(target_shape);

        if (argc == 4) {
            const ComparisonConfigs configs = parseComparisonConfig(parseConfigArgument(argv[3]));
            origin_config = configs.original;
            target_config = configs.target;
        }

        const Map3DImpl origin_map{origin_array, origin_config};
        Map3DImpl target_map{target_array, target_config};

        const std::vector<common::IMap3D*> targets{&target_map};
        const std::vector<double> scores = MapsComparison::compare(origin_map, targets);
        if (scores.empty()) {
            throw std::runtime_error("MapsComparison returned no score.");
        }

        std::cout << scores.front() << '\n';
        return 0;
    } catch (const std::exception& exception) {
        std::cout << "-1\n";
        std::cerr << "maps_comparison error: " << exception.what() << '\n';
        return 1;
    }
}
