#include <Simulator/MapsComparison.h>

#include <cmath>

namespace simulator_207610130_215664087 {

using namespace common;
// Compares one or more output maps against the hidden real map
// and returns a percentage score for each output map.
std::vector<double> MapsComparison::compare(
    const common::IMap3D& hidden_map,
    const std::vector<common::IMap3D*>& output_maps) {

    std::vector<double> scores;
    // Reserve space in advance for one score per output map.
    scores.reserve(output_maps.size());

    // Compare each output map separately.
    for (const auto* out_map : output_maps) {
        // Invalid map pointer gets a score of 0.
        if (!out_map) {
            scores.push_back(0.0);
            continue;
        }
        // Use the output map configuration to know
        // its resolution and mission boundaries.
        const auto config = out_map->getMapConfig();
        const double res_cm = config.resolution.force_numerical_value_in(cm);
        // Invalid resolution cannot be compared correctly.
        if (res_cm <= 0.0) {
            scores.push_back(0.0);
            continue;
        }
        // Convert the map boundaries to centimeters.
        const double min_x = config.boundaries.min_x.force_numerical_value_in(cm);
        const double max_x = config.boundaries.max_x.force_numerical_value_in(cm);
        const double min_y = config.boundaries.min_y.force_numerical_value_in(cm);
        const double max_y = config.boundaries.max_y.force_numerical_value_in(cm);
        const double min_z = config.boundaries.min_height.force_numerical_value_in(cm);
        const double max_z = config.boundaries.max_height.force_numerical_value_in(cm);

        std::size_t total_voxels = 0;
        std::size_t matched_voxels = 0;
        
        // Visit the center of every voxel inside the output map boundaries.
        for (double x = min_x + res_cm * 0.5; x < max_x; x += res_cm) {
            for (double y = min_y + res_cm * 0.5; y < max_y; y += res_cm) {
                for (double z = min_z + res_cm * 0.5; z < max_z; z += res_cm) {
                    Position3D pos{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};

                    // Read the real value and the mapped value
                    // at the same physical position.
                    const auto hidden_val = hidden_map.atVoxel(pos);
                    const auto out_val = out_map->atVoxel(pos);

                    ++total_voxels;
                    // Exact occupancy match counts as correct.
                    if (hidden_val == out_val) {
                        ++matched_voxels;
                    }
                }
            }
        }
        // Avoid division by zero if no voxels were checked.
        if (total_voxels == 0) {
            scores.push_back(100.0);
        } else {
            // Score = percentage of correctly mapped voxels.
            const double score = (static_cast<double>(matched_voxels) / static_cast<double>(total_voxels)) * 100.0;
            scores.push_back(score);
        }
    }

    return scores;
}

} // namespace simulator_207610130_215664087
