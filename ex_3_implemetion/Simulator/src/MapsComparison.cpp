#include <Simulator/MapsComparison.h>

#include <cmath>

namespace simulator_207610130_215664087 {

using namespace common;

std::vector<double> MapsComparison::compare(
    const common::IMap3D& hidden_map,
    const std::vector<common::IMap3D*>& output_maps) {

    std::vector<double> scores;
    scores.reserve(output_maps.size());

    for (const auto* out_map : output_maps) {
        if (!out_map) {
            scores.push_back(0.0);
            continue;
        }

        const auto config = out_map->getMapConfig();
        const double res_cm = config.resolution.force_numerical_value_in(cm);
        if (res_cm <= 0.0) {
            scores.push_back(0.0);
            continue;
        }

        const double min_x = config.boundaries.min_x.force_numerical_value_in(cm);
        const double max_x = config.boundaries.max_x.force_numerical_value_in(cm);
        const double min_y = config.boundaries.min_y.force_numerical_value_in(cm);
        const double max_y = config.boundaries.max_y.force_numerical_value_in(cm);
        const double min_z = config.boundaries.min_height.force_numerical_value_in(cm);
        const double max_z = config.boundaries.max_height.force_numerical_value_in(cm);

        std::size_t total_voxels = 0;
        std::size_t matched_voxels = 0;

        for (double x = min_x + res_cm * 0.5; x < max_x; x += res_cm) {
            for (double y = min_y + res_cm * 0.5; y < max_y; y += res_cm) {
                for (double z = min_z + res_cm * 0.5; z < max_z; z += res_cm) {
                    Position3D pos{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
                    const auto hidden_val = hidden_map.atVoxel(pos);
                    const auto out_val = out_map->atVoxel(pos);

                    ++total_voxels;
                    if (hidden_val == out_val) {
                        ++matched_voxels;
                    }
                }
            }
        }

        if (total_voxels == 0) {
            scores.push_back(100.0);
        } else {
            const double score = (static_cast<double>(matched_voxels) / static_cast<double>(total_voxels)) * 100.0;
            scores.push_back(score);
        }
    }

    return scores;
}

} // namespace simulator_207610130_215664087
