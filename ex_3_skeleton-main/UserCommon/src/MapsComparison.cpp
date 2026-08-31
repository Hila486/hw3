#include <UserCommon/MapsComparison.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace user_common_207610130_215664087 {

namespace {

[[nodiscard]] double xCm(XLength value) { return value.force_numerical_value_in(cm); }
[[nodiscard]] double yCm(YLength value) { return value.force_numerical_value_in(cm); }
[[nodiscard]] double zCm(ZLength value) { return value.force_numerical_value_in(cm); }
[[nodiscard]] double pCm(PhysicalLength value) { return value.force_numerical_value_in(cm); }

struct RegionCm {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    double min_z = 0.0;
    double max_z = 0.0;
    bool valid = false;
};

[[nodiscard]] RegionCm overlapRegion(const common::types::MapConfig& a, const common::types::MapConfig& b) {
    RegionCm region;
    region.min_x = std::max(xCm(a.boundaries.min_x), xCm(b.boundaries.min_x));
    region.max_x = std::min(xCm(a.boundaries.max_x), xCm(b.boundaries.max_x));
    region.min_y = std::max(yCm(a.boundaries.min_y), yCm(b.boundaries.min_y));
    region.max_y = std::min(yCm(a.boundaries.max_y), yCm(b.boundaries.max_y));
    region.min_z = std::max(zCm(a.boundaries.min_height), zCm(b.boundaries.min_height));
    region.max_z = std::min(zCm(a.boundaries.max_height), zCm(b.boundaries.max_height));
    region.valid = region.min_x <= region.max_x &&
                   region.min_y <= region.max_y &&
                   region.min_z <= region.max_z;
    return region;
}

[[nodiscard]] int centerCount(double lo, double hi, double res) {
    const double first_center = lo + 0.5 * res;
    if (first_center > hi) {
        return 0;
    }
    return static_cast<int>(std::floor((hi - first_center) / res + 1.0e-9)) + 1;
}

[[nodiscard]] double scoreOne(const common::IMap3D& origin, const common::IMap3D& target) {
    const common::types::MapConfig origin_config = origin.getMapConfig();
    const common::types::MapConfig target_config = target.getMapConfig();

    const RegionCm region = overlapRegion(origin_config, target_config);
    if (!region.valid) {
        return 0.0;
    }

    double resolution_cm = pCm(origin_config.resolution);
    if (resolution_cm <= 0.0) {
        resolution_cm = pCm(target_config.resolution);
    }
    if (resolution_cm <= 0.0) {
        return 0.0;
    }

    const int nx = centerCount(region.min_x, region.max_x, resolution_cm);
    const int ny = centerCount(region.min_y, region.max_y, resolution_cm);
    const int nz = centerCount(region.min_z, region.max_z, resolution_cm);

    long long total = 0;
    long long correct = 0;

    for (int iz = 0; iz < nz; ++iz) {
        const double cz = region.min_z + (static_cast<double>(iz) + 0.5) * resolution_cm;
        for (int iy = 0; iy < ny; ++iy) {
            const double cy = region.min_y + (static_cast<double>(iy) + 0.5) * resolution_cm;
            for (int ix = 0; ix < nx; ++ix) {
                const double cx = region.min_x + (static_cast<double>(ix) + 0.5) * resolution_cm;

                const Position3D position{
                    cx * x_extent[cm],
                    cy * y_extent[cm],
                    cz * z_extent[cm],
                };

                const common::types::VoxelOccupancy expected = origin.atVoxel(position);
                const common::types::VoxelOccupancy actual = target.atVoxel(position);

                ++total;
                if (expected == actual) {
                    ++correct;
                }
            }
        }
    }

    if (total == 0) {
        return 0.0;
    }

    return 100.0 * static_cast<double>(correct) / static_cast<double>(total);
}

} // namespace

std::vector<double> MapsComparison::compare(const common::IMap3D& origin,
                                            const std::vector<common::IMap3D*> targets) {
    std::vector<double> scores;
    scores.reserve(targets.size());

    for (const common::IMap3D* target : targets) {
        if (target == nullptr) {
            scores.push_back(0.0);
            continue;
        }
        scores.push_back(scoreOne(origin, *target));
    }

    return scores;
}

} // namespace user_common_207610130_215664087
