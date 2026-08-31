#pragma once

#include <Common/IMap3D.h>
#include <Common/Types.h>

#include <vector>

namespace user_common_207610130_215664087 {

/**
 * @class MapsComparison
 * @brief Accuracy scoring utility comparing target mapped voxel grids against a ground-truth map.
 */
class MapsComparison {
public:
    /**
     * @brief Computes accuracy percentage score (0-100) comparing target maps against an origin ground-truth map.
     * @param origin The ground-truth IMap3D reference map.
     * @param targets Array of pointers to target IMap3D instances to score.
     * @return Vector of accuracy percentage scores corresponding to target maps.
     */
    [[nodiscard]] static std::vector<double> compare(const common::IMap3D& origin,
                                                     const std::vector<common::IMap3D*> targets);
};

} // namespace user_common_207610130_215664087
