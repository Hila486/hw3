#pragma once

#include <Common/IMap3D.h>

#include <vector>

namespace simulator_207610130_215664087 {

class MapsComparison {
public:
    static std::vector<double> compare(
        const common::IMap3D& hidden_map,
        const std::vector<common::IMap3D*>& output_maps);
};

} // namespace simulator_207610130_215664087
