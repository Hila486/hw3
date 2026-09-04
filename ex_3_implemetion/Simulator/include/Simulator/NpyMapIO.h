#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <vector>

namespace simulator_207610130_215664087 {

struct NpyArray3D {
    std::array<std::size_t, 3> shape{0, 0, 0};
    std::vector<int> data{};
};

struct NpyMapShape {
    std::size_t dim_x = 0;
    std::size_t dim_y = 0;
    std::size_t dim_z = 0;
};

std::shared_ptr<NpyArray3D> loadNormalizedNpyMap(const std::filesystem::path& file_path);

void validateInputMapValues(const NpyArray3D& array, const std::filesystem::path& file_path);

NpyMapShape npyMapShape(const NpyArray3D& array, const std::filesystem::path& file_path);

std::shared_ptr<NpyArray3D> makeFilledIntNpyArray(const NpyMapShape& shape, int fill_value);

void saveNpyMap(const std::filesystem::path& file_path, const NpyArray3D& map_array);

} // namespace simulator_207610130_215664087
