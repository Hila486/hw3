#pragma once

#include <TinyNPY.h>

#include <cstddef>
#include <filesystem>
#include <memory>

namespace user_common_207610130_215664087 {

/**
 * @struct NpyMapShape
 * @brief Represents 3D voxel grid dimensions (depth, height, width) in row-major layout.
 */
struct NpyMapShape {
    std::size_t depth = 0;   ///< Z-axis dimension
    std::size_t height = 0;  ///< Y-axis dimension
    std::size_t width = 0;   ///< X-axis dimension
};

/**
 * @brief Loads a 3D .npy map file and normalizes it to int32 format for map operations.
 * @param map_filename Path to the .npy file.
 * @return Shared pointer to an NpyArray instance containing normalized int32 map data.
 */
[[nodiscard]] std::shared_ptr<NpyArray>
loadNormalizedNpyMap(const std::filesystem::path& map_filename);

/**
 * @brief Validates layout and returns dimensions of a loaded NpyArray map.
 * @param map Reference to loaded NpyArray.
 * @param source Source file path context for error messages.
 * @return NpyMapShape containing depth, height, and width.
 */
[[nodiscard]] NpyMapShape npyMapShape(const NpyArray& map,
                                      const std::filesystem::path& source);

/**
 * @brief Creates a new owned int32 NpyArray filled with a specified initial value.
 * @param shape Grid dimensions.
 * @param fill_value Voxel value to populate across all grid cells.
 * @return Shared pointer to newly created NpyArray.
 */
[[nodiscard]] std::shared_ptr<NpyArray>
makeFilledIntNpyArray(const NpyMapShape& shape, int fill_value);

/**
 * @brief Validates that a hidden input map contains strictly legal 0 (Empty) and 1 (Occupied) values.
 * @param map Reference to loaded input map NpyArray.
 * @param source Source file path context.
 */
void validateInputMapValues(const NpyArray& map, const std::filesystem::path& source);

} // namespace user_common_207610130_215664087
