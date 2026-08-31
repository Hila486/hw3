#pragma once

#include <Common/IMutableMap3D.h>
#include <TinyNPY.h>

#include <filesystem>
#include <memory>

namespace user_common_207610130_215664087 {

/**
 * @class Map3DImpl
 * @brief Concrete implementation of a 3D voxel grid map.
 * 
 * Manages 3D voxel spatial occupancy using an underlying NPY array buffer.
 * Implements common::IMutableMap3D to support reading, updating, bounds checking,
 * and saving map data to NumPy file format (.npy).
 */
class Map3DImpl final : public common::IMutableMap3D {
public:
    /**
     * @brief Constructs a 3D map from an existing NpyArray pointer with default config.
     * @param map_ptr Shared pointer to the raw NPY array data.
     */
    explicit Map3DImpl(std::shared_ptr<NpyArray> map_ptr);

    /**
     * @brief Constructs a 3D map with explicit configuration (boundaries, resolution, offsets).
     * @param map_ptr Shared pointer to the raw NPY array data.
     * @param map_config Map configuration data defining bounds and resolution.
     */
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const common::types::MapConfig map_config);

    /**
     * @brief Retrieves voxel occupancy state at a specific 3D spatial coordinate.
     * @param pos 3D world coordinate.
     * @return VoxelOccupancy enum (Occupied, Empty, Unmapped, OutOfBounds, etc.)
     */
    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const Position3D& pos) const override;

    /**
     * @brief Returns the map geometry metadata (resolution, offsets, boundaries).
     */
    [[nodiscard]] common::types::MapConfig getMapConfig() const override;

    /**
     * @brief Checks if a given 3D position falls within the map's configured spatial boundaries.
     * @param pos 3D coordinate to test.
     * @return True if within bounds, false otherwise.
     */
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override;

    /**
     * @brief Sets the occupancy state of a voxel at a specific 3D coordinate.
     * @param pos Target 3D coordinate.
     * @param value Occupancy state to record.
     */
    void set(const Position3D& pos, common::types::VoxelOccupancy value) override;

    /**
     * @brief Saves the current map grid to a NumPy (.npy) file format on disk.
     * @param output_path File path where the .npy file will be written.
     */
    void save(const std::filesystem::path& output_path) const override;

private:
    std::shared_ptr<NpyArray> map_;        ///< Internal raw NPY array storage
    common::types::MapConfig config_;       ///< Map metadata (resolution, offsets, bounds)
};

} // namespace user_common_207610130_215664087
