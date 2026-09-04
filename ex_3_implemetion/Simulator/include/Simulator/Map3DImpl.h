#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>
#include <Simulator/NpyMapIO.h>

#include <filesystem>
#include <memory>
#include <optional>

namespace simulator_207610130_215664087 {

using namespace common;

// Concrete implementation of IMutableMap3D used by the simulator.
// Wraps the map's NPY data and provides access using Position3D coordinates.
class Map3DImpl final : public common::IMutableMap3D {
public:
    // Creates a map using the given voxel data and map configuration.
    Map3DImpl(std::shared_ptr<NpyArray3D> data, common::types::MapConfig config);

    // Returns the occupancy value at the given position.
    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const Position3D& position) const override;
    // Returns the map configuration.
    [[nodiscard]] common::types::MapConfig getMapConfig() const override;
    // Checks whether the given position is inside the map boundaries.
    [[nodiscard]] bool isInBounds(const Position3D& position) const override;

    //[nodiscard] - means the return value of this function is important; warn  if someone calls it and ignores the result.

    // Sets the occupancy value of the voxel at the given position.
    void set(const Position3D& position, common::types::VoxelOccupancy value) override;
    // Saves the current map data to an NPY file.
    void save(const std::filesystem::path& file_path) const override;

    // Gives read-only access to the underlying NPY array.
    [[nodiscard]] std::shared_ptr<const NpyArray3D> rawData() const;
    //Gives mutable access to the underlying NPY array.
    [[nodiscard]] std::shared_ptr<NpyArray3D> rawData();

private:
    // Converts a Position3D into the corresponding flat index
    // in the underlying NPY array. Returns false if the position is invalid.
    [[nodiscard]] bool positionToFlatIndex(const Position3D& pos, std::size_t& flat_idx) const;

    //Actual voxel data of the map.
    std::shared_ptr<NpyArray3D> data_;
    // Map dimensions, resolution and coordinate information.
    common::types::MapConfig config_;
};

} // namespace simulator_207610130_215664087
