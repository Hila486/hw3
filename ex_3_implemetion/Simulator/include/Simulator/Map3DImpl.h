#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>
#include <Simulator/NpyMapIO.h>

#include <filesystem>
#include <memory>
#include <optional>
#include <tuple>

namespace simulator_207610130_215664087 {

using namespace common;

class Map3DImpl final : public common::IMutableMap3D {
public:
    Map3DImpl(std::shared_ptr<NpyArray3D> data, common::types::MapConfig config);

    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const Position3D& position) const override;
    [[nodiscard]] common::types::MapConfig getMapConfig() const override;
    void setVoxel(const Position3D& position, common::types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& file_path) const override;

    [[nodiscard]] std::shared_ptr<const NpyArray3D> rawData() const;
    [[nodiscard]] std::shared_ptr<NpyArray3D> rawData();

private:
    [[nodiscard]] std::optional<std::tuple<std::size_t, std::size_t, std::size_t>>
    worldToGrid(const Position3D& position) const;

    std::shared_ptr<NpyArray3D> data_;
    common::types::MapConfig config_;
};

} // namespace simulator_207610130_215664087
