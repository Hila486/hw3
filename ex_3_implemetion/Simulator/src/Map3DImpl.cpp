#include <Simulator/Map3DImpl.h>

#include <cmath>
#include <utility>

namespace simulator_207610130_215664087 {

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray3D> data, common::types::MapConfig config)
    : data_(std::move(data)), config_(std::move(config)) {}

std::optional<std::tuple<std::size_t, std::size_t, std::size_t>>
Map3DImpl::worldToGrid(const Position3D& position) const {
    const double res_cm = config_.resolution.force_numerical_value_in(cm);
    if (res_cm <= 0.0) {
        return std::nullopt;
    }

    const double min_x = config_.boundaries.x.min.force_numerical_value_in(cm);
    const double max_x = config_.boundaries.x.max.force_numerical_value_in(cm);
    const double min_y = config_.boundaries.y.min.force_numerical_value_in(cm);
    const double max_y = config_.boundaries.y.max.force_numerical_value_in(cm);
    const double min_z = config_.boundaries.z.min.force_numerical_value_in(cm);
    const double max_z = config_.boundaries.z.max.force_numerical_value_in(cm);

    const double pos_x = position.x.force_numerical_value_in(cm);
    const double pos_y = position.y.force_numerical_value_in(cm);
    const double pos_z = position.z.force_numerical_value_in(cm);

    if (pos_x < min_x || pos_x > max_x ||
        pos_y < min_y || pos_y > max_y ||
        pos_z < min_z || pos_z > max_z) {
        return std::nullopt;
    }

    const double offset_x = config_.offset.x.force_numerical_value_in(cm);
    const double offset_y = config_.offset.y.force_numerical_value_in(cm);
    const double offset_z = config_.offset.z.force_numerical_value_in(cm);

    const double local_x = pos_x + offset_x;
    const double local_y = pos_y + offset_y;
    const double local_z = pos_z + offset_z;

    if (local_x < 0.0 || local_y < 0.0 || local_z < 0.0) {
        return std::nullopt;
    }

    const auto gx = static_cast<std::size_t>(std::floor(local_x / res_cm));
    const auto gy = static_cast<std::size_t>(std::floor(local_y / res_cm));
    const auto gz = static_cast<std::size_t>(std::floor(local_z / res_cm));

    if (gx >= data_->shape[0] || gy >= data_->shape[1] || gz >= data_->shape[2]) {
        return std::nullopt;
    }

    return std::make_tuple(gx, gy, gz);
}

common::types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& position) const {
    const auto grid_idx = worldToGrid(position);
    if (!grid_idx) {
        return common::types::VoxelOccupancy::Unmapped;
    }

    const auto [gx, gy, gz] = *grid_idx;
    const int val = data_->data[gx * data_->shape[1] * data_->shape[2] + gy * data_->shape[2] + gz];

    switch (val) {
        case 0:  return common::types::VoxelOccupancy::Empty;
        case 1:  return common::types::VoxelOccupancy::Occupied;
        default: return common::types::VoxelOccupancy::Unmapped;
    }
}

common::types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

void Map3DImpl::setVoxel(const Position3D& position, common::types::VoxelOccupancy value) {
    const auto grid_idx = worldToGrid(position);
    if (!grid_idx) {
        return;
    }

    const auto [gx, gy, gz] = *grid_idx;
    int int_val = 2; // Unmapped
    if (value == common::types::VoxelOccupancy::Empty) {
        int_val = 0;
    } else if (value == common::types::VoxelOccupancy::Occupied) {
        int_val = 1;
    }

    data_->data[gx * data_->shape[1] * data_->shape[2] + gy * data_->shape[2] + gz] = int_val;
}

void Map3DImpl::save(const std::filesystem::path& file_path) const {
    saveNpyMap(file_path, *data_);
}

std::shared_ptr<const NpyArray3D> Map3DImpl::rawData() const {
    return data_;
}

std::shared_ptr<NpyArray3D> Map3DImpl::rawData() {
    return data_;
}

} // namespace simulator_207610130_215664087
