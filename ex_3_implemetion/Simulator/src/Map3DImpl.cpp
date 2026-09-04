#include <Simulator/Map3DImpl.h>

#include <cmath>
#include <utility>

namespace simulator_207610130_215664087 {


// Stores the map data and configuration.
// std::move transfers ownership where possible instead of copying.
Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray3D> data, common::types::MapConfig config)
    : data_(std::move(data)), config_(std::move(config)) {}

// Checks whether a real-world position is inside the configured map boundaries.
bool Map3DImpl::isInBounds(const Position3D& position) const {
    const auto& bounds = config_.boundaries;
    return !(position.x < bounds.min_x || position.x > bounds.max_x ||
             position.y < bounds.min_y || position.y > bounds.max_y ||
             position.z < bounds.min_height || position.z > bounds.max_height);
}

// Converts a real-world Position3D into an index in the flat NPY data array.
// Returns false if the position cannot be represented in the map.
bool Map3DImpl::positionToFlatIndex(const Position3D& pos, std::size_t& flat_idx) const {
    if (!isInBounds(pos)) {
        return false;
    }
    // Convert the map resolution to centimeters.
    const double res_cm = config_.resolution.force_numerical_value_in(cm);
    // Invalid resolution cannot be used for coordinate conversion.
    if (res_cm <= 0.0) {
        return false;
    }
     // Convert world coordinates into coordinates relative to the stored NPY map.
    const double map_x_cm = pos.x.force_numerical_value_in(cm) +
                            config_.offset.x.force_numerical_value_in(cm);
    const double map_y_cm = pos.y.force_numerical_value_in(cm) +
                            config_.offset.y.force_numerical_value_in(cm);
    const double map_z_cm = pos.z.force_numerical_value_in(cm) +
                            config_.offset.z.force_numerical_value_in(cm);

    // Convert physical coordinates into integer voxel coordinates.
    const double gx_d = std::floor(map_x_cm / res_cm);
    const double gy_d = std::floor(map_y_cm / res_cm);
    const double gz_d = std::floor(map_z_cm / res_cm);

    // Negative voxel indices are invalid.
    if (gx_d < 0.0 || gy_d < 0.0 || gz_d < 0.0) {
        return false;
    }

    const auto gx = static_cast<std::size_t>(gx_d);
    const auto gy = static_cast<std::size_t>(gy_d);
    const auto gz = static_cast<std::size_t>(gz_d);


    if (gx >= data_->shape[0] || gy >= data_->shape[1] || gz >= data_->shape[2]) {
        return false;
    }
    // Convert the 3D voxel coordinates into one flat array index.
    flat_idx = gx * data_->shape[1] * data_->shape[2] + gy * data_->shape[2] + gz;
    return true;
}

// Returns the occupancy state of the voxel at the requested position.
common::types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& position) const {
    if (!isInBounds(position)) {
        return common::types::VoxelOccupancy::OutOfBounds;
    }

    std::size_t flat_idx = 0;
    // Translate the position into the internal array index.
    if (!positionToFlatIndex(position, flat_idx)) {
        return common::types::VoxelOccupancy::OutOfBounds;
    }

    const int val = data_->data[flat_idx];
    // Convert the raw integer stored in the NPY file into the enum used by the system.
    if (val == 0) {
        return common::types::VoxelOccupancy::Empty;
    }
    if (val > 0) {
        return common::types::VoxelOccupancy::Occupied;
    }
    if (val == -3) {
        return common::types::VoxelOccupancy::PotentiallyOccupied;
    }
    if (val == -2) {
        return common::types::VoxelOccupancy::OutOfBounds;
    }
    return common::types::VoxelOccupancy::Unmapped;
}

// Returns a copy of the map configuration.
common::types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

// Updates the occupancy value of a voxel.
// Invalid/out-of-bounds positions are ignored.
void Map3DImpl::set(const Position3D& position, common::types::VoxelOccupancy value) {
    std::size_t flat_idx = 0;
    if (!positionToFlatIndex(position, flat_idx)) {
        return;
    }

    data_->data[flat_idx] = static_cast<int>(value);
}

// Saves the current NPY map data to a file.
void Map3DImpl::save(const std::filesystem::path& file_path) const {
    saveNpyMap(file_path, *data_);
}
// Read-only access to the underlying NPY array
std::shared_ptr<const NpyArray3D> Map3DImpl::rawData() const {
    return data_;
}
// Mutable access to the underlying NPY array.
std::shared_ptr<NpyArray3D> Map3DImpl::rawData() {
    return data_;
}

} // namespace simulator_207610130_215664087
