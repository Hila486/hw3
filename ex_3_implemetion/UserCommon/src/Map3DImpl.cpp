#include <UserCommon/Map3DImpl.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

namespace user_common_207610130_215664087 {

namespace {

/// Represents 3D voxel grid dimensions (depth=Z, height=Y, width=X).
struct MapShape {
    std::size_t depth = 0;
    std::size_t height = 0;
    std::size_t width = 0;
};

/// Represents 3D discrete voxel grid indices.
struct VoxelIndex {
    std::size_t x = 0;
    std::size_t y = 0;
    std::size_t z = 0;
};

/**
 * @brief Extracts map shape dimensions from a row-major 3D NpyArray.
 * @param map Raw NpyArray map data.
 * @param shape Output shape structure populated on success.
 * @return True if valid 3D row-major array, false otherwise.
 */
[[nodiscard]] bool getMapShape(const NpyArray& map, MapShape& shape) {
    if (map.IsEmpty() || map.ColMajor()) {
        return false;
    }

    const NpyArray::shape_t& npy_shape = map.Shape();
    if (npy_shape.size() != 3) {
        return false;
    }

    shape.depth = npy_shape[0];
    shape.height = npy_shape[1];
    shape.width = npy_shape[2];

    if (shape.depth == 0 || shape.height == 0 || shape.width == 0) {
        return false;
    }

    return map.NumValue() == shape.depth * shape.height * shape.width;
}

/**
 * @brief Calculates 1D flattened array index from 3D voxel grid indices.
 * Row-major index formula: flat = z * (height * width) + y * width + x
 */
[[nodiscard]] std::size_t flatIndex(const VoxelIndex& index, const MapShape& shape) {
    return index.z * (shape.height * shape.width) + index.y * shape.width + index.x;
}

/**
 * @brief Converts a 3D world space coordinate (Position3D) to a 1D flat array index.
 * Handles boundary validation, offset shift, and resolution scaling.
 */
[[nodiscard]] bool positionToFlatIndex(const NpyArray& map,
                                       const common::types::MapConfig& config,
                                       const Position3D& pos,
                                       std::size_t& output_index) {
    MapShape shape;
    if (!getMapShape(map, shape)) {
        return false;
    }

    // Check bounds
    const common::types::MappingBounds& bounds = config.boundaries;
    if (pos.x < bounds.min_x || pos.x > bounds.max_x ||
        pos.y < bounds.min_y || pos.y > bounds.max_y ||
        pos.z < bounds.min_height || pos.z > bounds.max_height) {
        return false;
    }

    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    if (resolution_cm <= 0.0) {
        return false;
    }

    // Translate 3D position by map axis offset
    const double map_x_cm = pos.x.force_numerical_value_in(cm) +
                            config.offset.x.force_numerical_value_in(cm);
    const double map_y_cm = pos.y.force_numerical_value_in(cm) +
                            config.offset.y.force_numerical_value_in(cm);
    const double map_z_cm = pos.z.force_numerical_value_in(cm) +
                            config.offset.z.force_numerical_value_in(cm);

    // Compute voxel cell index
    const double x_index_value = std::floor(map_x_cm / resolution_cm);
    const double y_index_value = std::floor(map_y_cm / resolution_cm);
    const double z_index_value = std::floor(map_z_cm / resolution_cm);

    if (x_index_value < 0.0 || y_index_value < 0.0 || z_index_value < 0.0) {
        return false;
    }

    if (x_index_value >= static_cast<double>(shape.width) ||
        y_index_value >= static_cast<double>(shape.height) ||
        z_index_value >= static_cast<double>(shape.depth)) {
        return false;
    }

    const VoxelIndex voxel_index{
        static_cast<std::size_t>(x_index_value),
        static_cast<std::size_t>(y_index_value),
        static_cast<std::size_t>(z_index_value),
    };

    output_index = flatIndex(voxel_index, shape);
    return true;
}

/**
 * @brief Converts raw integer code from NpyArray to VoxelOccupancy enum.
 */
[[nodiscard]] common::types::VoxelOccupancy rawToVoxelOccupancy(int raw_value) {
    switch (raw_value) {
        case -3:
            return common::types::VoxelOccupancy::PotentiallyOccupied;
        case -2:
            return common::types::VoxelOccupancy::OutOfBounds;
        case -1:
            return common::types::VoxelOccupancy::Unmapped;
        case 0:
            return common::types::VoxelOccupancy::Empty;
        case 1:
            return common::types::VoxelOccupancy::Occupied;
        default:
            throw std::runtime_error("Invalid raw voxel occupancy value.");
    }
}

/**
 * @brief Reads raw int value at flat index from NpyArray buffer.
 */
[[nodiscard]] int readRawValue(const NpyArray& map, std::size_t index) {
    const int raw_type = static_cast<int>(map.Type());
    const char type = static_cast<char>(raw_type < 0 ? -raw_type : raw_type);
    if (type != 'i' || map.SizeValueBytes() != sizeof(int)) {
        throw std::runtime_error("Unsupported NPY map data type.");
    }

    const int* data = map.Data<int>();
    if (data == nullptr) {
        throw std::runtime_error("Failed to access NPY map data.");
    }

    return data[index];
}

/**
 * @brief Writes raw int value at flat index into NpyArray buffer.
 */
void writeRawValue(NpyArray& map, std::size_t index, int raw_value) {
    const int raw_type = static_cast<int>(map.Type());
    const char type = static_cast<char>(raw_type < 0 ? -raw_type : raw_type);
    if (type != 'i' || map.SizeValueBytes() != sizeof(int)) {
        throw std::runtime_error("Unsupported NPY map data type.");
    }

    int* data = map.Data<int>();
    if (data == nullptr) {
        throw std::runtime_error("Failed to access NPY map data.");
    }

    data[index] = raw_value;
}

} // namespace

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), common::types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const common::types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
}

common::types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    std::size_t index = 0;
    if (!positionToFlatIndex(*map_, config_, pos, index)) {
        return common::types::VoxelOccupancy::OutOfBounds;
    }

    return rawToVoxelOccupancy(readRawValue(*map_, index));
}

common::types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    const common::types::MappingBounds& bounds = config_.boundaries;
    return pos.x >= bounds.min_x && pos.x <= bounds.max_x &&
           pos.y >= bounds.min_y && pos.y <= bounds.max_y &&
           pos.z >= bounds.min_height && pos.z <= bounds.max_height;
}

void Map3DImpl::set(const Position3D& pos, common::types::VoxelOccupancy value) {
    std::size_t index = 0;
    if (!positionToFlatIndex(*map_, config_, pos, index)) {
        return;
    }

    writeRawValue(*map_, index, static_cast<int>(value));
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    const std::filesystem::path parent_path = path.parent_path();
    if (!parent_path.empty()) {
        std::filesystem::create_directories(parent_path);
    }

    const char* error = map_->SaveNPY(path.string());
    if (error != nullptr) {
        throw std::runtime_error(std::string("Failed to save NPY file: ") + error);
    }
}

} // namespace user_common_207610130_215664087
