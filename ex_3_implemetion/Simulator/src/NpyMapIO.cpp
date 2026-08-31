#include <Simulator/NpyMapIO.h>

#include <cstdint>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace simulator_207610130_215664087 {

namespace {

constexpr const char* kNpyMagic = "\x93NUMPY";

struct NpyHeader {
    std::string descr;
    bool fortran_order = false;
    std::vector<std::size_t> shape;
};

NpyHeader parseNpyHeader(const std::string& header_dict) {
    NpyHeader header;

    std::smatch match;
    std::regex descr_regex(R"('descr':\s*'([^']+)')");
    if (std::regex_search(header_dict, match, descr_regex)) {
        header.descr = match[1].str();
    }

    std::regex fortran_regex(R"('fortran_order':\s*(True|False))");
    if (std::regex_search(header_dict, match, fortran_regex)) {
        header.fortran_order = (match[1].str() == "True");
    }

    std::regex shape_regex(R"('shape':\s*\(([^)]*)\))");
    if (std::regex_search(header_dict, match, shape_regex)) {
        std::string shape_str = match[1].str();
        std::regex num_regex(R"(\d+)");
        auto begin = std::sregex_iterator(shape_str.begin(), shape_str.end(), num_regex);
        auto end = std::sregex_iterator();
        for (auto it = begin; it != end; ++it) {
            header.shape.push_back(std::stoull(it->str()));
        }
    }

    return header;
}

} // namespace

std::shared_ptr<NpyArray3D> loadNormalizedNpyMap(const std::filesystem::path& file_path) {
    std::ifstream file(file_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open NPY map file: " + file_path.string());
    }

    char magic[6];
    file.read(magic, 6);
    if (std::string(magic, 6) != kNpyMagic) {
        throw std::runtime_error("Invalid NPY magic bytes: " + file_path.string());
    }

    uint8_t major = 0, minor = 0;
    file.read(reinterpret_cast<char*>(&major), 1);
    file.read(reinterpret_cast<char*>(&minor), 1);

    uint32_t header_len = 0;
    if (major == 1) {
        uint16_t hlen16 = 0;
        file.read(reinterpret_cast<char*>(&hlen16), 2);
        header_len = hlen16;
    } else if (major == 2) {
        file.read(reinterpret_cast<char*>(&header_len), 4);
    } else {
        throw std::runtime_error("Unsupported NPY version in: " + file_path.string());
    }

    std::string header_str(header_len, '\0');
    file.read(&header_str[0], header_len);

    NpyHeader header = parseNpyHeader(header_str);
    if (header.shape.size() != 3) {
        throw std::runtime_error("NPY array must be 3D. Found dimensions: " + std::to_string(header.shape.size()));
    }

    auto result = std::make_shared<NpyArray3D>();
    result->shape = {header.shape[0], header.shape[1], header.shape[2]};
    const std::size_t total_elements = header.shape[0] * header.shape[1] * header.shape[2];
    result->data.resize(total_elements);

    if (header.descr == "|i1" || header.descr == "|b1" || header.descr == "b1" || header.descr == "i1") {
        std::vector<int8_t> raw(total_elements);
        file.read(reinterpret_cast<char*>(raw.data()), total_elements);
        for (std::size_t i = 0; i < total_elements; ++i) {
            result->data[i] = static_cast<int>(raw[i]);
        }
    } else if (header.descr == "|u1" || header.descr == "u1") {
        std::vector<uint8_t> raw(total_elements);
        file.read(reinterpret_cast<char*>(raw.data()), total_elements);
        for (std::size_t i = 0; i < total_elements; ++i) {
            result->data[i] = static_cast<int>(raw[i]);
        }
    } else if (header.descr == "<i4" || header.descr == "i4" || header.descr == "=i4") {
        std::vector<int32_t> raw(total_elements);
        file.read(reinterpret_cast<char*>(raw.data()), total_elements * sizeof(int32_t));
        for (std::size_t i = 0; i < total_elements; ++i) {
            result->data[i] = static_cast<int>(raw[i]);
        }
    } else if (header.descr == "<i8" || header.descr == "i8" || header.descr == "=i8") {
        std::vector<int64_t> raw(total_elements);
        file.read(reinterpret_cast<char*>(raw.data()), total_elements * sizeof(int64_t));
        for (std::size_t i = 0; i < total_elements; ++i) {
            result->data[i] = static_cast<int>(raw[i]);
        }
    } else {
        throw std::runtime_error("Unsupported NPY dtype: " + header.descr);
    }

    validateInputMapValues(*result, file_path);
    return result;
}

void validateInputMapValues(const NpyArray3D& array, const std::filesystem::path& file_path) {
    for (int val : array.data) {
        if (val < 0) {
            throw std::runtime_error("Invalid negative voxel value " + std::to_string(val) + " in map: " + file_path.string());
        }
    }
}

NpyMapShape npyMapShape(const NpyArray3D& array, const std::filesystem::path& /*file_path*/) {
    return NpyMapShape{array.shape[0], array.shape[1], array.shape[2]};
}

std::shared_ptr<NpyArray3D> makeFilledIntNpyArray(const NpyMapShape& shape, int fill_value) {
    auto array = std::make_shared<NpyArray3D>();
    array->shape = {shape.dim_x, shape.dim_y, shape.dim_z};
    array->data.assign(shape.dim_x * shape.dim_y * shape.dim_z, fill_value);
    return array;
}

void saveNpyMap(const std::filesystem::path& file_path, const NpyArray3D& map_array) {
    std::ofstream file(file_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open NPY file for writing: " + file_path.string());
    }

    file.write(kNpyMagic, 6);
    const uint8_t major = 1;
    const uint8_t minor = 0;
    file.write(reinterpret_cast<const char*>(&major), 1);
    file.write(reinterpret_cast<const char*>(&minor), 1);

    std::ostringstream ss;
    ss << "{'descr': '<i4', 'fortran_order': False, 'shape': ("
       << map_array.shape[0] << ", "
       << map_array.shape[1] << ", "
       << map_array.shape[2] << "), }";
    std::string header_dict = ss.str();

    // Pad header string to make (10 + header_len) divisible by 64
    const std::size_t total_header_prefix = 10;
    std::size_t rem = (total_header_prefix + header_dict.size() + 1) % 64;
    std::size_t padding = (rem == 0) ? 0 : (64 - rem);
    header_dict.append(padding, ' ');
    header_dict.push_back('\n');

    uint16_t hlen = static_cast<uint16_t>(header_dict.size());
    file.write(reinterpret_cast<const char*>(&hlen), 2);
    file.write(header_dict.data(), header_dict.size());

    std::vector<int32_t> int32_data(map_array.data.begin(), map_array.data.end());
    file.write(reinterpret_cast<const char*>(int32_data.data()), int32_data.size() * sizeof(int32_t));
}

} // namespace simulator_207610130_215664087
