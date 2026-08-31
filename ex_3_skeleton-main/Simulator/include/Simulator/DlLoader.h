#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>
#include <memory>
#include <string>

namespace simulator_207610130_215664087 {

/**
 * @class DlLoader
 * @brief Dynamic shared library (.so) loader wrapper.
 * 
 * Manages runtime dynamic loading of compiled shared libraries via dlopen/LoadLibrary,
 * captures auto-registered factories, and handles clean library unloading (dlclose).
 */
class DlLoader {
public:
    /**
     * @brief Constructs a DlLoader instance for a given .so library file path.
     * @param library_path Path to the .so shared library file.
     */
    explicit DlLoader(std::filesystem::path library_path);

    /**
     * @brief Destructor automatically unloads the library handle via dlclose.
     */
    ~DlLoader();

    // Disable copy semantics to prevent duplicate dlclose calls
    DlLoader(const DlLoader&) = delete;
    DlLoader& operator=(const DlLoader&) = delete;

    // Enable move semantics
    DlLoader(DlLoader&& other) noexcept;
    DlLoader& operator=(DlLoader&& other) noexcept;

    /**
     * @brief Loads the shared library into memory.
     * @return True if loaded successfully, false on error.
     */
    bool load();

    /**
     * @brief Unloads the shared library via dlclose.
     */
    void unload();

    /**
     * @brief Returns error string if load failed.
     */
    [[nodiscard]] std::string error() const { return error_message_; }

    /**
     * @brief Retrieves captured MappingAlgorithmFactory if this .so was an Algorithm library.
     */
    [[nodiscard]] std::optional<common::MappingAlgorithmFactory> getAlgorithmFactory() const {
        return algorithm_factory_;
    }

    /**
     * @brief Retrieves captured MissionControlFactory if this .so was a MissionControl library.
     */
    [[nodiscard]] std::optional<common::MissionControlFactory> getMissionControlFactory() const {
        return mc_factory_;
    }

    /**
     * @brief Returns the filename of the loaded .so library.
     */
    [[nodiscard]] std::string filename() const { return library_path_.filename().string(); }

private:
    std::filesystem::path library_path_;                             ///< Filepath to .so library
    void* handle_ = nullptr;                                         ///< POSIX dlopen handle / HINSTANCE
    std::string error_message_;                                      ///< Load error message details
    std::optional<common::MappingAlgorithmFactory> algorithm_factory_;   ///< Registered algorithm factory
    std::optional<common::MissionControlFactory> mc_factory_;            ///< Registered mission control factory
};

} // namespace simulator_207610130_215664087
