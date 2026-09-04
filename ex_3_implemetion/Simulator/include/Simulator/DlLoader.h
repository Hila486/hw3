#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>
#include <memory>
#include <string>

namespace simulator_207610130_215664087 {

/**
 * Manages runtime dynamic loading of compiled shared libraries via dlopen/LoadLibrary,
 * captures auto-registered factories, and handles clean library unloading (dlclose).
 */
class DlLoader {
public:
    // Creates a loader for the given shared library path.
    explicit DlLoader(std::filesystem::path library_path);

    /**
     *  Destructor automatically unloads the library handle via dlclose.
     */
    ~DlLoader();

    // Copying is disabled because two objects must not manage
    // and unload the same library handle.
    DlLoader(const DlLoader&) = delete;
    DlLoader& operator=(const DlLoader&) = delete;

    // Moving is allowed so ownership of the loaded library can be transferred.
    DlLoader(DlLoader&& other) noexcept;
    DlLoader& operator=(DlLoader&& other) noexcept;

    // Loads the shared library into memory.
    // Returns true on success and false on failure.
    bool load();

    // Unloads the currently loaded shared library.
    void unload();

    // Returns the error message from a failed load operation.
    [[nodiscard]] std::string error() const { return error_message_; }

    // Returns the mapping algorithm factory registered by the library,
    // if the loaded library contains one.
    [[nodiscard]] std::optional<common::MappingAlgorithmFactory> getAlgorithmFactory() const {
        return algorithm_factory_;
    }

 
    // Returns the mission control factory registered by the library,
    // if the loaded library contains one.
    [[nodiscard]] std::optional<common::MissionControlFactory> getMissionControlFactory() const {
        return mc_factory_;
    }

    // Returns only the filename of the shared library.
    [[nodiscard]] std::string filename() const { return library_path_.filename().string(); }

private:
    // Path of the .so file being managed.
    std::filesystem::path library_path_;
    // Handle returned by the operating system when the library is loaded.
    void* handle_ = nullptr;
    // Stores information about the latest loading error.                                         
    std::string error_message_;
    // Factory registered if the loaded library contains a mapping algorithm.                                 
    std::optional<common::MappingAlgorithmFactory> algorithm_factory_;
    // Factory registered if the loaded library contains mission control.   
    std::optional<common::MissionControlFactory> mc_factory_;            
};
} // namespace simulator_207610130_215664087
