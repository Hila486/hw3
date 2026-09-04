#include <Simulator/DlLoader.h>
#include <Simulator/Registrar.h>

#include <iostream>
#include <mutex>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace simulator_207610130_215664087 {

namespace {
// Prevents multiple libraries from being loaded and registered at the same time.
std::mutex g_dl_load_mutex;
}

// Creates a loader for the given shared-library path.
DlLoader::DlLoader(std::filesystem::path library_path)
    : library_path_(std::move(library_path)) {}

// Automatically unloads the library when the loader is destroyed.
DlLoader::~DlLoader() {
    unload();
}

// Move constructor transfers ownership of the loaded library
// and its captured factories from another DlLoader.
DlLoader::DlLoader(DlLoader&& other) noexcept
    : library_path_(std::move(other.library_path_)),
      handle_(other.handle_),
      error_message_(std::move(other.error_message_)),
      algorithm_factory_(std::move(other.algorithm_factory_)),
      mc_factory_(std::move(other.mc_factory_)) {

    // The moved-from object must no longer own the library handle.
    other.handle_ = nullptr;
}

// Move assignment transfers ownership from another DlLoader.
DlLoader& DlLoader::operator=(DlLoader&& other) noexcept {
    if (this != &other) {
        // First release any library currently owned by this object.
        unload();
        library_path_ = std::move(other.library_path_);
        handle_ = other.handle_;
        error_message_ = std::move(other.error_message_);
        algorithm_factory_ = std::move(other.algorithm_factory_);
        mc_factory_ = std::move(other.mc_factory_);
        // Prevent the moved-from object from unloading the same handle.
        other.handle_ = nullptr;
    }
    return *this;
}

// Loads the shared library and captures the factory it registers.
bool DlLoader::load() {
    if (handle_ != nullptr) {
        return true;
    }

    // Only one dynamic-library load/registration process at a time.
    std::lock_guard<std::mutex> lock(g_dl_load_mutex);

    // Remove any factory left from a previous library load.
    Registrar::instance().clear();

#ifdef _WIN32
    // Load the DLL on Windows.
    handle_ = static_cast<void*>(LoadLibraryA(library_path_.string().c_str()));
    if (handle_ == nullptr) {
        const DWORD err_code = GetLastError();
        error_message_ = "LoadLibrary failed for " + library_path_.string() +
                         " (Win32 error " + std::to_string(err_code) + ")";
        return false;
    }
#else

    // Load the .so on Linux.
    // RTLD_NOW resolves symbols immediately.
    // RTLD_LOCAL keeps each plugin's symbols private so that loading several
    // plugins together (e.g. many algorithms in competition mode) cannot
    // interpose identically named global symbols across plugins. The simulator
    // exports its own symbols via -rdynamic, so registration still resolves.
    handle_ = dlopen(library_path_.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle_ == nullptr) {
        const char* err = dlerror();
        error_message_ = "dlopen failed for " + library_path_.string() + ": " +
                         (err != nullptr ? err : "unknown error");
        return false;
    }
#endif

    // Loading the library runs its registration macro,
    // which stores its factory in the Registrar.
    algorithm_factory_ = Registrar::instance().popAlgorithmFactory();
    mc_factory_ = Registrar::instance().popMissionControlFactory();

    // A valid plugin must register at least one supported factory.
    if (!algorithm_factory_ && !mc_factory_) {
        error_message_ = "No registered factory found in loaded library: " + library_path_.string();
        unload();
        return false;
    }

    return true;
}

// Releases the factories and unloads the shared library.
void DlLoader::unload() {
    // Factories may contain code from the loaded library,
    // so destroy them before unloading the library itself.
    algorithm_factory_.reset();
    mc_factory_.reset();
    if (handle_ != nullptr) {
#ifdef _WIN32
        FreeLibrary(static_cast<HMODULE>(handle_));
#else
        dlclose(handle_);
#endif
        handle_ = nullptr;
    }
}

} // namespace simulator_207610130_215664087
