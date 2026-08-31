/**
 * @file DlLoader.cpp
 * @brief Dynamic shared library loader with thread-safe registration sequence.
 *
 * The critical invariant: the sequence (Registrar::clear → dlopen → Registrar::pop)
 * must be atomic with respect to other threads doing the same sequence. Without the
 * global mutex, two concurrent load() calls can interleave and steal each other's
 * registered factories from the Registrar singleton.
 */

#include <Simulator/DlLoader.h>
#include <Simulator/Registrar.h>

#include <utility>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace simulator_207610130_215664087 {

// Static global mutex definition — protects the clear→dlopen→pop sequence.
std::mutex DlLoader::load_mutex_;

DlLoader::DlLoader(std::filesystem::path library_path)
    : library_path_(std::move(library_path)) {}

DlLoader::~DlLoader() {
    unload();
}

DlLoader::DlLoader(DlLoader&& other) noexcept
    : library_path_(std::move(other.library_path_)),
      handle_(other.handle_),
      error_message_(std::move(other.error_message_)),
      algorithm_factory_(std::move(other.algorithm_factory_)),
      mc_factory_(std::move(other.mc_factory_)) {
    other.handle_ = nullptr;
}

DlLoader& DlLoader::operator=(DlLoader&& other) noexcept {
    if (this != &other) {
        unload();
        library_path_      = std::move(other.library_path_);
        handle_            = other.handle_;
        error_message_     = std::move(other.error_message_);
        algorithm_factory_ = std::move(other.algorithm_factory_);
        mc_factory_        = std::move(other.mc_factory_);
        other.handle_      = nullptr;
    }
    return *this;
}

/**
 * @brief Thread-safe .so loading.
 *
 * The global mutex serialises the three-step sequence so that no two threads
 * can interleave their registration steps through the Registrar singleton.
 */
bool DlLoader::load() {
    if (handle_ != nullptr) {
        return true;
    }

    // Lock globally so no other thread can run its own clear→dlopen→pop concurrently.
    std::lock_guard<std::mutex> global_lock(load_mutex_);

    Registrar::instance().clear();

#ifdef _WIN32
    handle_ = static_cast<void*>(LoadLibraryA(library_path_.string().c_str()));
    if (handle_ == nullptr) {
        const DWORD err_code = GetLastError();
        error_message_ = "LoadLibrary failed for " + library_path_.string() +
                         " (Win32 error " + std::to_string(err_code) + ")";
        return false;
    }
#else
    handle_ = dlopen(library_path_.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (handle_ == nullptr) {
        const char* err = dlerror();
        error_message_ = "dlopen failed for " + library_path_.string() + ": " +
                         (err != nullptr ? err : "unknown error");
        return false;
    }
#endif

    algorithm_factory_ = Registrar::instance().popAlgorithmFactory();
    mc_factory_        = Registrar::instance().popMissionControlFactory();

    if (!algorithm_factory_ && !mc_factory_) {
        error_message_ = "No registered factory found in: " + library_path_.string();
        unload();
        return false;
    }

    return true;
}

void DlLoader::unload() {
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
