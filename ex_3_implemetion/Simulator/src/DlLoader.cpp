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
std::mutex g_dl_load_mutex;
}

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
        library_path_ = std::move(other.library_path_);
        handle_ = other.handle_;
        error_message_ = std::move(other.error_message_);
        algorithm_factory_ = std::move(other.algorithm_factory_);
        mc_factory_ = std::move(other.mc_factory_);
        other.handle_ = nullptr;
    }
    return *this;
}

bool DlLoader::load() {
    if (handle_ != nullptr) {
        return true;
    }

    std::lock_guard<std::mutex> lock(g_dl_load_mutex);

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
    mc_factory_ = Registrar::instance().popMissionControlFactory();

    if (!algorithm_factory_ && !mc_factory_) {
        error_message_ = "No registered factory found in loaded library: " + library_path_.string();
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
