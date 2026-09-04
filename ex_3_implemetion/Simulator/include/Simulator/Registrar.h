#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <mutex>
#include <optional>

namespace simulator_207610130_215664087 {

/**
 * @class Registrar
 * @brief Thread-safe registry for capturing factories registered by loaded .so shared libraries.
 * 
 * When a dynamic library (.so) is loaded via dlopen, its static constructors execute the
 * REGISTER_MAPPING_ALGORITHM or REGISTER_MISSION_CONTROL macros, invoking the registration
 * constructors defined in the Simulator. This registrar captures and stores those factories.
 */
class Registrar {
public:
    /**
     * @brief Singleton instance accessor.
     */
    static Registrar& instance();

    /**
     * @brief Sets the active registered mapping algorithm factory.
     */
    void registerAlgorithm(common::MappingAlgorithmFactory factory);

    /**
     * @brief Sets the active registered mission control factory.
     */
    void registerMissionControl(common::MissionControlFactory factory);

    /**
     * @brief Clears currently registered factories (call before dlopen of a new .so).
     */
    void clear();

    /**
     * @brief Retrieves and consumes the registered algorithm factory.
     */
    [[nodiscard]] std::optional<common::MappingAlgorithmFactory> popAlgorithmFactory();

    /**
     * @brief Retrieves and consumes the registered mission control factory.
     */
    [[nodiscard]] std::optional<common::MissionControlFactory> popMissionControlFactory();

private:
    Registrar() = default;

    std::mutex mutex_;                                        ///< Mutex for thread safety
    std::optional<common::MappingAlgorithmFactory> algorithm_factory_;   ///< Captured algorithm factory
    std::optional<common::MissionControlFactory> mc_factory_;            ///< Captured mission control factory
};

} // namespace simulator_207610130_215664087
