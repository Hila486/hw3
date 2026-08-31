#include <Simulator/Registrar.h>

#include <Common/MappingAlgorithmRegistration.h>
#include <Common/MissionControlRegistration.h>

namespace simulator_207610130_215664087 {

Registrar& Registrar::instance() {
    static Registrar instance;
    return instance;
}

void Registrar::registerAlgorithm(common::MappingAlgorithmFactory factory) {
    std::lock_guard<std::mutex> lock(mutex_);
    algorithm_factory_ = std::move(factory);
}

void Registrar::registerMissionControl(common::MissionControlFactory factory) {
    std::lock_guard<std::mutex> lock(mutex_);
    mc_factory_ = std::move(factory);
}

void Registrar::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    algorithm_factory_.reset();
    mc_factory_.reset();
}

std::optional<common::MappingAlgorithmFactory> Registrar::popAlgorithmFactory() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::move(algorithm_factory_);
    algorithm_factory_.reset();
    return result;
}

std::optional<common::MissionControlFactory> Registrar::popMissionControlFactory() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::move(mc_factory_);
    mc_factory_.reset();
    return result;
}

} // namespace simulator_207610130_215664087

// Implement the registration constructors defined in common/ headers.
// These constructors execute when a loaded .so calls REGISTER_MAPPING_ALGORITHM or REGISTER_MISSION_CONTROL.

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    simulator_207610130_215664087::Registrar::instance().registerAlgorithm(std::move(factory));
}

MissionControlRegistration::MissionControlRegistration(MissionControlFactory factory) {
    simulator_207610130_215664087::Registrar::instance().registerMissionControl(std::move(factory));
}

} // namespace common
