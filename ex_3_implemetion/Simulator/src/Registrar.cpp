#include <Simulator/Registrar.h>

#include <Common/MappingAlgorithmRegistration.h>
#include <Common/MissionControlRegistration.h>

namespace simulator_207610130_215664087 {
// Returns the single global Registrar instance.
Registrar& Registrar::instance() {
    static Registrar instance;
    return instance;
}

// Stores the mapping algorithm factory registered by a loaded library.
// The mutex makes the operation thread-safe.
void Registrar::registerAlgorithm(common::MappingAlgorithmFactory factory) {
    std::lock_guard<std::mutex> lock(mutex_);
    algorithm_factory_ = std::move(factory);
}


// Stores the mission control factory registered by a loaded library.
// The mutex makes the operation thread-safe.
void Registrar::registerMissionControl(common::MissionControlFactory factory) {
    std::lock_guard<std::mutex> lock(mutex_);
    mc_factory_ = std::move(factory);
}

// Removes any factories currently stored in the registrar.
void Registrar::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    algorithm_factory_.reset();
    mc_factory_.reset();
}

// Returns the registered algorithm factory and removes it
// from the registrar.
std::optional<common::MappingAlgorithmFactory> Registrar::popAlgorithmFactory() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::move(algorithm_factory_);
    algorithm_factory_.reset();
    return result;
}


// Returns the registered mission control factory and removes it
// from the registrar.
std::optional<common::MissionControlFactory> Registrar::popMissionControlFactory() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::move(mc_factory_);
    mc_factory_.reset();
    return result;
}

} // namespace simulator_207610130_215664087

// These constructors connect the registration system in Common
// to the Simulator's Registrar.
//
// When a loaded .so uses REGISTER_MAPPING_ALGORITHM or
// REGISTER_MISSION_CONTROL, one of these constructors runs and
// stores the factory in Registrar.
namespace common {

// Registers a mapping algorithm factory with the simulator.
MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    simulator_207610130_215664087::Registrar::instance().registerAlgorithm(std::move(factory));
}
// Registers a mission control factory with the simulator.
MissionControlRegistration::MissionControlRegistration(MissionControlFactory factory) {
    simulator_207610130_215664087::Registrar::instance().registerMissionControl(std::move(factory));
}

} // namespace common
