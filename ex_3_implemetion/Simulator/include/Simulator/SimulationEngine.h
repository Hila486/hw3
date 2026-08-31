#pragma once

#include <Simulator/ArgumentParser.h>

#include <filesystem>
#include <vector>

namespace simulator_207610130_215664087 {

/**
 * @class SimulationEngine
 * @brief Multi-threaded simulation orchestrator executing Comparative and Competitive runs.
 */
class SimulationEngine {
public:
    /**
     * @brief Constructs SimulationEngine with parsed CLI options.
     */
    explicit SimulationEngine(ParsedArgs args);

    /**
     * @brief Executes the simulation batch using multi-threaded task allocation.
     * @return True if batch completed successfully, false on critical setup failure.
     */
    bool run();

private:
    bool runComparative();
    bool runCompetitive();

    ParsedArgs args_;
};

} // namespace simulator_207610130_215664087
