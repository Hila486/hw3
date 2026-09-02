#pragma once

#include <Simulator/ArgumentParser.h>

#include <filesystem>
#include <vector>

namespace simulator_207610130_215664087 {

/**
 * header file that declares the main class responsible for running the simulation
 */
class SimulationEngine {
public:
    /**
     * Constructs SimulationEngine with parsed CLI options.
     */
    explicit SimulationEngine(ParsedArgs args);

    /**
     *  Executes the simulation batch using multi-threaded task allocation.
     * @return True if batch completed successfully, false on critical setup failure.
     */
    bool run();

private:
    // Helper methods for running the simulation in different modes
    bool runComparative();
    bool runCompetitive();

    ParsedArgs args_;
};

} // namespace simulator_207610130_215664087
