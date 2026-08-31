#include <Simulator/ArgumentParser.h>
#include <Simulator/SimulationEngine.h>

#include <iostream>

using namespace simulator_207610130_215664087;

/**
 * @brief Main entry point for simulator_207610130_215664087 executable.
 */
int main(int argc, char* argv[]) {
    auto parsed_args = ArgumentParser::parse(argc, argv);
    if (!parsed_args) {
        std::cerr << ArgumentParser::getLastError() << std::endl;
        return 1;
    }

    SimulationEngine engine(*parsed_args);
    if (!engine.run()) {
        std::cerr << "Error: Simulation execution failed." << std::endl;
        return 1;
    }

    return 0;
}
