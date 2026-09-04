#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace simulator_207610130_215664087 {

/**
 * Simulation run mode (Comparative vs Competitive).
 */
enum class ExecutionMode {
    Comparative,  //comparative: runs multiple MissionControls against single algorithm
    Competition   //ompetition: runs multiple Algorithms against single MissionControl
};

/**
 *  Holds validated command-line arguments.
 */
struct ParsedArgs {
    ExecutionMode mode = ExecutionMode::Comparative;
    std::filesystem::path simulation_file;        // YAML composition file path
    std::filesystem::path mission_control_folder; // Folder containing MissionControl .so files (Comparative)
    std::filesystem::path mission_control_file;   // Single MissionControl .so file path (Competition)
    std::filesystem::path algorithm_file;        // Single Algorithm .so file path (Comparative)
    std::filesystem::path algorithms_folder;     // Folder containing Algorithm .so files (Competition)
    std::size_t num_threads = 1;                  // Number of worker threads (default 1)
    bool verbose = false;                         // Verbose logging flag
};

/**
 *  Parses and validates command-line arguments for the simulator.
 */
class ArgumentParser {
public:
    /**
     * @brief Parses command-line arguments array.
     * @param argc Argument count.
     * @param argv Argument string array.
     * @return std::optional<ParsedArgs> containing parsed parameters, or std::nullopt on error.
     */
    static std::optional<ParsedArgs> parse(int argc, char* argv[]);

    /**
     * @brief Returns error description if parsing failed.
     */
    static std::string getLastError() { return last_error_; }

private:
    static inline std::string last_error_;
};

} // namespace simulator_207610130_215664087
