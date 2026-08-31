#include <Simulator/ArgumentParser.h>

#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace simulator_207610130_215664087 {

namespace {

constexpr const char* kUsageText =
    "Usage (Comparative Mode):\n"
    "  ./simulator_<ids> -comparative simulation=<simulation_composition_yaml> mission_control_folder=<folder> algorithm=<algorithm_so> [num_threads=<num>] [-verbose]\n"
    "Usage (Competition Mode):\n"
    "  ./simulator_<ids> -competition simulation=<simulation_composition_yaml> mission_control=<mission_control_so> algorithms_folder=<folder> [num_threads=<num>] [-verbose]\n";

bool hasDesiredSoFiles(const std::filesystem::path& folder_path, const std::string& expected_prefix) {
    try {
        if (!std::filesystem::is_directory(folder_path)) {
            return false;
        }
        bool found_any_so = false;
        for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
            if (entry.is_regular_file() && entry.path().extension() == ".so") {
                found_any_so = true;
                const std::string filename = entry.path().filename().string();
                if (expected_prefix.empty() || filename.rfind(expected_prefix, 0) == 0) {
                    return true;
                }
            }
        }
        // Fallback: if no prefix-matching file found, return true if any .so exists
        return found_any_so;
    } catch (...) {
        return false;
    }
}

bool canOpenFile(const std::filesystem::path& file_path) {
    try {
        if (!std::filesystem::is_regular_file(file_path)) {
            return false;
        }
        std::ifstream file(file_path);
        return file.is_open();
    } catch (...) {
        return false;
    }
}

bool isStrictPositiveInteger(const std::string& str, std::size_t& result) {
    if (str.empty()) {
        return false;
    }
    for (char c : str) {
        if (!std::isdigit(static_cast<unsigned char>(c))) {
            return false;
        }
    }
    try {
        std::size_t pos = 0;
        int val = std::stoi(str, &pos);
        if (pos != str.length() || val <= 0) {
            return false;
        }
        result = static_cast<std::size_t>(val);
        return true;
    } catch (...) {
        return false;
    }
}

std::string formatErrorWithUsage(const std::string& error_message) {
    return error_message + "\n" + kUsageText;
}

} // namespace

std::optional<ParsedArgs> ArgumentParser::parse(int argc, char* argv[]) {
    last_error_.clear();

    if (argc < 2) {
        last_error_ = formatErrorWithUsage("Error: No command line arguments provided.");
        return std::nullopt;
    }

    std::optional<ExecutionMode> mode;
    std::unordered_map<std::string, std::string> kv_args;
    std::vector<std::string> unsupported_args;
    bool verbose = false;

    // Supported keys across both modes
    const std::set<std::string> known_keys = {
        "simulation",
        "mission_control_folder",
        "mission_control",
        "algorithm",
        "algorithms_folder",
        "num_threads"
    };

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-comparative") {
            if (mode) {
                last_error_ = formatErrorWithUsage("Error: Duplicate execution mode flag specified.");
                return std::nullopt;
            }
            mode = ExecutionMode::Comparative;
        } else if (arg == "-competition") {
            if (mode) {
                last_error_ = formatErrorWithUsage("Error: Duplicate execution mode flag specified.");
                return std::nullopt;
            }
            mode = ExecutionMode::Competition;
        } else if (arg == "-verbose") {
            verbose = true;
        } else {
            auto pos = arg.find('=');
            if (pos != std::string::npos && pos > 0 && pos < arg.length() - 1) {
                std::string key = arg.substr(0, pos);
                std::string value = arg.substr(pos + 1);
                if (known_keys.count(key)) {
                    kv_args[key] = value;
                } else {
                    unsupported_args.push_back(arg);
                }
            } else {
                unsupported_args.push_back(arg);
            }
        }
    }

    if (!mode) {
        last_error_ = formatErrorWithUsage("Error: Missing mandatory execution mode flag (-comparative or -competition).");
        return std::nullopt;
    }

    // Mode-specific whitelist validation
    const std::set<std::string> comparative_allowed = {
        "simulation", "mission_control_folder", "algorithm", "num_threads"
    };
    const std::set<std::string> competition_allowed = {
        "simulation", "mission_control", "algorithms_folder", "num_threads"
    };

    const auto& allowed_keys = (*mode == ExecutionMode::Comparative) ? comparative_allowed : competition_allowed;
    for (const auto& [key, value] : kv_args) {
        if (!allowed_keys.count(key)) {
            unsupported_args.push_back(key + "=" + value);
        }
    }

    if (!unsupported_args.empty()) {
        std::ostringstream error_stream;
        error_stream << "Error: Unsupported command line arguments provided: ";
        for (std::size_t i = 0; i < unsupported_args.size(); ++i) {
            if (i > 0) error_stream << ", ";
            error_stream << unsupported_args[i];
        }
        last_error_ = formatErrorWithUsage(error_stream.str());
        return std::nullopt;
    }

    ParsedArgs args;
    args.mode = *mode;
    args.verbose = verbose;

    // Check optional num_threads with strict positive integer parsing
    if (kv_args.count("num_threads")) {
        std::size_t threads = 0;
        if (!isStrictPositiveInteger(kv_args["num_threads"], threads)) {
            last_error_ = formatErrorWithUsage("Error: Invalid num_threads value: " + kv_args["num_threads"] + ". Must be a positive integer.");
            return std::nullopt;
        }
        args.num_threads = threads;
    }

    // Validate simulation config file
    if (!kv_args.count("simulation")) {
        last_error_ = formatErrorWithUsage("Error: Missing mandatory argument 'simulation=<file.yaml>'.");
        return std::nullopt;
    }
    args.simulation_file = kv_args["simulation"];
    if (!canOpenFile(args.simulation_file)) {
        last_error_ = formatErrorWithUsage("Error: Simulation file does not exist or cannot be opened: " + args.simulation_file.string());
        return std::nullopt;
    }

    if (args.mode == ExecutionMode::Comparative) {
        std::vector<std::string> missing_args;
        if (!kv_args.count("mission_control_folder")) {
            missing_args.push_back("mission_control_folder=<folder>");
        }
        if (!kv_args.count("algorithm")) {
            missing_args.push_back("algorithm=<algorithm_so>");
        }

        if (!missing_args.empty()) {
            std::ostringstream oss;
            oss << "Error: Missing mandatory argument(s) for comparative mode: ";
            for (std::size_t i = 0; i < missing_args.size(); ++i) {
                if (i > 0) oss << ", ";
                oss << missing_args[i];
            }
            last_error_ = formatErrorWithUsage(oss.str());
            return std::nullopt;
        }

        args.mission_control_folder = kv_args["mission_control_folder"];
        args.algorithm_file = kv_args["algorithm"];

        if (!canOpenFile(args.algorithm_file)) {
            last_error_ = formatErrorWithUsage("Error: Algorithm library file does not exist or cannot be opened: " + args.algorithm_file.string());
            return std::nullopt;
        }

        try {
            if (!std::filesystem::is_directory(args.mission_control_folder)) {
                last_error_ = formatErrorWithUsage("Error: Mission control folder does not exist or is not a directory: " + args.mission_control_folder.string());
                return std::nullopt;
            }
        } catch (const std::exception& e) {
            last_error_ = formatErrorWithUsage("Error: Mission control folder cannot be accessed: " + args.mission_control_folder.string() + " (" + e.what() + ")");
            return std::nullopt;
        }

        if (!hasDesiredSoFiles(args.mission_control_folder, "MissionControl")) {
            last_error_ = formatErrorWithUsage("Error: Mission control folder contains zero MissionControl .so files: " + args.mission_control_folder.string());
            return std::nullopt;
        }

    } else { // Competition mode
        std::vector<std::string> missing_args;
        if (!kv_args.count("mission_control")) {
            missing_args.push_back("mission_control=<mission_control_so>");
        }
        if (!kv_args.count("algorithms_folder")) {
            missing_args.push_back("algorithms_folder=<folder>");
        }

        if (!missing_args.empty()) {
            std::ostringstream oss;
            oss << "Error: Missing mandatory argument(s) for competition mode: ";
            for (std::size_t i = 0; i < missing_args.size(); ++i) {
                if (i > 0) oss << ", ";
                oss << missing_args[i];
            }
            last_error_ = formatErrorWithUsage(oss.str());
            return std::nullopt;
        }

        args.mission_control_file = kv_args["mission_control"];
        args.algorithms_folder = kv_args["algorithms_folder"];

        if (!canOpenFile(args.mission_control_file)) {
            last_error_ = formatErrorWithUsage("Error: Mission control library file does not exist or cannot be opened: " + args.mission_control_file.string());
            return std::nullopt;
        }

        try {
            if (!std::filesystem::is_directory(args.algorithms_folder)) {
                last_error_ = formatErrorWithUsage("Error: Algorithms folder does not exist or is not a directory: " + args.algorithms_folder.string());
                return std::nullopt;
            }
        } catch (const std::exception& e) {
            last_error_ = formatErrorWithUsage("Error: Algorithms folder cannot be accessed: " + args.algorithms_folder.string() + " (" + e.what() + ")");
            return std::nullopt;
        }

        if (!hasDesiredSoFiles(args.algorithms_folder, "Algorithm")) {
            last_error_ = formatErrorWithUsage("Error: Algorithms folder contains zero Algorithm .so files: " + args.algorithms_folder.string());
            return std::nullopt;
        }
    }

    return args;
}

} // namespace simulator_207610130_215664087
