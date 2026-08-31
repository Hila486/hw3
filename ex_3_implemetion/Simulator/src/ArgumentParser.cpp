#include <Simulator/ArgumentParser.h>

#include <filesystem>
#include <iostream>
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

bool hasSoFiles(const std::filesystem::path& folder_path) {
    if (!std::filesystem::is_directory(folder_path)) {
        return false;
    }
    for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".so") {
            return true;
        }
    }
    return false;
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
                kv_args[key] = value;
            } else {
                unsupported_args.push_back(arg);
            }
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

    if (!mode) {
        last_error_ = formatErrorWithUsage("Error: Missing mandatory execution mode flag (-comparative or -competition).");
        return std::nullopt;
    }

    ParsedArgs args;
    args.mode = *mode;
    args.verbose = verbose;

    // Check optional num_threads
    if (kv_args.count("num_threads")) {
        try {
            int threads = std::stoi(kv_args["num_threads"]);
            if (threads <= 0) {
                last_error_ = formatErrorWithUsage("Error: num_threads must be a positive integer.");
                return std::nullopt;
            }
            args.num_threads = static_cast<std::size_t>(threads);
        } catch (...) {
            last_error_ = formatErrorWithUsage("Error: Invalid num_threads argument value: " + kv_args["num_threads"]);
            return std::nullopt;
        }
    }

    // Validate simulation config file
    if (!kv_args.count("simulation")) {
        last_error_ = formatErrorWithUsage("Error: Missing mandatory argument 'simulation=<file.yaml>'.");
        return std::nullopt;
    }
    args.simulation_file = kv_args["simulation"];
    if (!std::filesystem::exists(args.simulation_file) || !std::filesystem::is_regular_file(args.simulation_file)) {
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

        if (!std::filesystem::exists(args.algorithm_file) || !std::filesystem::is_regular_file(args.algorithm_file)) {
            last_error_ = formatErrorWithUsage("Error: Algorithm library file does not exist or cannot be opened: " + args.algorithm_file.string());
            return std::nullopt;
        }

        if (!std::filesystem::exists(args.mission_control_folder) || !std::filesystem::is_directory(args.mission_control_folder)) {
            last_error_ = formatErrorWithUsage("Error: Mission control folder does not exist or cannot be traversed: " + args.mission_control_folder.string());
            return std::nullopt;
        }

        if (!hasSoFiles(args.mission_control_folder)) {
            last_error_ = formatErrorWithUsage("Error: Mission control folder contains zero .so files: " + args.mission_control_folder.string());
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

        if (!std::filesystem::exists(args.mission_control_file) || !std::filesystem::is_regular_file(args.mission_control_file)) {
            last_error_ = formatErrorWithUsage("Error: Mission control library file does not exist or cannot be opened: " + args.mission_control_file.string());
            return std::nullopt;
        }

        if (!std::filesystem::exists(args.algorithms_folder) || !std::filesystem::is_directory(args.algorithms_folder)) {
            last_error_ = formatErrorWithUsage("Error: Algorithms folder does not exist or cannot be traversed: " + args.algorithms_folder.string());
            return std::nullopt;
        }

        if (!hasSoFiles(args.algorithms_folder)) {
            last_error_ = formatErrorWithUsage("Error: Algorithms folder contains zero .so files: " + args.algorithms_folder.string());
            return std::nullopt;
        }
    }

    return args;
}

} // namespace simulator_207610130_215664087
