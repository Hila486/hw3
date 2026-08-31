#include <Simulator/ArgumentParser.h>

#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace simulator_207610130_215664087 {

namespace {

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

} // namespace

std::optional<ParsedArgs> ArgumentParser::parse(int argc, char* argv[]) {
    last_error_.clear();
    std::ostringstream error_stream;

    if (argc < 2) {
        last_error_ = "Error: No command line arguments provided.\n"
                      "Usage (Comparative):\n"
                      "  ./simulator_<ids> -comparative simulation=<file.yaml> mission_control_folder=<dir> algorithm=<algo.so> [num_threads=<N>] [-verbose]\n"
                      "Usage (Competition):\n"
                      "  ./simulator_<ids> -competition simulation=<file.yaml> mission_control=<mc.so> algorithms_folder=<dir> [num_threads=<N>] [-verbose]\n";
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
                error_stream << "Error: Duplicate execution mode flag specified.\n";
                last_error_ = error_stream.str();
                return std::nullopt;
            }
            mode = ExecutionMode::Comparative;
        } else if (arg == "-competition") {
            if (mode) {
                error_stream << "Error: Duplicate execution mode flag specified.\n";
                last_error_ = error_stream.str();
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
        error_stream << "Error: Unsupported command line arguments provided: ";
        for (std::size_t i = 0; i < unsupported_args.size(); ++i) {
            if (i > 0) error_stream << ", ";
            error_stream << unsupported_args[i];
        }
        error_stream << "\n";
        last_error_ = error_stream.str();
        return std::nullopt;
    }

    if (!mode) {
        error_stream << "Error: Missing mandatory execution mode flag (-comparative or -competition).\n";
        last_error_ = error_stream.str();
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
                error_stream << "Error: num_threads must be a positive integer.\n";
                last_error_ = error_stream.str();
                return std::nullopt;
            }
            args.num_threads = static_cast<std::size_t>(threads);
        } catch (...) {
            error_stream << "Error: Invalid num_threads argument value: " << kv_args["num_threads"] << "\n";
            last_error_ = error_stream.str();
            return std::nullopt;
        }
    }

    // Validate simulation config file
    if (!kv_args.count("simulation")) {
        error_stream << "Error: Missing mandatory argument 'simulation=<file.yaml>'.\n";
        last_error_ = error_stream.str();
        return std::nullopt;
    }
    args.simulation_file = kv_args["simulation"];
    if (!std::filesystem::exists(args.simulation_file) || !std::filesystem::is_regular_file(args.simulation_file)) {
        error_stream << "Error: Simulation file does not exist or cannot be opened: " << args.simulation_file.string() << "\n";
        last_error_ = error_stream.str();
        return std::nullopt;
    }

    if (args.mode == ExecutionMode::Comparative) {
        if (!kv_args.count("mission_control_folder")) {
            error_stream << "Error: Missing mandatory argument 'mission_control_folder=<dir>' for comparative run.\n";
            last_error_ = error_stream.str();
            return std::nullopt;
        }
        if (!kv_args.count("algorithm")) {
            error_stream << "Error: Missing mandatory argument 'algorithm=<file.so>' for comparative run.\n";
            last_error_ = error_stream.str();
            return std::nullopt;
        }

        args.mission_control_folder = kv_args["mission_control_folder"];
        args.algorithm_file = kv_args["algorithm"];

        if (!std::filesystem::exists(args.algorithm_file) || !std::filesystem::is_regular_file(args.algorithm_file)) {
            error_stream << "Error: Algorithm library file does not exist or cannot be opened: " << args.algorithm_file.string() << "\n";
            last_error_ = error_stream.str();
            return std::nullopt;
        }

        if (!std::filesystem::exists(args.mission_control_folder) || !std::filesystem::is_directory(args.mission_control_folder)) {
            error_stream << "Error: Mission control folder does not exist or cannot be traversed: " << args.mission_control_folder.string() << "\n";
            last_error_ = error_stream.str();
            return std::nullopt;
        }

        if (!hasSoFiles(args.mission_control_folder)) {
            error_stream << "Error: Mission control folder contains zero .so files: " << args.mission_control_folder.string() << "\n";
            last_error_ = error_stream.str();
            return std::nullopt;
        }

    } else { // Competition mode
        if (!kv_args.count("mission_control")) {
            error_stream << "Error: Missing mandatory argument 'mission_control=<file.so>' for competition run.\n";
            last_error_ = error_stream.str();
            return std::nullopt;
        }
        if (!kv_args.count("algorithms_folder")) {
            error_stream << "Error: Missing mandatory argument 'algorithms_folder=<dir>' for competition run.\n";
            last_error_ = error_stream.str();
            return std::nullopt;
        }

        args.mission_control_file = kv_args["mission_control"];
        args.algorithms_folder = kv_args["algorithms_folder"];

        if (!std::filesystem::exists(args.mission_control_file) || !std::filesystem::is_regular_file(args.mission_control_file)) {
            error_stream << "Error: Mission control library file does not exist or cannot be opened: " << args.mission_control_file.string() << "\n";
            last_error_ = error_stream.str();
            return std::nullopt;
        }

        if (!std::filesystem::exists(args.algorithms_folder) || !std::filesystem::is_directory(args.algorithms_folder)) {
            error_stream << "Error: Algorithms folder does not exist or cannot be traversed: " << args.algorithms_folder.string() << "\n";
            last_error_ = error_stream.str();
            return std::nullopt;
        }

        if (!hasSoFiles(args.algorithms_folder)) {
            error_stream << "Error: Algorithms folder contains zero .so files: " << args.algorithms_folder.string() << "\n";
            last_error_ = error_stream.str();
            return std::nullopt;
        }
    }

    return args;
}

} // namespace simulator_207610130_215664087
