/**
 * @file SimulationEngine.cpp
 * @brief Multi-threaded simulation orchestrator for Comparative and Competitive runs.
 *
 * Key design decisions:
 * - All .so files are pre-loaded ONCE on the main thread before worker threads start.
 *   This prevents race conditions in the Registrar singleton and avoids repeated
 *   dlopen/dlclose per run (as prohibited by the assignment spec).
 * - MapConfig is initialized as {boundaries, offset, resolution} (correct field order).
 * - output_mapping_resolution_factor is applied to scale the output map resolution.
 * - Per-SO individual YAML result files are exported after each SO's runs complete.
 * - All errors are collected and written to an error_log.txt file.
 */

#include <Simulator/SimulationEngine.h>
#include <Simulator/DlLoader.h>
#include <Simulator/ResultExporter.h>

#include <UserCommon/ConfigParser.h>
#include <UserCommon/Map3DImpl.h>
#include <UserCommon/MapsComparison.h>
#include <UserCommon/MockGPS.h>
#include <UserCommon/MockLidar.h>
#include <UserCommon/MockMovement.h>
#include <UserCommon/NpyMapIO.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace simulator_207610130_215664087 {

namespace {

using namespace user_common_207610130_215664087;

/// Discovers all .so files in a folder, sorted by path.
std::vector<std::filesystem::path> discoverSoFiles(const std::filesystem::path& folder) {
    std::vector<std::filesystem::path> files;
    if (!std::filesystem::is_directory(folder)) {
        return files;
    }
    for (const auto& entry : std::filesystem::directory_iterator(folder)) {
        if (entry.is_regular_file() && entry.path().extension() == ".so") {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

/// A single (simulation × mission × drone × lidar) run specification.
struct SingleRunSpec {
    SimulationConfigData simulation_config;
    common::types::MissionConfigData mission_config;
    common::types::DroneConfigData drone_config;
    common::types::LidarConfigData lidar_config;
    std::size_t run_index = 0;
};

/// Converts MissionRunStatus to string.
std::string statusToString(common::types::MissionRunStatus status) {
    switch (status) {
        case common::types::MissionRunStatus::Completed: return "Completed";
        case common::types::MissionRunStatus::MaxSteps:  return "MaxSteps";
        case common::types::MissionRunStatus::Error:     return "Error";
    }
    return "Unknown";
}

/// Executes one single simulation run using pre-loaded factories.
SingleRunResult executeSingleRun(
    const common::MappingAlgorithmFactory& algo_factory,
    const common::MissionControlFactory& mc_factory,
    const SingleRunSpec& spec,
    const std::filesystem::path& run_output_file,
    bool verbose) {

    SingleRunResult result;
    result.run_index = spec.run_index;
    result.output_map_file = run_output_file;

    try {
        // --- Load ground truth (hidden) map from .npy file ---
        auto hidden_map_data = loadNormalizedNpyMap(spec.simulation_config.map_filename);

        // MapConfig field order: {MappingBounds boundaries, Position3D offset, PhysicalLength resolution}
        common::types::MapConfig hidden_map_config{
            spec.mission_config.mission_bounds,
            spec.simulation_config.map_axes_offset,
            spec.simulation_config.map_resolution
        };
        Map3DImpl hidden_map(hidden_map_data, hidden_map_config);

        // --- Create output map, applying output_mapping_resolution_factor ---
        NpyMapShape hidden_shape = npyMapShape(*hidden_map_data, spec.simulation_config.map_filename);

        double resolution_factor = spec.mission_config.output_mapping_resolution_factor;
        if (resolution_factor <= 0.0) {
            resolution_factor = 1.0;
        }
        const common::PhysicalLength output_resolution =
            spec.simulation_config.map_resolution * resolution_factor;

        NpyMapShape output_shape = hidden_shape;
        if (std::abs(resolution_factor - 1.0) > 1.0e-9) {
            output_shape.dim_x = static_cast<std::size_t>(
                std::ceil(static_cast<double>(hidden_shape.dim_x) / resolution_factor));
            output_shape.dim_y = static_cast<std::size_t>(
                std::ceil(static_cast<double>(hidden_shape.dim_y) / resolution_factor));
            output_shape.dim_z = static_cast<std::size_t>(
                std::ceil(static_cast<double>(hidden_shape.dim_z) / resolution_factor));
        }

        auto output_map_data = makeFilledIntNpyArray(
            output_shape,
            static_cast<int>(common::types::VoxelOccupancy::Unmapped));

        common::types::MapConfig output_map_config{
            spec.mission_config.mission_bounds,
            spec.simulation_config.map_axes_offset,
            output_resolution
        };
        Map3DImpl output_map(output_map_data, output_map_config);

        // --- Hardware mocks ---
        MockGPS gps(spec.simulation_config.initial_drone_position,
                    Orientation{spec.simulation_config.initial_angle,
                                0.0 * altitude_angle[deg]},
                    spec.mission_config.gps_resolution);
        MockMovement movement(gps, &hidden_map);
        MockLidar lidar(spec.lidar_config, hidden_map, gps);

        // --- Instantiate Algorithm and MissionControl from pre-loaded factories ---
        common::MappingAlgorithmDependencies algo_deps{
            spec.mission_config,
            spec.lidar_config,
            spec.drone_config,
            output_map
        };
        auto mapping_algorithm = algo_factory(std::move(algo_deps));

        common::MissionControlDependencies mc_deps{
            spec.mission_config,
            spec.drone_config,
            lidar,
            gps,
            movement,
            output_map,
            *mapping_algorithm,
            run_output_file,
            verbose
        };
        auto mission_control = mc_factory(std::move(mc_deps));

        // --- Run the mission ---
        common::types::MissionRunResult run_res = mission_control->runMission();
        result.steps = run_res.steps;
        result.status = statusToString(run_res.status);

        if (run_res.status == common::types::MissionRunStatus::Error) {
            result.score = -1.0;
            if (!run_res.errors.empty()) {
                result.error_message = run_res.errors[0].message;
            } else {
                result.error_message = "Mission control reported an error.";
            }
            mission_control.reset();
            mapping_algorithm.reset();
            return result;
        }

        // --- Score the output map vs the hidden map ---
        std::vector<common::IMap3D*> targets{&output_map};
        std::vector<double> scores = MapsComparison::compare(hidden_map, targets);
        result.score = scores.empty() ? 0.0 : scores[0];

        // Explicitly destroy instances BEFORE returning (and before DlLoader could unload)
        mission_control.reset();
        mapping_algorithm.reset();

    } catch (const std::exception& ex) {
        result.error_message = std::string("Exception: ") + ex.what();
        result.status = "Error";
        result.score = -1.0;
    }

    return result;
}

/// Pre-loads a .so file and appends any error to error_log. Returns nullptr on failure.
std::unique_ptr<DlLoader> preloadLibrary(
    const std::filesystem::path& so_path,
    std::vector<std::string>& error_log) {

    auto loader = std::make_unique<DlLoader>(so_path);
    if (!loader->load()) {
        error_log.push_back("Failed to load " + so_path.filename().string() +
                            ": " + loader->error());
        return nullptr;
    }
    return loader;
}

} // namespace

SimulationEngine::SimulationEngine(ParsedArgs args) : args_(std::move(args)) {}

bool SimulationEngine::run() {
    if (args_.mode == ExecutionMode::Comparative) {
        runComparative();
    } else {
        runCompetitive();
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Comparative mode
// ─────────────────────────────────────────────────────────────────────────────
void SimulationEngine::runComparative() {
    auto mc_files = discoverSoFiles(args_.mission_control_folder);
    if (mc_files.empty()) {
        std::cerr << "Error: No .so files found in: "
                  << args_.mission_control_folder.string() << "\n";
        return;
    }

    SimulationCompositionData composition;
    try {
        composition = ConfigParser::parseSimulationComposition(args_.simulation_file);
    } catch (const std::exception& ex) {
        std::cerr << "Error parsing composition file: " << ex.what() << "\n";
        return;
    }

    // Build Cartesian product of (simulation × mission × drone × lidar) specs
    std::vector<SingleRunSpec> run_specs;
    std::size_t run_idx = 0;
    for (const auto& group : composition.simulation_mission_groups) {
        const auto& sim_config = std::get<0>(group);
        const auto& missions   = std::get<1>(group);
        for (const auto& mission_config : missions) {
            for (const auto& drone_config : composition.drone_configs) {
                for (const auto& lidar_config : composition.lidar_configs) {
                    run_specs.push_back(SingleRunSpec{
                        sim_config, mission_config, drone_config, lidar_config, ++run_idx});
                }
            }
        }
    }

    // ── Pre-load all .so files on the main thread ──────────────────────────
    std::vector<std::string> error_log;
    std::vector<std::string> error_managers;

    // Pre-load Algorithm .so
    auto algo_loader = preloadLibrary(args_.algorithm_file, error_log);
    if (!algo_loader) {
        std::cerr << "Error: Failed to load algorithm .so.\n";
        return;
    }
    auto algo_factory_opt = algo_loader->getAlgorithmFactory();
    if (!algo_factory_opt) {
        std::cerr << "Error: Algorithm .so did not register a factory.\n";
        return;
    }
    const common::MappingAlgorithmFactory& algo_factory = *algo_factory_opt;

    // Pre-load all MissionControl .so files
    struct PreloadedMC {
        std::string so_name;
        common::MissionControlFactory factory;
        std::unique_ptr<DlLoader> loader;   // Keeps the .so mapped in memory
    };
    std::vector<PreloadedMC> preloaded_mcs;
    for (const auto& mc_path : mc_files) {
        auto mc_loader = preloadLibrary(mc_path, error_log);
        if (!mc_loader) {
            error_managers.push_back(mc_path.filename().string());
            continue;
        }
        auto mc_factory_opt = mc_loader->getMissionControlFactory();
        if (!mc_factory_opt) {
            error_managers.push_back(mc_path.filename().string());
            error_log.push_back(mc_path.filename().string() +
                                " did not register a MissionControl factory.");
            continue;
        }
        preloaded_mcs.push_back(PreloadedMC{
            mc_path.filename().string(), *mc_factory_opt, std::move(mc_loader)});
    }

    // Prepare output directory
    const std::filesystem::path output_dir =
        args_.mission_control_folder /
        ("comparative_results_" + ResultExporter::getFolderTimestamp());
    std::filesystem::create_directories(output_dir);

    // ── Worker lambda — each thread picks the next MissionControl SO ───────
    std::vector<ComparativeManagerResult> manager_results;
    std::mutex results_mutex;
    std::atomic<std::size_t> mc_idx{0};

    auto worker = [&]() {
        while (true) {
            const std::size_t idx = mc_idx.fetch_add(1);
            if (idx >= preloaded_mcs.size()) break;

            const auto& mc = preloaded_mcs[idx];
            ComparativeManagerResult mgr_result;
            mgr_result.manager_so_name = mc.so_name;
            bool mc_failed = false;

            for (const auto& spec : run_specs) {
                std::ostringstream oss;
                oss << "output_map_" << mc.so_name << "_run_" << spec.run_index << ".npy";
                const std::filesystem::path run_file = output_dir / oss.str();

                SingleRunResult run_res = executeSingleRun(
                    algo_factory, mc.factory, spec, run_file, args_.verbose);
                mgr_result.individual_runs.push_back(run_res);

                if (run_res.score < 0.0) {
                    mc_failed = true;
                    std::lock_guard<std::mutex> lock(results_mutex);
                    error_log.push_back("[" + mc.so_name + " run " +
                                        std::to_string(spec.run_index) + "] " +
                                        run_res.error_message);
                    break;
                }
                mgr_result.total_score += run_res.score;
                mgr_result.total_steps += run_res.steps;
            }

            std::lock_guard<std::mutex> lock(results_mutex);
            if (mc_failed) {
                error_managers.push_back(mc.so_name);
            } else {
                manager_results.push_back(std::move(mgr_result));
            }
        }
    };

    // ── Launch threads ─────────────────────────────────────────────────────
    if (args_.num_threads <= 1) {
        worker();
    } else {
        const std::size_t actual = std::min(args_.num_threads, preloaded_mcs.size());
        std::vector<std::thread> threads;
        threads.reserve(actual);
        for (std::size_t i = 0; i < actual; ++i) threads.emplace_back(worker);
        for (auto& t : threads) t.join();
    }

    // ── Export all reports ─────────────────────────────────────────────────
    ResultExporter::exportComparativeReport(
        output_dir,
        args_.simulation_file.filename().string(),
        args_.mission_control_folder.filename().string(),
        manager_results, error_managers);

    for (const auto& mgr : manager_results) {
        ResultExporter::exportPerSoReport(
            output_dir, mgr.manager_so_name,
            args_.simulation_file.filename().string(),
            mgr.individual_runs);
    }

    ResultExporter::exportErrorLog(output_dir, error_log);

    // Clean up instances before unloading .so libraries
    preloaded_mcs.clear();
    algo_loader.reset();

    std::cout << "[Simulator] Comparative run done -> " << output_dir.string() << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Competitive mode
// ─────────────────────────────────────────────────────────────────────────────
void SimulationEngine::runCompetitive() {
    auto algo_files = discoverSoFiles(args_.algorithms_folder);
    if (algo_files.empty()) {
        std::cerr << "Error: No .so files found in: "
                  << args_.algorithms_folder.string() << "\n";
        return;
    }

    SimulationCompositionData composition;
    try {
        composition = ConfigParser::parseSimulationComposition(args_.simulation_file);
    } catch (const std::exception& ex) {
        std::cerr << "Error parsing composition file: " << ex.what() << "\n";
        return;
    }

    std::vector<SingleRunSpec> run_specs;
    std::size_t run_idx = 0;
    for (const auto& group : composition.simulation_mission_groups) {
        const auto& sim_config = std::get<0>(group);
        const auto& missions   = std::get<1>(group);
        for (const auto& mission_config : missions) {
            for (const auto& drone_config : composition.drone_configs) {
                for (const auto& lidar_config : composition.lidar_configs) {
                    run_specs.push_back(SingleRunSpec{
                        sim_config, mission_config, drone_config, lidar_config, ++run_idx});
                }
            }
        }
    }

    // ── Pre-load all .so files on the main thread ──────────────────────────
    std::vector<std::string> error_log;
    std::vector<std::string> error_algorithms;

    // Pre-load MissionControl .so
    auto mc_loader = preloadLibrary(args_.mission_control_file, error_log);
    if (!mc_loader) {
        std::cerr << "Error: Failed to load mission control .so.\n";
        return;
    }
    auto mc_factory_opt = mc_loader->getMissionControlFactory();
    if (!mc_factory_opt) {
        std::cerr << "Error: MissionControl .so did not register a factory.\n";
        return;
    }
    const common::MissionControlFactory& mc_factory = *mc_factory_opt;

    // Pre-load all Algorithm .so files
    struct PreloadedAlgo {
        std::string so_name;
        common::MappingAlgorithmFactory factory;
        std::unique_ptr<DlLoader> loader;
    };
    std::vector<PreloadedAlgo> preloaded_algos;
    for (const auto& algo_path : algo_files) {
        auto algo_loader = preloadLibrary(algo_path, error_log);
        if (!algo_loader) {
            error_algorithms.push_back(algo_path.filename().string());
            continue;
        }
        auto algo_factory_opt = algo_loader->getAlgorithmFactory();
        if (!algo_factory_opt) {
            error_algorithms.push_back(algo_path.filename().string());
            error_log.push_back(algo_path.filename().string() +
                                " did not register an Algorithm factory.");
            continue;
        }
        preloaded_algos.push_back(PreloadedAlgo{
            algo_path.filename().string(), *algo_factory_opt, std::move(algo_loader)});
    }

    const std::filesystem::path output_dir =
        args_.algorithms_folder /
        ("competition_" + ResultExporter::getFolderTimestamp());
    std::filesystem::create_directories(output_dir);

    std::vector<CompetitiveAlgoResult> algo_results;
    std::mutex results_mutex;
    std::atomic<std::size_t> algo_idx{0};

    auto worker = [&]() {
        while (true) {
            const std::size_t idx = algo_idx.fetch_add(1);
            if (idx >= preloaded_algos.size()) break;

            const auto& algo = preloaded_algos[idx];
            CompetitiveAlgoResult algo_result;
            algo_result.algorithm_so_name = algo.so_name;
            bool algo_failed = false;

            for (const auto& spec : run_specs) {
                std::ostringstream oss;
                oss << "output_map_" << algo.so_name << "_run_" << spec.run_index << ".npy";
                const std::filesystem::path run_file = output_dir / oss.str();

                SingleRunResult run_res = executeSingleRun(
                    algo.factory, mc_factory, spec, run_file, args_.verbose);
                algo_result.individual_runs.push_back(run_res);

                if (run_res.score < 0.0) {
                    algo_failed = true;
                    std::lock_guard<std::mutex> lock(results_mutex);
                    error_log.push_back("[" + algo.so_name + " run " +
                                        std::to_string(spec.run_index) + "] " +
                                        run_res.error_message);
                    break;
                }
                algo_result.total_score += run_res.score;
                algo_result.total_steps += run_res.steps;
            }

            std::lock_guard<std::mutex> lock(results_mutex);
            if (algo_failed) {
                error_algorithms.push_back(algo.so_name);
            } else {
                algo_results.push_back(std::move(algo_result));
            }
        }
    };

    if (args_.num_threads <= 1) {
        worker();
    } else {
        const std::size_t actual = std::min(args_.num_threads, preloaded_algos.size());
        std::vector<std::thread> threads;
        threads.reserve(actual);
        for (std::size_t i = 0; i < actual; ++i) threads.emplace_back(worker);
        for (auto& t : threads) t.join();
    }

    ResultExporter::exportCompetitiveReport(
        output_dir,
        args_.simulation_file.filename().string(),
        args_.mission_control_file.filename().string(),
        algo_results, error_algorithms);

    for (const auto& algo : algo_results) {
        ResultExporter::exportPerSoReport(
            output_dir, algo.algorithm_so_name,
            args_.simulation_file.filename().string(),
            algo.individual_runs);
    }

    ResultExporter::exportErrorLog(output_dir, error_log);

    preloaded_algos.clear();
    mc_loader.reset();

    std::cout << "[Simulator] Competition done -> " << output_dir.string() << "\n";
}

} // namespace simulator_207610130_215664087
