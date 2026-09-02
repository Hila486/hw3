#include <Simulator/SimulationEngine.h>
#include <Simulator/DlLoader.h>
#include <Simulator/ResultExporter.h>
#include <Simulator/ConfigParser.h>
#include <Simulator/Map3DImpl.h>
#include <Simulator/MapsComparison.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/NpyMapIO.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace simulator_207610130_215664087 {

namespace {

    //helper functions for SimulationEngine

// Discovers all .so files in a folder matching prefix, sorted by path.
//Return a list of filesystem paths
std::vector<std::filesystem::path> discoverSoFiles(
    const std::filesystem::path& folder,
    const std::string& expected_prefix = "") {

    std::vector<std::filesystem::path> files;
    try {
        if (!std::filesystem::is_directory(folder)) {
            return files;
        }
        for (const auto& entry : std::filesystem::directory_iterator(folder)) {
            if (entry.is_regular_file() && entry.path().extension() == ".so") {
                const std::string filename = entry.path().filename().string();
                if (expected_prefix.empty() || filename.rfind(expected_prefix, 0) == 0) {
                    files.push_back(entry.path());
                }
            }
        }
    } catch (...) {
        return files;
    }
    std::sort(files.begin(), files.end());
    return files;
}

//small data container  to remember the original filenames from the simulation composition YAML
struct RawCompositionLayout {
    // Represents one simulation configuration + all the missions belonging to that simulation
    struct SimGroup {
        std::string simulation_config;
        std::vector<std::string> mission_configs;
    };

    std::vector<SimGroup> simulations;
    std::vector<std::string> drone_configs;
    std::vector<std::string> lidar_configs;
};

/*
    * Reads and returns the raw layout of the simulation composition YAML file.
    * @param filepath Path to the simulation composition YAML file.
    */
RawCompositionLayout readRawLayout(const std::filesystem::path& filepath) {
    RawCompositionLayout layout;
    try {
        const YAML::Node root = YAML::LoadFile(filepath.string());
        const YAML::Node compositions = root["simulation_compositions"];
        if (compositions) {
            const YAML::Node simulations = compositions["simulations"];
            if (simulations && simulations.IsSequence()) {
                for (const auto& sim : simulations) {
                    RawCompositionLayout::SimGroup group;
                    if (sim["simulation_config"]) {
                        group.simulation_config = sim["simulation_config"].as<std::string>();
                    }
                    if (sim["mission_configs"] && sim["mission_configs"].IsSequence()) {
                        for (const auto& mc : sim["mission_configs"]) {
                            group.mission_configs.push_back(mc.as<std::string>());
                        }
                    }
                    layout.simulations.push_back(std::move(group));
                }
            }

            if (compositions["drone_configs"] && compositions["drone_configs"].IsSequence()) {
                for (const auto& dc : compositions["drone_configs"]) {
                    layout.drone_configs.push_back(dc.as<std::string>());
                }
            }

            if (compositions["lidar_configs"] && compositions["lidar_configs"].IsSequence()) {
                for (const auto& lc : compositions["lidar_configs"]) {
                    layout.lidar_configs.push_back(lc.as<std::string>());
                }
            }
        }
    } catch (...) {
        // Handled by ConfigParser
        //This function intentionally doesn't report the YAML parsing error.it is only helping preserve filenames.
    }
    return layout;
}
// ----------------------------------------
/// A single (simulation × mission × drone × lidar) run specification.
struct SingleRunSpec {
    SimulationConfigData simulation_config;
    common::types::MissionConfigData mission_config;
    common::types::DroneConfigData drone_config;
    common::types::LidarConfigData lidar_config;
    std::string simulation_config_name;
    std::string mission_config_name;
    std::string drone_config_name;
    std::string lidar_config_name;
    std::size_t run_index = 0;
};

/// Converts MissionRunStatus to string.
std::string statusToString(common::types::MissionRunStatus status) {
    switch (status) {
        case common::types::MissionRunStatus::Completed: return "completed";
        case common::types::MissionRunStatus::MaxSteps:  return "max_steps";
        case common::types::MissionRunStatus::Error:     return "error";
    }
    return "error";
}

/// Executes one single simulation run using pre-loaded factories.
SingleRunResult executeSingleRun(
    const common::MappingAlgorithmFactory& algo_factory,
    const common::MissionControlFactory& mc_factory,
    const SingleRunSpec& spec,
    const std::filesystem::path& run_output_file,
    const std::filesystem::path& output_dir,
    bool verbose) {

    SingleRunResult result;
    result.run_index = spec.run_index;
    result.simulation_config_name = spec.simulation_config_name;
    result.mission_config_name = spec.mission_config_name;
    result.drone_config_name = spec.drone_config_name;
    result.lidar_config_name = spec.lidar_config_name;
    result.output_map_file = run_output_file;

    try {
        // --- 1. Load and validate ground truth (hidden) map ---
        auto hidden_map_data = loadNormalizedNpyMap(spec.simulation_config.map_filename);
        validateInputMapValues(*hidden_map_data, spec.simulation_config.map_filename);
        NpyMapShape hidden_shape = npyMapShape(*hidden_map_data, spec.simulation_config.map_filename);

        // Derive hidden map physical boundaries from NPY array dimensions (covers full building)
        const double map_res_cm = spec.simulation_config.map_resolution.force_numerical_value_in(cm);
        const double full_width_cm = static_cast<double>(hidden_shape.dim_x) * map_res_cm;
        const double full_length_cm = static_cast<double>(hidden_shape.dim_y) * map_res_cm;
        const double full_height_cm = static_cast<double>(hidden_shape.dim_z) * map_res_cm;

        const double offset_x = spec.simulation_config.map_axes_offset.x.force_numerical_value_in(cm);
        const double offset_y = spec.simulation_config.map_axes_offset.y.force_numerical_value_in(cm);
        const double offset_z = spec.simulation_config.map_axes_offset.z.force_numerical_value_in(cm);

        common::types::MappingBounds full_map_bounds{
            -offset_x * x_extent[cm],
            (full_width_cm - offset_x) * x_extent[cm],
            -offset_y * y_extent[cm],
            (full_length_cm - offset_y) * y_extent[cm],
            -offset_z * z_extent[cm],
            (full_height_cm - offset_z) * z_extent[cm]
        };
        common::types::MapConfig hidden_map_config{
            full_map_bounds,
            spec.simulation_config.map_axes_offset,
            spec.simulation_config.map_resolution
        };
        Map3DImpl hidden_map(hidden_map_data, hidden_map_config);

        // --- 2. Output mapping resolution based on GPS resolution ---
        const double factor = spec.mission_config.output_mapping_resolution_factor;
        common::PhysicalLength output_resolution;

        if (factor < 1.0) {
            result.resolution_request_status = "IGNORED TOO SMALL";
            output_resolution = spec.mission_config.gps_resolution;
            ResultExporter::logErrorImmediately(
                output_dir,
                "[Run " + std::to_string(spec.run_index) +
                "] output_mapping_resolution_factor < 1 (" + std::to_string(factor) +
                ") is ignored. Using GPS resolution.");
        } else if (std::abs(factor - 1.0) < 1.0e-9) {
            result.resolution_request_status = "ACCEPTED";
            output_resolution = spec.mission_config.gps_resolution;
        } else {
            result.resolution_request_status = "IGNORED";
            output_resolution = spec.mission_config.gps_resolution;
        }
        result.resolution_cm = output_resolution.force_numerical_value_in(cm);

        // Derive output map voxel grid dimensions matching the physical world building extent at output_resolution
        const double out_res_cm = result.resolution_cm > 0.0 ? result.resolution_cm : 10.0;
        NpyMapShape output_shape{
            static_cast<std::size_t>(std::max(1.0, std::ceil(full_width_cm / out_res_cm))),
            static_cast<std::size_t>(std::max(1.0, std::ceil(full_length_cm / out_res_cm))),
            static_cast<std::size_t>(std::max(1.0, std::ceil(full_height_cm / out_res_cm)))
        };

        auto output_map_data = makeFilledIntNpyArray(
            output_shape,
            static_cast<int>(common::types::VoxelOccupancy::Unmapped));

        common::types::MapConfig output_map_config{
            spec.mission_config.mission_bounds,
            spec.simulation_config.map_axes_offset,
            output_resolution
        };
        Map3DImpl output_map(output_map_data, output_map_config);

        // --- 3. Hardware mocks ---
        MockGPS gps(spec.simulation_config.initial_drone_position,
                    Orientation{spec.simulation_config.initial_angle,
                                0.0 * altitude_angle[deg]},
                    spec.mission_config.gps_resolution);
        MockMovement movement(gps, &hidden_map, spec.drone_config.radius);
        MockLidar lidar(spec.lidar_config, hidden_map, gps);

        // --- 4. Instantiate Algorithm and MissionControl from pre-loaded factories ---
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

        // --- 5. Run the mission ---
        common::types::MissionRunResult run_res = mission_control->runMission();
        result.steps = run_res.steps;
        result.status = statusToString(run_res.status);

        if (run_res.status == common::types::MissionRunStatus::Error) {
            result.score = -1.0;
            if (!run_res.errors.empty()) {
                result.error_code = run_res.errors[0].code;
                result.error_message = run_res.errors[0].message;
            } else {
                result.error_code = "MISSION_ERROR";
                result.error_message = "Mission control reported an error.";
            }
            ResultExporter::logErrorImmediately(
                output_dir,
                "[Run " + std::to_string(spec.run_index) + "] " + result.error_message);

            mission_control.reset();
            mapping_algorithm.reset();
            return result;
        }

        // --- 6. Score the output map vs the hidden map ---
        std::vector<common::IMap3D*> targets{&output_map};
        std::vector<double> scores = MapsComparison::compare(hidden_map, targets);
        result.score = scores.empty() ? 0.0 : scores[0];

        // Explicitly destroy instances BEFORE returning
        mission_control.reset();
        mapping_algorithm.reset();

    } catch (const std::exception& ex) {
        result.error_code = "EXCEPTION";
        result.error_message = std::string("Exception: ") + ex.what();
        result.status = "error";
        result.score = -1.0;
        ResultExporter::logErrorImmediately(
            output_dir,
            "[Run " + std::to_string(spec.run_index) + "] " + result.error_message);
    }

    return result;
}

/// Pre-loads a .so file and logs errors immediately. Returns nullptr on failure.
std::unique_ptr<DlLoader> preloadLibrary(
    const std::filesystem::path& so_path,
    const std::filesystem::path& output_dir) {

    auto loader = std::make_unique<DlLoader>(so_path);
    if (!loader->load()) {
        ResultExporter::logErrorImmediately(
            output_dir,
            "Failed to load " + so_path.filename().string() + ": " + loader->error());
        return nullptr;
    }
    return loader;
}

/// Creates a unique collision-free output directory.
std::filesystem::path createUniqueOutputDir(
    const std::filesystem::path& parent_dir,
    const std::string& prefix) {

    std::filesystem::path output_dir;
    int suffix = 0;
    do {
        std::string name = prefix + "_" + ResultExporter::getFolderTimestamp();
        if (suffix > 0) {
            name += "_" + std::to_string(suffix);
        }
        output_dir = parent_dir / name;
        ++suffix;
    } while (std::filesystem::exists(output_dir));

    std::error_code ec;
    std::filesystem::create_directories(output_dir, ec);
    if (ec) {
        std::cerr << "Error: Failed to create output directory: "
                  << output_dir.string() << " (" << ec.message() << ")\n";
        return {};
    }
    {
        std::ofstream init_log(output_dir / "error_log.txt");
    }
    return output_dir;
}

} // namespace

SimulationEngine::SimulationEngine(ParsedArgs args) : args_(std::move(args)) {}

bool SimulationEngine::run() {
    if (args_.mode == ExecutionMode::Comparative) {
        return runComparative();
    } else {
        return runCompetitive();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Comparative mode
// ─────────────────────────────────────────────────────────────────────────────
bool SimulationEngine::runComparative() {
    const auto mc_files = discoverSoFiles(args_.mission_control_folder, "MissionControl");
    if (mc_files.empty()) {
        std::cerr << "Error: No MissionControl*.so files found in: "
                  << args_.mission_control_folder.string() << "\n";
        return false;
    }

    SimulationCompositionData composition;
    try {
        composition = ConfigParser::parseSimulationComposition(args_.simulation_file);
    } catch (const std::exception& ex) {
        std::cerr << "Error parsing composition file: " << ex.what() << "\n";
        return false;
    }

    const RawCompositionLayout raw_layout = readRawLayout(args_.simulation_file);

    const std::filesystem::path output_dir =
        createUniqueOutputDir(args_.mission_control_folder, "comparative_results");
    if (output_dir.empty()) {
        return false;
    }

    std::vector<SingleRunSpec> run_specs;
    std::size_t current_run_idx = 0;

    for (std::size_t s = 0; s < composition.simulation_mission_groups.size(); ++s) {
        const auto& group = composition.simulation_mission_groups[s];
        const auto& sim_config = std::get<0>(group);
        const auto& missions   = std::get<1>(group);

        std::string sim_name = (s < raw_layout.simulations.size())
                                   ? raw_layout.simulations[s].simulation_config
                                   : "simulation.yaml";

        for (std::size_t m = 0; m < missions.size(); ++m) {
            const auto& mission_config = missions[m];
            std::string mission_name =
                (s < raw_layout.simulations.size() && m < raw_layout.simulations[s].mission_configs.size())
                    ? raw_layout.simulations[s].mission_configs[m]
                    : "mission.yaml";

            for (std::size_t d = 0; d < composition.drone_configs.size(); ++d) {
                const auto& drone_config = composition.drone_configs[d];
                std::string drone_name = (d < raw_layout.drone_configs.size())
                                             ? raw_layout.drone_configs[d]
                                             : "drone.yaml";

                for (std::size_t l = 0; l < composition.lidar_configs.size(); ++l) {
                    const auto& lidar_config = composition.lidar_configs[l];
                    std::string lidar_name = (l < raw_layout.lidar_configs.size())
                                                 ? raw_layout.lidar_configs[l]
                                                 : "lidar.yaml";

                    run_specs.push_back(SingleRunSpec{
                        sim_config, mission_config, drone_config, lidar_config,
                        sim_name, mission_name, drone_name, lidar_name,
                        ++current_run_idx
                    });
                }
            }
        }
    }

    if (run_specs.empty()) {
        std::cerr << "Error: No simulation scenarios generated from composition file.\n";
        return false;
    }

    // ── Pre-load all .so files on the main thread ──────────────────────────
    std::vector<std::string> error_managers;

    auto algo_loader = preloadLibrary(args_.algorithm_file, output_dir);
    if (!algo_loader) {
        std::cerr << "Error: Failed to load algorithm .so.\n";
        return false;
    }
    auto algo_factory_opt = algo_loader->getAlgorithmFactory();
    if (!algo_factory_opt) {
        std::cerr << "Error: Algorithm .so did not register a factory.\n";
        return false;
    }
    common::MappingAlgorithmFactory algo_factory = *algo_factory_opt;

    struct PreloadedMC {
        std::string so_name;
        std::unique_ptr<DlLoader> loader;
        common::MissionControlFactory factory;
    };
    std::vector<PreloadedMC> preloaded_mcs;
    for (const auto& mc_path : mc_files) {
        auto mc_loader = preloadLibrary(mc_path, output_dir);
        if (!mc_loader) {
            error_managers.push_back(mc_path.filename().string());
            continue;
        }
        auto mc_factory_opt = mc_loader->getMissionControlFactory();
        if (!mc_factory_opt) {
            error_managers.push_back(mc_path.filename().string());
            ResultExporter::logErrorImmediately(
                output_dir,
                mc_path.filename().string() + " did not register a MissionControl factory.");
            continue;
        }
        preloaded_mcs.push_back(PreloadedMC{
            mc_path.filename().string(), std::move(mc_loader), *mc_factory_opt});
    }

    if (preloaded_mcs.empty()) {
        std::cerr << "Error: No MissionControl .so files could be loaded.\n";
        ResultExporter::exportComparativeReport(
            output_dir,
            args_.simulation_file.filename().string(),
            args_.mission_control_folder.filename().string(),
            {}, error_managers);
        return false;
    }

    // ── Build fine-grained flat job table: MC × run_spec ────────────────────
    struct SimulationJob {
        std::size_t mc_idx;
        std::size_t spec_idx;
        std::size_t job_idx;
    };

    std::vector<SimulationJob> jobs;
    jobs.reserve(preloaded_mcs.size() * run_specs.size());
    for (std::size_t m = 0; m < preloaded_mcs.size(); ++m) {
        for (std::size_t s = 0; s < run_specs.size(); ++s) {
            jobs.push_back(SimulationJob{m, s, jobs.size() + 1});
        }
    }

    std::vector<std::vector<SingleRunResult>> all_results(
        preloaded_mcs.size(), std::vector<SingleRunResult>(run_specs.size()));

    std::atomic<std::size_t> next_job_idx{0};

    auto worker = [&]() {
        while (true) {
            const std::size_t idx = next_job_idx.fetch_add(1);
            if (idx >= jobs.size()) {
                break;
            }

            const auto& job = jobs[idx];
            const auto& mc = preloaded_mcs[job.mc_idx];
            const auto& spec = run_specs[job.spec_idx];

            std::ostringstream oss;
            oss << "output_map_" << mc.so_name << "_run_" << spec.run_index << ".npy";
            const std::filesystem::path run_file = output_dir / oss.str();

            SingleRunResult run_res = executeSingleRun(
                algo_factory, mc.factory, spec, run_file, output_dir, args_.verbose);

            all_results[job.mc_idx][job.spec_idx] = std::move(run_res);
        }
    };

    // ── Launch worker threads (Total threads is NEVER 2) ───────────────────
    if (args_.num_threads <= 1 || jobs.size() < 2) {
        worker();
    } else {
        const std::size_t thread_count = std::min(args_.num_threads, jobs.size());
        std::vector<std::thread> threads;
        threads.reserve(thread_count);
        for (std::size_t i = 0; i < thread_count; ++i) {
            threads.emplace_back(worker);
        }
        for (auto& t : threads) {
            t.join();
        }
    }

    // ── Aggregate results per manager ──────────────────────────────────────
    std::vector<ComparativeManagerResult> manager_results;
    for (std::size_t m = 0; m < preloaded_mcs.size(); ++m) {
        ComparativeManagerResult mgr_result;
        mgr_result.manager_so_name = preloaded_mcs[m].so_name;
        mgr_result.individual_runs = std::move(all_results[m]);

        for (const auto& run : mgr_result.individual_runs) {
            mgr_result.total_score += run.score;
            mgr_result.total_steps += run.steps;
        }
        manager_results.push_back(std::move(mgr_result));
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
            args_.simulation_file,
            mgr.individual_runs);
    }

    // ── Strict Lifetime Cleanup: Reset all factory references BEFORE dlclose
    for (auto& mc : preloaded_mcs) {
        mc.factory = nullptr;
    }
    preloaded_mcs.clear();

    algo_factory = nullptr;
    algo_factory_opt.reset();
    algo_loader.reset();

    std::cout << "[Simulator] Comparative run done -> " << output_dir.string() << "\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Competitive mode
// ─────────────────────────────────────────────────────────────────────────────
bool SimulationEngine::runCompetitive() {
    const auto algo_files = discoverSoFiles(args_.algorithms_folder, "Algorithm");
    if (algo_files.empty()) {
        std::cerr << "Error: No Algorithm*.so files found in: "
                  << args_.algorithms_folder.string() << "\n";
        return false;
    }

    SimulationCompositionData composition;
    try {
        composition = ConfigParser::parseSimulationComposition(args_.simulation_file);
    } catch (const std::exception& ex) {
        std::cerr << "Error parsing composition file: " << ex.what() << "\n";
        return false;
    }

    const RawCompositionLayout raw_layout = readRawLayout(args_.simulation_file);

    const std::filesystem::path output_dir =
        createUniqueOutputDir(args_.algorithms_folder, "competition");
    if (output_dir.empty()) {
        return false;
    }

    std::vector<SingleRunSpec> run_specs;
    std::size_t current_run_idx = 0;

    for (std::size_t s = 0; s < composition.simulation_mission_groups.size(); ++s) {
        const auto& group = composition.simulation_mission_groups[s];
        const auto& sim_config = std::get<0>(group);
        const auto& missions   = std::get<1>(group);

        std::string sim_name = (s < raw_layout.simulations.size())
                                   ? raw_layout.simulations[s].simulation_config
                                   : "simulation.yaml";

        for (std::size_t m = 0; m < missions.size(); ++m) {
            const auto& mission_config = missions[m];
            std::string mission_name =
                (s < raw_layout.simulations.size() && m < raw_layout.simulations[s].mission_configs.size())
                    ? raw_layout.simulations[s].mission_configs[m]
                    : "mission.yaml";

            for (std::size_t d = 0; d < composition.drone_configs.size(); ++d) {
                const auto& drone_config = composition.drone_configs[d];
                std::string drone_name = (d < raw_layout.drone_configs.size())
                                             ? raw_layout.drone_configs[d]
                                             : "drone.yaml";

                for (std::size_t l = 0; l < composition.lidar_configs.size(); ++l) {
                    const auto& lidar_config = composition.lidar_configs[l];
                    std::string lidar_name = (l < raw_layout.lidar_configs.size())
                                                 ? raw_layout.lidar_configs[l]
                                                 : "lidar.yaml";

                    run_specs.push_back(SingleRunSpec{
                        sim_config, mission_config, drone_config, lidar_config,
                        sim_name, mission_name, drone_name, lidar_name,
                        ++current_run_idx
                    });
                }
            }
        }
    }

    if (run_specs.empty()) {
        std::cerr << "Error: No simulation scenarios generated from composition file.\n";
        return false;
    }

    // ── Pre-load all .so files on the main thread ──────────────────────────
    std::vector<std::string> error_algorithms;

    auto mc_loader = preloadLibrary(args_.mission_control_file, output_dir);
    if (!mc_loader) {
        std::cerr << "Error: Failed to load mission control .so.\n";
        return false;
    }
    auto mc_factory_opt = mc_loader->getMissionControlFactory();
    if (!mc_factory_opt) {
        std::cerr << "Error: MissionControl .so did not register a factory.\n";
        return false;
    }
    common::MissionControlFactory mc_factory = *mc_factory_opt;

    struct PreloadedAlgo {
        std::string so_name;
        std::unique_ptr<DlLoader> loader;
        common::MappingAlgorithmFactory factory;
    };
    std::vector<PreloadedAlgo> preloaded_algos;
    for (const auto& algo_path : algo_files) {
        auto algo_loader = preloadLibrary(algo_path, output_dir);
        if (!algo_loader) {
            error_algorithms.push_back(algo_path.filename().string());
            continue;
        }
        auto algo_factory_opt = algo_loader->getAlgorithmFactory();
        if (!algo_factory_opt) {
            error_algorithms.push_back(algo_path.filename().string());
            ResultExporter::logErrorImmediately(
                output_dir,
                algo_path.filename().string() + " did not register an Algorithm factory.");
            continue;
        }
        preloaded_algos.push_back(PreloadedAlgo{
            algo_path.filename().string(), std::move(algo_loader), *algo_factory_opt});
    }

    if (preloaded_algos.empty()) {
        std::cerr << "Error: No Algorithm .so files could be loaded.\n";
        ResultExporter::exportCompetitiveReport(
            output_dir,
            args_.simulation_file.filename().string(),
            args_.mission_control_file.filename().string(),
            {}, error_algorithms);
        return false;
    }

    // ── Build fine-grained flat job table: Algorithm × run_spec ─────────────
    struct SimulationJob {
        std::size_t algo_idx;
        std::size_t spec_idx;
        std::size_t job_idx;
    };

    std::vector<SimulationJob> jobs;
    jobs.reserve(preloaded_algos.size() * run_specs.size());
    for (std::size_t a = 0; a < preloaded_algos.size(); ++a) {
        for (std::size_t s = 0; s < run_specs.size(); ++s) {
            jobs.push_back(SimulationJob{a, s, jobs.size() + 1});
        }
    }

    std::vector<std::vector<SingleRunResult>> all_results(
        preloaded_algos.size(), std::vector<SingleRunResult>(run_specs.size()));

    std::atomic<std::size_t> next_job_idx{0};

    auto worker = [&]() {
        while (true) {
            const std::size_t idx = next_job_idx.fetch_add(1);
            if (idx >= jobs.size()) {
                break;
            }

            const auto& job = jobs[idx];
            const auto& algo = preloaded_algos[job.algo_idx];
            const auto& spec = run_specs[job.spec_idx];

            std::ostringstream oss;
            oss << "output_map_" << algo.so_name << "_run_" << spec.run_index << ".npy";
            const std::filesystem::path run_file = output_dir / oss.str();

            SingleRunResult run_res = executeSingleRun(
                algo.factory, mc_factory, spec, run_file, output_dir, args_.verbose);

            all_results[job.algo_idx][job.spec_idx] = std::move(run_res);
        }
    };

    // ── Launch worker threads (Total threads is NEVER 2) ───────────────────
    if (args_.num_threads <= 1 || jobs.size() < 2) {
        worker();
    } else {
        const std::size_t thread_count = std::min(args_.num_threads, jobs.size());
        std::vector<std::thread> threads;
        threads.reserve(thread_count);
        for (std::size_t i = 0; i < thread_count; ++i) {
            threads.emplace_back(worker);
        }
        for (auto& t : threads) {
            t.join();
        }
    }

    std::vector<CompetitiveAlgoResult> algo_results;
    for (std::size_t a = 0; a < preloaded_algos.size(); ++a) {
        CompetitiveAlgoResult algo_result;
        algo_result.algorithm_so_name = preloaded_algos[a].so_name;
        algo_result.individual_runs = std::move(all_results[a]);

        for (const auto& run : algo_result.individual_runs) {
            algo_result.total_score += run.score;
            algo_result.total_steps += run.steps;
        }
        algo_results.push_back(std::move(algo_result));
    }

    ResultExporter::exportCompetitiveReport(
        output_dir,
        args_.simulation_file.filename().string(),
        args_.mission_control_file.filename().string(),
        algo_results, error_algorithms);

    for (const auto& algo : algo_results) {
        ResultExporter::exportPerSoReport(
            output_dir, algo.algorithm_so_name,
            args_.simulation_file,
            algo.individual_runs);
    }

    // ── Strict Lifetime Cleanup: Reset all factory references BEFORE dlclose
    for (auto& algo : preloaded_algos) {
        algo.factory = nullptr;
    }
    preloaded_algos.clear();

    mc_factory = nullptr;
    mc_factory_opt.reset();
    mc_loader.reset();

    std::cout << "[Simulator] Competition done -> " << output_dir.string() << "\n";
    return true;
}

} // namespace simulator_207610130_215664087
