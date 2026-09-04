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
// -----------------------------------------------------------------------------
// Helper functions and structures used internally by SimulationEngine.
// -----------------------------------------------------------------------------

// Finds all .so files in a folder whose filename starts with the given prefix.
// The returned paths are sorted to keep execution deterministic.
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

std::string getCleanDirectoryName(const std::filesystem::path& p) {
    auto norm = p.lexically_normal();
    if (norm.filename().empty()) {
        norm = norm.parent_path();
    }
    return norm.filename().string();
}

// Stores the original filenames appearing in the composition YAML.
// ConfigParser gives us parsed objects, while this structure preserves
// the names needed later for reports.
struct RawCompositionLayout {
    // One simulation configuration and its related mission configurations.
    struct SimGroup {
        std::string simulation_config;
        std::vector<std::string> mission_configs;
    };

    std::vector<SimGroup> simulations;
    std::vector<std::string> drone_configs;
    std::vector<std::string> lidar_configs;
};

// Reads the composition YAML only to preserve the original configuration names.
// Actual validation/parsing is handled separately by ConfigParser.
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
        // ConfigParser handles real YAML errors.
        // This helper is only used to preserve filenames.
    }
    return layout;
}

// Describes one complete simulation run:
// simulation × mission × drone × lidar.
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


// Converts MissionRunStatus into text used in reports.
std::string statusToString(common::types::MissionRunStatus status) {
    switch (status) {
        case common::types::MissionRunStatus::Completed: return "completed";
        case common::types::MissionRunStatus::MaxSteps:  return "max_steps";
        case common::types::MissionRunStatus::Error:     return "error";
    }
    return "error";
}


// Executes one complete simulation scenario using already-loaded
// Algorithm and MissionControl factories.
SingleRunResult executeSingleRun(
    const common::MappingAlgorithmFactory& algo_factory,
    const common::MissionControlFactory& mc_factory,
    const SingleRunSpec& spec,
    const std::filesystem::path& run_output_file,
    const std::filesystem::path& output_dir,
    bool verbose) {

    SingleRunResult result;
    // Store identifying information for the final report.
    result.run_index = spec.run_index;
    result.simulation_config_name = spec.simulation_config_name;
    result.mission_config_name = spec.mission_config_name;
    result.drone_config_name = spec.drone_config_name;
    result.lidar_config_name = spec.lidar_config_name;
    result.output_map_file = run_output_file;

    try {
        // ---------------------------------------------------------------------
        // 1. Load the hidden real map.
        // This map represents the actual building and is never exposed
        // directly to the mapping algorithm.
        // ---------------------------------------------------------------------
        auto hidden_map_data = loadNormalizedNpyMap(spec.simulation_config.map_filename);
        validateInputMapValues(*hidden_map_data, spec.simulation_config.map_filename);
        NpyMapShape hidden_shape = npyMapShape(*hidden_map_data, spec.simulation_config.map_filename);

        // Calculate the physical size represented by the NPY dimensions.
        const double map_res_cm = spec.simulation_config.map_resolution.force_numerical_value_in(cm);
        const double full_width_cm = static_cast<double>(hidden_shape.dim_x) * map_res_cm;
        const double full_length_cm = static_cast<double>(hidden_shape.dim_y) * map_res_cm;
        const double full_height_cm = static_cast<double>(hidden_shape.dim_z) * map_res_cm;

        // Offsets translate between world coordinates
        // and coordinates inside the NPY array.
        const double offset_x = spec.simulation_config.map_axes_offset.x.force_numerical_value_in(cm);
        const double offset_y = spec.simulation_config.map_axes_offset.y.force_numerical_value_in(cm);
        const double offset_z = spec.simulation_config.map_axes_offset.z.force_numerical_value_in(cm);

         // Hidden map covers the complete physical building.
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

        

        // ---------------------------------------------------------------------
        // 2. Create the empty output map that the drone will build.
        // ---------------------------------------------------------------------

        const double factor = spec.mission_config.output_mapping_resolution_factor;
        common::PhysicalLength output_resolution;

        // Current implementation always uses GPS resolution.
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

        // Calculate how many voxels are required to represent
        // the physical map at the selected output resolution.
        const double out_res_cm = result.resolution_cm > 0.0 ? result.resolution_cm : 10.0;
        NpyMapShape output_shape{
            static_cast<std::size_t>(std::max(1.0, std::ceil(full_width_cm / out_res_cm))),
            static_cast<std::size_t>(std::max(1.0, std::ceil(full_length_cm / out_res_cm))),
            static_cast<std::size_t>(std::max(1.0, std::ceil(full_height_cm / out_res_cm)))
        };

        // Start with every voxel marked as Unmapped.
        auto output_map_data = makeFilledIntNpyArray(
            output_shape,
            static_cast<int>(common::types::VoxelOccupancy::Unmapped));

        common::types::MapConfig output_map_config{
            spec.mission_config.mission_bounds,
            spec.simulation_config.map_axes_offset,
            output_resolution
        };
        Map3DImpl output_map(output_map_data, output_map_config);

        // ---------------------------------------------------------------------
        // 3. Create simulated hardware.
        // ---------------------------------------------------------------------
        MockGPS gps(spec.simulation_config.initial_drone_position,
                    Orientation{spec.simulation_config.initial_angle,
                                0.0 * altitude_angle[deg]},
                    spec.mission_config.gps_resolution);
        // Movement uses the hidden map only for real collision detection.
        MockMovement movement(gps, &hidden_map, spec.drone_config.radius);
        // Lidar reads the hidden map to simulate real sensor measurements.
        MockLidar lidar(spec.lidar_config, hidden_map, gps);

        
        // ---------------------------------------------------------------------
        // 4. Create Algorithm and MissionControl instances from factories.
        // ---------------------------------------------------------------------
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

        
        // ---------------------------------------------------------------------
        // 5. Run the complete mission.
        // ---------------------------------------------------------------------
        common::types::MissionRunResult run_res = mission_control->runMission();
        result.steps = run_res.steps;
        result.status = statusToString(run_res.status);
        // Handle mission failure.
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

            
            // Destroy plugin-created objects while their .so is still loaded.
            mission_control.reset();
            mapping_algorithm.reset();
            return result;
        }

        // ---------------------------------------------------------------------
        // 6. Compare the produced output map with the hidden real map.
        // ---------------------------------------------------------------------
        std::vector<common::IMap3D*> targets{&output_map};
        std::vector<double> scores = MapsComparison::compare(hidden_map, targets);
        result.score = scores.empty() ? 0.0 : scores[0];

        // Mark whether the output map file was actually written to disk.
        result.output_map_written = std::filesystem::exists(run_output_file);

         // Explicitly destroy plugin objects before library unloading.
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
    } catch (...) {
        result.error_code = "UNKNOWN_EXCEPTION";
        result.error_message = "Unknown non-standard exception caught during simulation run.";
        result.status = "error";
        result.score = -1.0;
        ResultExporter::logErrorImmediately(
            output_dir,
            "[Run " + std::to_string(spec.run_index) + "] " + result.error_message);
    }

    return result;
}


// Loads one .so before worker threads start.
// Errors are written immediately to the output directory.
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

// Creates a new output directory with a timestamp.
// If the name already exists, a numeric suffix is added.
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
    // Create an empty error log immediately.
    {
        std::ofstream init_log(output_dir / "error_log.txt");
    }
    return output_dir;
}

} // namespace

// Stores the command-line arguments used to configure the simulation.
SimulationEngine::SimulationEngine(ParsedArgs args) : args_(std::move(args)) {}

// Selects which simulation mode to run.
bool SimulationEngine::run() {
    if (args_.mode == ExecutionMode::Comparative) {
        return runComparative();
    } else {
        return runCompetitive();
    }
}

// =============================================================================
// COMPARATIVE MODE
//
// One Algorithm is tested with multiple MissionControl implementations.
// =============================================================================
bool SimulationEngine::runComparative() {
    // Find all MissionControl plugins.
    const auto mc_files = discoverSoFiles(args_.mission_control_folder, "MissionControl");
    if (mc_files.empty()) {
        std::cerr << "Error: No MissionControl*.so files found in: "
                  << args_.mission_control_folder.string() << "\n";
        return false;
    }
    // Create output directory FIRST so that any subsequent parse errors
    // can be logged to error_log.txt inside it.
    const std::filesystem::path output_dir =
        createUniqueOutputDir(args_.mission_control_folder, "comparative_results");
    if (output_dir.empty()) {
        return false;
    }

    // Parse all simulation, mission, drone and lidar configurations.
    SimulationCompositionData composition;
    try {
        composition = ConfigParser::parseSimulationComposition(args_.simulation_file);
    } catch (const std::exception& ex) {
        const std::string msg = std::string("Error parsing composition file: ") + ex.what();
        std::cerr << msg << "\n";
        ResultExporter::logErrorImmediately(output_dir, msg);
        ResultExporter::exportComparativeReport(
            output_dir,
            args_.simulation_file.filename().string(),
            getCleanDirectoryName(args_.mission_control_folder),
            {}, {});
        return false;
    }
    // Preserve original YAML filenames for the reports.
    const RawCompositionLayout raw_layout = readRawLayout(args_.simulation_file);

    // -------------------------------------------------------------------------
    // Build every simulation combination.
    //
    // simulation × mission × drone × lidar
    // -------------------------------------------------------------------------
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
        const std::string msg = "Error: No simulation scenarios generated from composition file.";
        std::cerr << msg << "\n";
        ResultExporter::logErrorImmediately(output_dir, msg);
        ResultExporter::exportComparativeReport(
            output_dir,
            args_.simulation_file.filename().string(),
            getCleanDirectoryName(args_.mission_control_folder),
            {}, {});
        return false;
    }

    // -------------------------------------------------------------------------
    // Pre-load plugins on the main thread.
    //
    // Dynamic loading and registration happen before worker threads begin.
    // -------------------------------------------------------------------------

    std::vector<std::string> error_managers;

     // Load the single Algorithm used for every comparative run.
    auto algo_loader = preloadLibrary(args_.algorithm_file, output_dir);
    if (!algo_loader) {
        std::cerr << "Error: Failed to load algorithm .so.\n";
        ResultExporter::exportComparativeReport(
            output_dir,
            args_.simulation_file.filename().string(),
            getCleanDirectoryName(args_.mission_control_folder),
            {}, {args_.algorithm_file.filename().string()});
        return false;
    }
    auto algo_factory_opt = algo_loader->getAlgorithmFactory();
    if (!algo_factory_opt) {
        std::cerr << "Error: Algorithm .so did not register a factory.\n";
        ResultExporter::exportComparativeReport(
            output_dir,
            args_.simulation_file.filename().string(),
            getCleanDirectoryName(args_.mission_control_folder),
            {}, {args_.algorithm_file.filename().string()});
        return false;
    }
    common::MappingAlgorithmFactory algo_factory = *algo_factory_opt;

    // Stores each loaded MissionControl together with its factory.
    struct PreloadedMC {
        std::string so_name;
        std::unique_ptr<DlLoader> loader;
        common::MissionControlFactory factory;
    };
    std::vector<PreloadedMC> preloaded_mcs;

    // Load all MissionControl plugins.
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

    // -------------------------------------------------------------------------
    // Build the parallel job table.
    //
    // Every job = one MissionControl × one simulation specification.
    // -------------------------------------------------------------------------

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
    // Each MissionControl gets a result for every run specification.
    std::vector<std::vector<SingleRunResult>> all_results(
        preloaded_mcs.size(), std::vector<SingleRunResult>(run_specs.size()));

     // Shared atomic counter used by threads to claim jobs safely.
    std::atomic<std::size_t> next_job_idx{0};

      // Worker repeatedly claims one job and executes it.
    auto worker = [&]() {
        while (true) {
             // fetch_add atomically gives each thread a unique job index.
            const std::size_t idx = next_job_idx.fetch_add(1);
            if (idx >= jobs.size()) {
                break;
            }

            const auto& job = jobs[idx];
            const auto& mc = preloaded_mcs[job.mc_idx];
            const auto& spec = run_specs[job.spec_idx];

            // Each simulation writes to its own output map file.
            std::ostringstream oss;
            oss << "output_map_" << mc.so_name << "_run_" << spec.run_index << ".npy";
            const std::filesystem::path run_file = output_dir / oss.str();

            SingleRunResult run_res = executeSingleRun(
                algo_factory, mc.factory, spec, run_file, output_dir, args_.verbose);
            // Each job owns a unique result slot.
            all_results[job.mc_idx][job.spec_idx] = std::move(run_res);
        }
    };

    // -------------------------------------------------------------------------
    // Execute jobs using either the current thread or worker threads.
    // -------------------------------------------------------------------------
    if (args_.num_threads <= 1 || jobs.size() < 2) {
        worker();
    } else {
        const std::size_t thread_count = std::min(args_.num_threads, jobs.size());
        std::vector<std::thread> threads;
        threads.reserve(thread_count);
        for (std::size_t i = 0; i < thread_count; ++i) {
            threads.emplace_back(worker);
        }
        // Wait until every worker finishes.
        for (auto& t : threads) {
            t.join();
        }
    }

    // -------------------------------------------------------------------------
    // Combine all individual runs into one result per MissionControl.
    // -------------------------------------------------------------------------
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

    // Export comparative summary and individual reports.
    ResultExporter::exportComparativeReport(
        output_dir,
        args_.simulation_file.filename().string(),
        getCleanDirectoryName(args_.mission_control_folder),
        manager_results, error_managers);

    for (const auto& mgr : manager_results) {
        ResultExporter::exportPerSoReport(
            output_dir, mgr.manager_so_name,
            args_.simulation_file,
            mgr.individual_runs);
    }

    // -------------------------------------------------------------------------
    // Cleanup.
    //
    // Factory objects are destroyed before their shared libraries are unloaded.
    // -------------------------------------------------------------------------
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

// =============================================================================
// COMPETITIVE MODE
//
// One MissionControl is used to compare multiple Algorithm implementations.
// =============================================================================
bool SimulationEngine::runCompetitive() {
    // Find all Algorithm plugins.
    const auto algo_files = discoverSoFiles(args_.algorithms_folder, "Algorithm");
    if (algo_files.empty()) {
        std::cerr << "Error: No Algorithm*.so files found in: "
                  << args_.algorithms_folder.string() << "\n";
        return false;
    }

    // Create competition output directory FIRST so that any subsequent parse
    // errors can be logged to error_log.txt inside it.
    const std::filesystem::path output_dir =
        createUniqueOutputDir(args_.algorithms_folder, "competition");
    if (output_dir.empty()) {
        return false;
    }

    // Parse all simulation configurations.
    SimulationCompositionData composition;
    try {
        composition = ConfigParser::parseSimulationComposition(args_.simulation_file);
    } catch (const std::exception& ex) {
        const std::string msg = std::string("Error parsing composition file: ") + ex.what();
        std::cerr << msg << "\n";
        ResultExporter::logErrorImmediately(output_dir, msg);
        ResultExporter::exportCompetitiveReport(
            output_dir,
            args_.simulation_file.filename().string(),
            args_.mission_control_file.filename().string(),
            {}, {});
        return false;
    }

    const RawCompositionLayout raw_layout = readRawLayout(args_.simulation_file);
    // -------------------------------------------------------------------------
    // Build all simulation × mission × drone × lidar combinations.
    // -------------------------------------------------------------------------
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
        const std::string msg = "Error: No simulation scenarios generated from composition file.";
        std::cerr << msg << "\n";
        ResultExporter::logErrorImmediately(output_dir, msg);
        ResultExporter::exportCompetitiveReport(
            output_dir,
            args_.simulation_file.filename().string(),
            args_.mission_control_file.filename().string(),
            {}, {});
        return false;
    }

    // -------------------------------------------------------------------------
    // Load the single MissionControl and all Algorithm plugins.
    // -------------------------------------------------------------------------
    std::vector<std::string> error_algorithms;

    // Fixed MissionControl used by all algorithms.
    auto mc_loader = preloadLibrary(args_.mission_control_file, output_dir);
    if (!mc_loader) {
        std::cerr << "Error: Failed to load mission control .so.\n";
        ResultExporter::exportCompetitiveReport(
            output_dir,
            args_.simulation_file.filename().string(),
            args_.mission_control_file.filename().string(),
            {}, {args_.mission_control_file.filename().string()});
        return false;
    }
    auto mc_factory_opt = mc_loader->getMissionControlFactory();
    if (!mc_factory_opt) {
        std::cerr << "Error: MissionControl .so did not register a factory.\n";
        ResultExporter::exportCompetitiveReport(
            output_dir,
            args_.simulation_file.filename().string(),
            args_.mission_control_file.filename().string(),
            {}, {args_.mission_control_file.filename().string()});
        return false;
    }
    common::MissionControlFactory mc_factory = *mc_factory_opt;

    // Keeps every Algorithm library alive together with its factory.
    struct PreloadedAlgo {
        std::string so_name;
        std::unique_ptr<DlLoader> loader;
        common::MappingAlgorithmFactory factory;
    };
    std::vector<PreloadedAlgo> preloaded_algos;
    // Load every Algorithm plugin.
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

    
    // -------------------------------------------------------------------------
    // Build job table:
    //
    // Algorithm × run specification
    // -------------------------------------------------------------------------

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

    // Atomic counter distributes jobs safely between threads.
    std::atomic<std::size_t> next_job_idx{0};

     // Worker executes simulation jobs until all jobs are claimed.
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

    // Execute all jobs.
    if (args_.num_threads <= 1 || jobs.size() < 2) {
        worker();
    } else {
        const std::size_t thread_count = std::min(args_.num_threads, jobs.size());
        std::vector<std::thread> threads;
        threads.reserve(thread_count);
        for (std::size_t i = 0; i < thread_count; ++i) {
            threads.emplace_back(worker);
        }
        // Wait for all simulation workers to finish.
        for (auto& t : threads) {
            t.join();
        }
    }

    
    // -------------------------------------------------------------------------
    // Aggregate individual runs into one result per Algorithm.
    // -------------------------------------------------------------------------
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
    // Export competition reports.
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

    // -------------------------------------------------------------------------
    // Cleanup factories before unloading their shared libraries.
    // -------------------------------------------------------------------------
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
