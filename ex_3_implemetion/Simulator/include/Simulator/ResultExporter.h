#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace simulator_207610130_215664087 {

/**
 * @struct SingleRunResult
 * @brief Result and metadata for one single simulation combination run.
 */
struct SingleRunResult {
    std::size_t run_index = 0;
    std::string simulation_config_name{};
    std::string mission_config_name{};
    std::string drone_config_name{};
    std::string lidar_config_name{};
    double resolution_cm = 0.0;
    std::string resolution_request_status = "ACCEPTED";
    std::filesystem::path output_map_file{};
    std::size_t steps = 0;
    std::string status = "completed";
    double score = 0.0;
    std::string error_code{};
    std::string error_message{};
};

/**
 * @struct ComparativeManagerResult
 * @brief Result aggregated for a single MissionControl .so implementation in Comparative mode.
 */
struct ComparativeManagerResult {
    std::string manager_so_name;
    double total_score = 0.0;
    std::size_t total_steps = 0;
    std::vector<SingleRunResult> individual_runs{};
};

/**
 * @struct ComparativeGroupResult
 * @brief Group of MissionControl implementations that produced matching results.
 */
struct ComparativeGroupResult {
    std::vector<std::string> agreeing_managers;
    double total_score = 0.0;
    std::size_t total_steps = 0;
};

/**
 * @struct CompetitiveAlgoResult
 * @brief Result aggregated for a single Algorithm .so implementation in Competitive mode.
 */
struct CompetitiveAlgoResult {
    std::string algorithm_so_name;
    double total_score = 0.0;
    std::size_t total_steps = 0;
    std::vector<SingleRunResult> individual_runs{};
};

/**
 * @class ResultExporter
 * @brief Formats and writes Comparative and Competitive simulation reports to YAML output files.
 */
class ResultExporter {
public:
    /**
     * @brief Exports Comparative run summary report YAML file.
     */
    static void exportComparativeReport(
        const std::filesystem::path& output_dir,
        const std::string& composition_filename,
        const std::string& mc_folder_name,
        const std::vector<ComparativeManagerResult>& manager_results,
        const std::vector<std::string>& error_managers);

    /**
     * @brief Exports Competitive run summary report YAML file.
     */
    static void exportCompetitiveReport(
        const std::filesystem::path& output_dir,
        const std::string& composition_filename,
        const std::string& mc_so_name,
        const std::vector<CompetitiveAlgoResult>& algo_results,
        const std::vector<std::string>& error_algorithms);

    /**
     * @brief Exports detailed per-SO Assignment-2 style score_report YAML file.
     */
    static void exportPerSoReport(
        const std::filesystem::path& output_dir,
        const std::string& so_name,
        const std::filesystem::path& composition_file_path,
        const std::vector<SingleRunResult>& individual_runs);

    /**
     * @brief Appends an error message to error_log.txt in output_dir thread-safely.
     */
    static void logErrorImmediately(
        const std::filesystem::path& output_dir,
        const std::string& message);

    /**
     * @brief Generates formatted UTC timestamp string (e.g. 2026-05-30T23:31:10Z).
     */
    static std::string getCurrentUtcTimestamp();

    /**
     * @brief Generates folder timestamp suffix string (e.g. 20260530_233110_123).
     */
    static std::string getFolderTimestamp();
};

} // namespace simulator_207610130_215664087
