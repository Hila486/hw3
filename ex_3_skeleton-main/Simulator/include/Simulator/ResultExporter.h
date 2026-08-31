#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace simulator_207610130_215664087 {

/**
 * @struct ComparativeManagerResult
 * @brief Result aggregated for a single MissionControl .so implementation in Comparative mode.
 */
struct ComparativeManagerResult {
    std::string manager_so_name;
    double total_score = 0.0;
    std::size_t total_steps = 0;
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
     * @brief Generates formatted UTC timestamp string (e.g. 2026-05-30T23:31:10Z).
     */
    static std::string getCurrentUtcTimestamp();

    /**
     * @brief Generates folder timestamp suffix string (e.g. 20260530_233110).
     */
    static std::string getFolderTimestamp();
};

} // namespace simulator_207610130_215664087
