#include <Simulator/ResultExporter.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace simulator_207610130_215664087 {

std::string ResultExporter::getCurrentUtcTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
#ifdef _WIN32
    gmtime_s(&utc_tm, &now_c);
#else
    gmtime_r(&now_c, &utc_tm);
#endif
    std::ostringstream ss;
    ss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%SZ");
    return ss.str();
}

std::string ResultExporter::getFolderTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
#ifdef _WIN32
    gmtime_s(&utc_tm, &now_c);
#else
    gmtime_r(&now_c, &utc_tm);
#endif
    std::ostringstream ss;
    ss << std::put_time(&utc_tm, "%Y%m%d_%H%M%S_") << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

void ResultExporter::exportComparativeReport(
    const std::filesystem::path& output_dir,
    const std::string& composition_filename,
    const std::string& mc_folder_name,
    const std::vector<ComparativeManagerResult>& manager_results,
    const std::vector<std::string>& error_managers) {

    // Group managers by matching (total_score, total_steps)
    // Key: pair<double score, size_t steps>
    struct ResultKey {
        double score;
        std::size_t steps;
        bool operator<(const ResultKey& other) const {
            if (std::abs(score - other.score) > 1.0e-6) {
                return score > other.score;
            }
            return steps < other.steps;
        }
    };

    std::map<ResultKey, std::vector<std::string>> groups_map;
    for (const auto& res : manager_results) {
        groups_map[{res.total_score, res.total_steps}].push_back(res.manager_so_name);
    }

    std::vector<ComparativeGroupResult> groups;
    for (const auto& [key, managers] : groups_map) {
        groups.push_back(ComparativeGroupResult{managers, key.score, key.steps});
    }

    // Sort by number of agreeing managers descending
    std::sort(groups.begin(), groups.end(), [](const ComparativeGroupResult& a, const ComparativeGroupResult& b) {
        if (a.agreeing_managers.size() != b.agreeing_managers.size()) {
            return a.agreeing_managers.size() > b.agreeing_managers.size();
        }
        if (std::abs(a.total_score - b.total_score) > 1.0e-6) {
            return a.total_score > b.total_score;
        }
        return a.total_steps < b.total_steps;
    });

    const std::filesystem::path report_path = output_dir / "comparative_simulation_report.yaml";
    std::ofstream out(report_path);
    if (!out) {
        std::cerr << "Error: Failed to create comparative report file at " << report_path.string() << std::endl;
        return;
    }

    out << "comparative_report:\n";
    out << "  composition_file: \"" << composition_filename << "\"\n";
    out << "  mission_control_folder: \"" << mc_folder_name << "\"\n";
    out << "  generated_at_utc: \"" << getCurrentUtcTimestamp() << "\"\n";
    out << "  results_summary:\n";

    for (const auto& group : groups) {
        out << "    - same_results: [";
        for (std::size_t i = 0; i < group.agreeing_managers.size(); ++i) {
            if (i > 0) out << ", ";
            out << "\"" << group.agreeing_managers[i] << "\"";
        }
        out << "]\n";
        out << "      total_score: " << std::llround(group.total_score) << "\n";
        out << "      total_steps: " << group.total_steps << "\n";
    }

    out << "  errors: [";
    for (std::size_t i = 0; i < error_managers.size(); ++i) {
        if (i > 0) out << ", ";
        out << "\"" << error_managers[i] << "\"";
    }
    out << "]\n";
}

void ResultExporter::exportCompetitiveReport(
    const std::filesystem::path& output_dir,
    const std::string& composition_filename,
    const std::string& mc_so_name,
    const std::vector<CompetitiveAlgoResult>& algo_results,
    const std::vector<std::string>& error_algorithms) {

    std::vector<CompetitiveAlgoResult> sorted_results = algo_results;
    // Sort by score descending, then by steps ascending
    std::sort(sorted_results.begin(), sorted_results.end(), [](const CompetitiveAlgoResult& a, const CompetitiveAlgoResult& b) {
        if (std::abs(a.total_score - b.total_score) > 1.0e-6) {
            return a.total_score > b.total_score;
        }
        return a.total_steps < b.total_steps;
    });

    const std::filesystem::path report_path = output_dir / "competitive_simulation_report.yaml";
    std::ofstream out(report_path);
    if (!out) {
        std::cerr << "Error: Failed to create competitive report file at " << report_path.string() << std::endl;
        return;
    }

    out << "competitive_report:\n";
    out << "  composition_file: \"" << composition_filename << "\"\n";
    out << "  mission_control: \"" << mc_so_name << "\"\n";
    out << "  generated_at_utc: \"" << getCurrentUtcTimestamp() << "\"\n";
    out << "  results_summary:\n";

    for (const auto& res : sorted_results) {
        out << "    - algorithm: \"" << res.algorithm_so_name << "\"\n";
        out << "      total_score: " << std::llround(res.total_score) << "\n";
        out << "      total_steps: " << res.total_steps << "\n";
    }

    out << "  errors: [";
    for (std::size_t i = 0; i < error_algorithms.size(); ++i) {
        if (i > 0) out << ", ";
        out << "\"" << error_algorithms[i] << "\"";
    }
    out << "]\n";
}

} // namespace simulator_207610130_215664087
