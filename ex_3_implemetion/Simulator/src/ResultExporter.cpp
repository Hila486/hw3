#include <Simulator/ResultExporter.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>

namespace simulator_207610130_215664087 {

namespace {

std::mutex g_error_log_mutex;

struct Summary {
    std::size_t total_runs = 0;
    std::size_t scored_runs = 0;
    std::size_t error_runs = 0;
    double average_score = 0.0;
    double min_score = 0.0;
    double max_score = 0.0;
};

Summary calculateSummary(const std::vector<SingleRunResult>& runs) {
    Summary summary;
    summary.total_runs = runs.size();

    double sum = 0.0;
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();

    for (const auto& run : runs) {
        if (run.score < 0.0 || run.status == "error" || run.status == "Error") {
            ++summary.error_runs;
            continue;
        }
        ++summary.scored_runs;
        sum += run.score;
        min_score = std::min(min_score, run.score);
        max_score = std::max(max_score, run.score);
    }

    if (summary.scored_runs > 0) {
        summary.average_score = sum / static_cast<double>(summary.scored_runs);
        summary.min_score = min_score;
        summary.max_score = max_score;
    } else {
        summary.min_score = 0.0;
        summary.max_score = 0.0;
    }

    return summary;
}

/// Signature representing the exact run-by-run outcome for grouping agreeing managers.
struct ManagerRunSignature {
    double total_score = 0.0;
    std::size_t total_steps = 0;
    std::vector<std::tuple<std::string, std::size_t, double>> run_outcomes;

    bool operator<(const ManagerRunSignature& other) const {
        if (std::abs(total_score - other.total_score) > 1.0e-6) {
            return total_score > other.total_score;
        }
        if (total_steps != other.total_steps) {
            return total_steps < other.total_steps;
        }
        return run_outcomes < other.run_outcomes;
    }
};

} // namespace

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

    // Group managers by matching run-by-run results signature
    std::map<ManagerRunSignature, std::vector<std::string>> groups_map;
    for (const auto& res : manager_results) {
        ManagerRunSignature sig;
        sig.total_score = res.total_score;
        sig.total_steps = res.total_steps;
        for (const auto& run : res.individual_runs) {
            sig.run_outcomes.emplace_back(run.status, run.steps, run.score);
        }
        groups_map[sig].push_back(res.manager_so_name);
    }

    std::vector<ComparativeGroupResult> groups;
    for (const auto& [sig, managers] : groups_map) {
        groups.push_back(ComparativeGroupResult{managers, sig.total_score, sig.total_steps});
    }

    // Sort by number of agreeing managers descending, then by score descending, then by steps ascending
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

    out << std::fixed << std::setprecision(2);
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
        out << "      total_score: " << group.total_score << "\n";
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

    out << std::fixed << std::setprecision(2);
    out << "competitive_report:\n";
    out << "  composition_file: \"" << composition_filename << "\"\n";
    out << "  mission_control: \"" << mc_so_name << "\"\n";
    out << "  generated_at_utc: \"" << getCurrentUtcTimestamp() << "\"\n";
    out << "  results_summary:\n";

    for (const auto& res : sorted_results) {
        out << "    - algorithm: \"" << res.algorithm_so_name << "\"\n";
        out << "      total_score: " << res.total_score << "\n";
        out << "      total_steps: " << res.total_steps << "\n";
    }

    out << "  errors: [";
    for (std::size_t i = 0; i < error_algorithms.size(); ++i) {
        if (i > 0) out << ", ";
        out << "\"" << error_algorithms[i] << "\"";
    }
    out << "]\n";
}

void ResultExporter::exportPerSoReport(
    const std::filesystem::path& output_dir,
    const std::string& so_name,
    const std::filesystem::path& composition_file_path,
    const std::vector<SingleRunResult>& individual_runs) {

    const Summary summary = calculateSummary(individual_runs);
    const std::filesystem::path report_path = output_dir / (so_name + "_simulation_report.yaml");
    std::ofstream out(report_path);
    if (!out) {
        std::cerr << "Error: Failed to create per-SO report file at " << report_path.string() << std::endl;
        return;
    }

    out << std::fixed << std::setprecision(2);
    out << "score_report:\n";
    out << "  composition_file: \"" << composition_file_path.filename().string() << "\"\n";
    out << "  generated_at_utc: \"" << getCurrentUtcTimestamp() << "\"\n";
    out << "  metric: \"output_map_accuracy\"\n";
    out << "  score_range:\n";
    out << "    min: 0\n";
    out << "    max: 100\n";
    out << "  error_score: -1\n";
    out << "  summary:\n";
    out << "    total_runs: " << summary.total_runs << "\n";
    out << "    scored_runs: " << summary.scored_runs << "\n";
    out << "    error_runs: " << summary.error_runs << "\n";
    out << "    average_score: " << summary.average_score << "\n";
    out << "    min_score: " << summary.min_score << "\n";
    out << "    max_score: " << summary.max_score << "\n";
    out << "  simulations:\n";

    // Group runs by simulation_config -> mission_config
    std::map<std::string, std::map<std::string, std::vector<SingleRunResult>>> sim_groups;
    for (const auto& run : individual_runs) {
        sim_groups[run.simulation_config_name][run.mission_config_name].push_back(run);
    }

    for (const auto& [sim_name, mission_map] : sim_groups) {
        out << "    - simulation_config: \"" << sim_name << "\"\n";
        out << "      missions:\n";

        for (const auto& [mission_name, runs] : mission_map) {
            double res_cm = runs.empty() ? 10.0 : runs.front().resolution_cm;
            std::string res_status = runs.empty() ? "ACCEPTED" : runs.front().resolution_request_status;

            out << "        - mission_config: \"" << mission_name << "\"\n";
            out << "          resolution_cm: " << res_cm << "\n";
            out << "          resolution_request_status: \"" << res_status << "\"\n";
            out << "          runs:\n";

            for (const auto& run : runs) {
                out << "            - drone_config: \"" << run.drone_config_name << "\"\n";
                out << "              lidar_config: \"" << run.lidar_config_name << "\"\n";
                out << "              status: \"" << run.status << "\"\n";
                out << "              steps: " << run.steps << "\n";
                out << "              score: " << run.score << "\n";
                out << "              output_map: \"" << run.output_map_file.filename().string() << "\"\n";
                if (!run.error_code.empty()) {
                    out << "              error_ref:\n";
                    out << "                code: \"" << run.error_code << "\"\n";
                }
            }
        }
    }
}

void ResultExporter::logErrorImmediately(
    const std::filesystem::path& output_dir,
    const std::string& message) {

    std::lock_guard<std::mutex> lock(g_error_log_mutex);
    try {
        const std::filesystem::path log_path = output_dir / "error_log.txt";
        std::ofstream out(log_path, std::ios::app);
        if (out) {
            out << message << std::endl;
        }
    } catch (...) {
        // Silently handle filesystem log errors
    }
}

} // namespace simulator_207610130_215664087
