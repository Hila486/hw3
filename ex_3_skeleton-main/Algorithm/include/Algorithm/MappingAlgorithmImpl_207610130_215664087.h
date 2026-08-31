#pragma once

#include <Common/IMappingAlgorithm.h>

#include <cstddef>
#include <optional>
#include <set>
#include <vector>

namespace algorithm_207610130_215664087 {

/**
 * @class MappingAlgorithmImpl_207610130_215664087
 * @brief Concrete drone mapping and exploration algorithm implementing common::IMappingAlgorithm.
 * 
 * Uses Depth-First Search (DFS) cell exploration over 3D discrete space to systematically scan 
 * and discover unknown voxel space while respecting drone physical bounds and battery limits.
 */
class MappingAlgorithmImpl_207610130_215664087 final : public common::IMappingAlgorithm {
public:
    /**
     * @brief Constructs mapping algorithm with dependencies (mission config, lidar config, drone config, map).
     * @param dependencies Common mapping algorithm dependencies struct.
     */
    using common::IMappingAlgorithm::IMappingAlgorithm;

    /**
     * @brief Determines the next mapping step command based on current state and scan data.
     * @param state Current position, heading, and step count.
     * @param latest_scan Pointer to latest LiDAR scan hits result.
     * @return MappingStepCommand specifying scan orientation or movement.
     */
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
    /// Discrete 3D grid cell identifier for DFS exploration.
    struct GridCell {
        int x = 0;
        int y = 0;
        int z = 0;

        [[nodiscard]] bool operator<(const GridCell& other) const;
        [[nodiscard]] bool operator==(const GridCell& other) const;
    };

    /// Path node storing cell grid location and 3D world coordinate.
    struct PathNode {
        GridCell cell{};
        Position3D position{};
    };

    /// Target destination for drone movement during exploration or backtracking.
    struct NavigationTarget {
        PathNode node{};
        bool backtracking = false;
        bool vertical = false;
        double desired_heading_degrees = 0.0;
        PhysicalLength distance{};
    };

    [[nodiscard]] GridCell cellFromPosition(const Position3D& position) const;
    [[nodiscard]] PhysicalLength planningStep() const;
    [[nodiscard]] bool isLegalDroneCenter(const Position3D& position) const;
    [[nodiscard]] std::optional<NavigationTarget> findExplorationTarget(
        const common::types::DroneState& state,
        const GridCell& current_cell) const;
    [[nodiscard]] common::types::MappingStepCommand commandForTarget(
        const common::types::DroneState& state,
        const GridCell& current_cell,
        const NavigationTarget& target);
    [[nodiscard]] common::types::MappingStepCommand scanCommand(std::size_t scan_index);
    [[nodiscard]] common::types::MappingStepCommand finishCommand() const;

    void initializeAtCurrentCell(const GridCell& current_cell, const Position3D& position);
    void handlePendingTranslationResult(const GridCell& current_cell);
    void resetScanSequenceForCell(const GridCell& current_cell);

    std::set<GridCell> visited_cells_;              ///< Set of fully explored grid cells
    std::set<GridCell> attempted_cells_;            ///< Set of cells attempted but blocked
    std::vector<PathNode> dfs_path_;                ///< Stack path for DFS backtracking
    std::optional<GridCell> scan_cell_;             ///< Cell currently being scanned
    std::size_t next_scan_index_ = 0;               ///< Multi-directional scan index
    bool waiting_for_scan_result_ = false;          ///< True if waiting for scan return
    std::optional<std::size_t> pending_scan_index_;  ///< Index of scan awaiting result
    std::optional<NavigationTarget> pending_target_; ///< Target being navigated to
    std::optional<NavigationTarget> pending_translation_; ///< Active movement execution
    std::size_t failed_translation_count_ = 0;      ///< Counter for failed movement attempts
};

} // namespace algorithm_207610130_215664087
