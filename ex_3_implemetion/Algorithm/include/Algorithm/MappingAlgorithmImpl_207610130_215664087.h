#pragma once

#include <Common/IMappingAlgorithm.h>
#include <Common/Types.h>

#include <cstddef>
#include <optional>
#include <set>
#include <vector>

namespace algorithm_207610130_215664087 {

using namespace common;


// Concrete mapping algorithm.
// Uses DFS over a discrete 3D grid to explore, scan, and backtrack through the map.
class MappingAlgorithmImpl_207610130_215664087 final : public common::IMappingAlgorithm {
public:

    // Reuse the base-class constructor that receives all algorithm dependencies.
    using common::IMappingAlgorithm::IMappingAlgorithm;


    // Decides what the drone should do next based on its current state
    // and the most recent lidar scan.
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
     // Represents one discrete cell in the 3D exploration grid.
    struct GridCell {
        int x = 0;
        int y = 0;
        int z = 0;

        // Needed so GridCell can be stored inside std::set.
        [[nodiscard]] bool operator<(const GridCell& other) const;
        // Checks whether two grid cells are the same.
        [[nodiscard]] bool operator==(const GridCell& other) const;
    };

    // Stores both a grid cell and its corresponding real-world position.
    struct PathNode {
        GridCell cell{};
        Position3D position{};
    };

    // Describes where the drone should navigate next.
    struct NavigationTarget {
        PathNode node{};
        // True when returning along the DFS path.
        bool backtracking = false;
        // True when this movement is vertical.
        bool vertical = false;
        // Heading needed before horizontal movement.
        double desired_heading_degrees = 0.0;
        // Distance to the target.
        PhysicalLength distance{};
    };

    // Converts a real-world position into a discrete DFS grid cell.
    [[nodiscard]] GridCell cellFromPosition(const Position3D& position) const;

    // Returns the distance between neighboring exploration cells.
    [[nodiscard]] PhysicalLength planningStep() const;
    
    // Checks whether the drone's center can legally be placed at this position.
    [[nodiscard]] bool isLegalDroneCenter(const Position3D& position) const;
    
    // Searches for the next DFS exploration target.
    // If no new neighbor is available, it may choose a backtracking target.
    [[nodiscard]] std::optional<NavigationTarget> findExplorationTarget(
        const common::types::DroneState& state,
        const GridCell& current_cell) const;

    // Converts a navigation target into the next required movement command.
    [[nodiscard]] common::types::MappingStepCommand commandForTarget(
        const common::types::DroneState& state,
        const GridCell& current_cell,
        const NavigationTarget& target);

     // Creates one lidar scan command from the cell's scan sequence.
    [[nodiscard]] common::types::MappingStepCommand scanCommand(std::size_t scan_index);

    // Creates the command that tells mission control mapping is finished.
    [[nodiscard]] common::types::MappingStepCommand finishCommand() const;
    
    // Initializes DFS state when the algorithm starts.
    void initializeAtCurrentCell(const GridCell& current_cell, const Position3D& position);
    // Handles the result of a previously requested movement.
    void handlePendingTranslationResult(const GridCell& current_cell);
    // Starts/restarts the scan sequence for a cell.
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
