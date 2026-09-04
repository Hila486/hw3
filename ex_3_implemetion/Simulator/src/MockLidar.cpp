#include <Simulator/MockLidar.h>

#include <UserCommon/GeometryUtils.h>
#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <limits>

namespace simulator_207610130_215664087 {

using namespace user_common_207610130_215664087;

namespace {

// Returns the number of lidar beams on a specific FOV circle.
// Each circle has 4 times more beams than the previous one.
[[nodiscard]] std::size_t beams_on_circle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t i = 0; i < circle_index; ++i) {
        count *= 4;
    }
    return count;
}
// Converts a horizontal physical offset at a given distance
// into the corresponding horizontal angle.
[[nodiscard]] HorizontalAngle horizontal_delta(PhysicalLength offset, PhysicalLength distance) {
    return HorizontalAngle{si::atan2(offset, distance)};
}

// Converts a vertical physical offset at a given distance
// into the corresponding altitude angle.
[[nodiscard]] AltitudeAngle altitude_delta(PhysicalLength offset, PhysicalLength distance) {
    return AltitudeAngle{si::atan2(offset, distance)};
}

} // namespace

// initializes the simulated lidar with its configuration,
// the hidden real map, and the GPS.
MockLidar::MockLidar(
    common::types::LidarConfigData lidar_config,
    const common::IMap3D& hidden_map,
    const MockGPS& gps)
    : lidar_config_(lidar_config),
      hidden_map_(hidden_map),
      gps_(gps) {}


// Returns the lidar configuration.
common::types::LidarConfigData MockLidar::config() const {
    return lidar_config_;
}

// Performs a complete lidar scan using the configured FOV circles and beams.
common::types::LidarScanResult MockLidar::scan(Orientation scan_orientation) const {
    common::types::LidarScanResult results;
    // No circles means there are no beams to scan.
    if (lidar_config_.fov_circles == 0) {
        return results;
    }

    // Get the drone's current heading.
    const Orientation sensor_heading = gps_.heading();

    // Convert the requested relative scan direction
    // into an absolute direction in the world.
    const Orientation center_beam_abs =
        absoluteBeamOrientation(sensor_heading, scan_orientation);

    // Trace the central beam (circle 0).
    const PhysicalLength center_distance = traceBeam(center_beam_abs);
    results.push_back(common::types::LidarHit{center_distance, scan_orientation});

    // Generate and trace the beams on all outer FOV circles.
    for (std::size_t circle = 1; circle < lidar_config_.fov_circles; ++circle) {
        const std::size_t beam_count = beams_on_circle(circle);
        // Radius of this beam circle at Z-min distance.
        const PhysicalLength radius = static_cast<double>(circle) * lidar_config_.d;

        for (std::size_t i = 0; i < beam_count; ++i) {
            // Spread the beams evenly around the circle.
            const auto theta = (360.0 * static_cast<double>(i) / static_cast<double>(beam_count)) * deg;
            // Calculate the horizontal and vertical offsets
            // of this beam on the FOV circle.
            const PhysicalLength horizontal_offset = radius * si::cos(theta);
            const PhysicalLength altitude_offset = radius * si::sin(theta);

             // Convert the physical offsets into angular offsets.
            const Orientation offset{
                horizontal_delta(horizontal_offset, lidar_config_.z_min),
                altitude_delta(altitude_offset, lidar_config_.z_min),
            };
            // Direction of this beam relative to the drone.
            const Orientation relative_beam{
                scan_orientation.horizontal + offset.horizontal,
                scan_orientation.altitude + offset.altitude,
            };
            // Convert it into an absolute world direction.
            const Orientation absolute_beam =
                absoluteBeamOrientation(sensor_heading, relative_beam);
            // Trace the beam until it hits an obstacle
            // or reaches the maximum lidar range.
            const PhysicalLength distance = traceBeam(absolute_beam);
            results.push_back(common::types::LidarHit{distance, relative_beam});
        }
    }

    return results;
}

// Traces one lidar beam through the hidden real map.
// Returns the distance to the first occupied voxel.
PhysicalLength MockLidar::traceBeam(const Orientation& beam_orientation) const {
    // Every beam starts from the drone's current position.
    const Position3D origin = gps_.position();
    // Move along the beam in small steps based on map resolution.
    const PhysicalLength step = 0.1 * hidden_map_.getMapConfig().resolution;
    if (step <= 0.0 * cm) {
        return std::numeric_limits<double>::max() * cm;
    }

    for (PhysicalLength distance = 0.0 * cm; distance <= lidar_config_.z_max; distance += step) {
        // Calculate the point at this distance along the beam.
        const Position3D sample = pointAlongBeam(origin, beam_orientation, distance);
        // Stop at the first occupied voxel.
        if (hidden_map_.atVoxel(sample) == common::types::VoxelOccupancy::Occupied) {
            // Objects closer than Z-min are detected,
            // but their exact distance cannot be measured.
            if (distance < lidar_config_.z_min) {
                return 0.0 * cm;
            }
            return distance;
        }
    }
    // Special very large value means that no obstacle
    // was detected before reaching Z-max.
    return std::numeric_limits<double>::max() * cm;
}

} // namespace simulator_207610130_215664087
