#pragma once

#include <Common/IGPS.h>
#include <Common/Types.h>

namespace simulator_207610130_215664087 {

using namespace common;
// Mock implementation of the GPS interface used by the simulator.
// Stores the drone's simulated position and heading.
class MockGPS final : public common::IGPS {
public:
// Creates the GPS with an initial position, heading and measurement resolution.
    MockGPS(Position3D initial_position,
            Orientation initial_heading,
            PhysicalLength resolution);

    // Returns the drone's current position as reported by the GPS.
    [[nodiscard]] Position3D position() const override;
    // Returns the drone's current heading.
    [[nodiscard]] Orientation heading() const override;

    // Updates the drone's simulated position.
    void setPosition(Position3D position);
    // Updates the drone's simulated heading.
    void setHeading(Orientation heading);

private:
    // Adjusts a position to the GPS resolution,
    // simulating the limited precision of the sensor.
    [[nodiscard]] Position3D quantize(const Position3D& position) const;

    // The drone's exact simulated position.
    Position3D actual_position_;
    // The drone's current simulated orientation.
    Orientation actual_heading_;
    // Resolution used when reporting the GPS position.
    PhysicalLength resolution_;
};

} // namespace simulator_207610130_215664087
