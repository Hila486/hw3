#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>
#include <Common/Types.h>
#include <Simulator/MockGPS.h>

namespace simulator_207610130_215664087 {

using namespace common;


// Mock implementation of the drone movement hardware.
// Updates the simulated GPS and can use the hidden real map for collision checks.
class MockMovement final : public common::IDroneMovement {
public:
    // Creates the movement mock.
    // gps is updated whenever the drone moves.
    // hidden_map is the real simulator map used only to detect collisions.
    // drone_radius represents the physical size of the drone for collision checking.
    explicit MockMovement(MockGPS& gps,
                          const common::IMap3D* hidden_map = nullptr,
                          PhysicalLength drone_radius = 0.0 * x_extent[cm]);

    // Rotates the drone left or right by the requested angle.
    [[nodiscard]] common::types::MovementResult rotate(
        common::types::RotationDirection direction,
        HorizontalAngle angle) override;

    // Moves the drone forward or backward according to its current heading.
    [[nodiscard]] common::types::MovementResult advance(PhysicalLength distance) override;

    // Moves the drone vertically up or down.
    [[nodiscard]] common::types::MovementResult elevate(PhysicalLength distance) override;

private:
    // Checks whether the drone would collide with an occupied voxel
    // in the hidden real map at the given position.
    void checkRealMapCollision(const Position3D& position) const;

    // GPS mock that stores the drone's simulated position and heading.
    MockGPS& gps_;
    // Real map used internally by the simulator for collision detection.
    // The drone algorithm itself should not have access to this map.
    const common::IMap3D* hidden_map_ = nullptr;
    // Physical radius of the drone used when checking collisions.
    PhysicalLength drone_radius_{0.0 * x_extent[cm]};
};

} // namespace simulator_207610130_215664087
