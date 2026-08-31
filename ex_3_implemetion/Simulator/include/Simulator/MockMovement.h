#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>
#include <Common/Types.h>
#include <Simulator/MockGPS.h>

namespace simulator_207610130_215664087 {

using namespace common;

/**
 * @class MockMovement
 * @brief Hardware mock implementation of IDroneMovement for simulation physics.
 */
class MockMovement final : public common::IDroneMovement {
public:
    explicit MockMovement(MockGPS& gps,
                          const common::IMap3D* hidden_map = nullptr,
                          PhysicalLength drone_radius = 0.0 * x_extent[cm]);

    [[nodiscard]] common::types::MovementResult rotate(
        common::types::RotationDirection direction,
        HorizontalAngle angle) override;

    [[nodiscard]] common::types::MovementResult advance(PhysicalLength distance) override;

    [[nodiscard]] common::types::MovementResult elevate(PhysicalLength distance) override;

private:
    void checkRealMapCollision(const Position3D& position) const;

    MockGPS& gps_;
    const common::IMap3D* hidden_map_ = nullptr;
    PhysicalLength drone_radius_{0.0 * x_extent[cm]};
};

} // namespace simulator_207610130_215664087
