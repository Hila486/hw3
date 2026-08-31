#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>
#include <UserCommon/MockGPS.h>

namespace user_common_207610130_215664087 {

/**
 * @class MockMovement
 * @brief Simulated drone physical movement execution.
 * 
 * Implements common::IDroneMovement to execute rotation, linear advancement,
 * and elevation changes, updating the underlying MockGPS sensor position and heading.
 * Optionally holds reference to hidden map for mandatory real-map collision detection.
 */
class MockMovement final : public common::IDroneMovement {
public:
    /**
     * @brief Constructs a MockMovement instance linked to a MockGPS sensor.
     * @param gps Reference to the simulated GPS sensor to update during movement.
     * @param hidden_map Optional pointer to ground-truth hidden map for collision detection.
     */
    explicit MockMovement(MockGPS& gps, const common::IMap3D* hidden_map = nullptr);

    /**
     * @brief Rotates the drone relative to its current heading.
     * @param direction Rotation direction (Left or Right).
     * @param angle Angular degree of rotation.
     * @return MovementResult indicating success or failure with error details.
     */
    common::types::MovementResult rotate(common::types::RotationDirection direction, HorizontalAngle angle) override;

    /**
     * @brief Advances the drone forward in its current heading direction.
     * @param distance Distance to travel forward.
     * @return MovementResult indicating success or collision/error details.
     */
    common::types::MovementResult advance(PhysicalLength distance) override;

    /**
     * @brief Elevates or lowers the drone altitude vertically.
     * @param distance Vertical distance change (positive = climb, negative = descend).
     * @return MovementResult indicating success or boundary/collision details.
     */
    common::types::MovementResult elevate(PhysicalLength distance) override;

private:
    void checkRealMapCollision(const Position3D& position) const;

    MockGPS& gps_;                          ///< Reference to the mock GPS sensor updated during movement
    const common::IMap3D* hidden_map_ = nullptr; ///< Optional ground-truth map for real-map collision checks
};

} // namespace user_common_207610130_215664087
