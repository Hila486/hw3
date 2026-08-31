#pragma once

#include <Common/IGPS.h>

namespace user_common_207610130_215664087 {

/**
 * @class MockGPS
 * @brief Simulated GPS sensor implementation.
 * 
 * Provides mock 3D spatial position and orientation heading tracking
 * for the drone simulation, implementing common::IGPS.
 */
class MockGPS final : public common::IGPS {
public:
    /**
     * @brief Constructs a MockGPS sensor with initial position, heading, and resolution.
     * @param position Initial 3D position of the drone.
     * @param heading Initial orientation/heading of the drone.
     * @param resolution Spatial resolution of the positioning unit.
     */
    MockGPS(Position3D position, Orientation heading, PhysicalLength resolution);

    /**
     * @brief Returns the current 3D position reported by the GPS.
     */
    [[nodiscard]] Position3D position() const override;

    /**
     * @brief Returns the current orientation heading reported by the GPS.
     */
    [[nodiscard]] Orientation heading() const override;

    /**
     * @brief Updates the simulated drone position (used by movement hardware Mocks).
     * @param position New 3D position.
     */
    void setPosition(Position3D position);

    /**
     * @brief Updates the simulated drone heading (used by movement hardware Mocks).
     * @param heading New orientation heading.
     */
    void setHeading(Orientation heading);

private:
    Position3D position_{};        ///< Current simulated 3D position
    Orientation heading_{};        ///< Current simulated heading/orientation
    PhysicalLength resolution_{};   ///< Sensor spatial resolution
};

} // namespace user_common_207610130_215664087
