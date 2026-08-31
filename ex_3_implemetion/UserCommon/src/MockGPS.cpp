#include <UserCommon/MockGPS.h>

namespace user_common_207610130_215664087 {

/**
 * @brief Constructs a MockGPS sensor with given initial position, heading, and resolution.
 */
MockGPS::MockGPS(Position3D position, Orientation heading, PhysicalLength resolution)
    : position_(position), heading_(heading), resolution_(resolution) {}

/**
 * @brief Returns current simulated 3D position.
 */
Position3D MockGPS::position() const {
    return position_;
}

/**
 * @brief Returns current simulated orientation heading.
 */
Orientation MockGPS::heading() const {
    return heading_;
}

/**
 * @brief Updates simulated position.
 */
void MockGPS::setPosition(Position3D position) {
    position_ = position;
}

/**
 * @brief Updates simulated heading.
 */
void MockGPS::setHeading(Orientation heading) {
    heading_ = heading;
}

} // namespace user_common_207610130_215664087
