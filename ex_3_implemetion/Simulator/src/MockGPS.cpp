#include <Simulator/MockGPS.h>

#include <cmath>

namespace simulator_207610130_215664087 {

// Initializes the simulated GPS with the drone's starting position,
// heading, and GPS resolution.
MockGPS::MockGPS(
    Position3D initial_position,
    Orientation initial_heading,
    PhysicalLength resolution)
    : actual_position_(initial_position),
      actual_heading_(initial_heading),
      resolution_(resolution) {}


// Rounds a position according to the GPS resolution.
// This simulates the limited precision of a real GPS sensor.
Position3D MockGPS::quantize(const Position3D& position) const {
    const double res_cm = resolution_.force_numerical_value_in(cm);
    
    // If the resolution is invalid, return the exact position.
    if (res_cm <= 0.0) {
        return position;
    }

    // Convert each coordinate to centimeters, divide by the resolution,
    // round to the nearest resolution step, and convert back.
    const double qx = std::round(position.x.force_numerical_value_in(cm) / res_cm) * res_cm;
    const double qy = std::round(position.y.force_numerical_value_in(cm) / res_cm) * res_cm;
    const double qz = std::round(position.z.force_numerical_value_in(cm) / res_cm) * res_cm;

    // Convert the rounded numeric values back into strong length types.
    return Position3D{
        qx * x_extent[cm],
        qy * y_extent[cm],
        qz * z_extent[cm]
    };
}

// Returns the drone's current position as seen by the GPS.
// The reported value is rounded according to the GPS resolution.
Position3D MockGPS::position() const {
    return quantize(actual_position_);
}
// Returns the drone's current heading.
Orientation MockGPS::heading() const {
    return actual_heading_;
}
// Updates the drone's exact simulated position.
void MockGPS::setPosition(Position3D position) {
    actual_position_ = position;
}
// Updates the drone's simulated heading.
void MockGPS::setHeading(Orientation heading) {
    actual_heading_ = heading;
}

} // namespace simulator_207610130_215664087
