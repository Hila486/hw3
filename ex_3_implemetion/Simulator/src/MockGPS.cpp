#include <Simulator/MockGPS.h>

#include <cmath>

namespace simulator_207610130_215664087 {

MockGPS::MockGPS(
    Position3D initial_position,
    Orientation initial_heading,
    PhysicalLength resolution)
    : actual_position_(initial_position),
      actual_heading_(initial_heading),
      resolution_(resolution) {}

Position3D MockGPS::quantize(const Position3D& position) const {
    const double res_cm = resolution_.force_numerical_value_in(cm);
    if (res_cm <= 0.0) {
        return position;
    }

    const double qx = std::round(position.x.force_numerical_value_in(cm) / res_cm) * res_cm;
    const double qy = std::round(position.y.force_numerical_value_in(cm) / res_cm) * res_cm;
    const double qz = std::round(position.z.force_numerical_value_in(cm) / res_cm) * res_cm;

    return Position3D{
        qx * x_extent[cm],
        qy * y_extent[cm],
        qz * z_extent[cm]
    };
}

Position3D MockGPS::position() const {
    return quantize(actual_position_);
}

Orientation MockGPS::heading() const {
    return actual_heading_;
}

void MockGPS::setPosition(Position3D position) {
    actual_position_ = position;
}

void MockGPS::setHeading(Orientation heading) {
    actual_heading_ = heading;
}

} // namespace simulator_207610130_215664087
