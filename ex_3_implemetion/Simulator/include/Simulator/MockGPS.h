#pragma once

#include <Common/IGPS.h>
#include <Common/Types.h>

namespace simulator_207610130_215664087 {

using namespace common;

class MockGPS final : public common::IGPS {
public:
    MockGPS(Position3D initial_position,
            Orientation initial_heading,
            PhysicalLength resolution);

    [[nodiscard]] Position3D position() const override;
    [[nodiscard]] Orientation heading() const override;

    void setPosition(Position3D position);
    void setHeading(Orientation heading);

private:
    [[nodiscard]] Position3D quantize(const Position3D& position) const;

    Position3D actual_position_;
    Orientation actual_heading_;
    PhysicalLength resolution_;
};

} // namespace simulator_207610130_215664087
