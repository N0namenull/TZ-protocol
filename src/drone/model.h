#pragma once

#include "common/protocol.h"
#include <optional>

namespace drone {

class Model {
public:
    explicit Model(Position start = {}, double horizontal_speed = 10.0,
                   double vertical_speed = 3.0, double battery_drain = 0.1);
    Reply handle(const Packet& command);
    void update(double seconds);
    const Telemetry& telemetry() const;
    SystemStatus status() const;
    bool has_target() const;

private:
    Telemetry state_;
    Position home_;
    std::optional<Position> target_;
    double horizontal_speed_;
    double vertical_speed_;
    double battery_drain_;
    double battery_ = 100.0;
};

} // namespace drone
