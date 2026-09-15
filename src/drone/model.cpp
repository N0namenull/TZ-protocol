#include "drone/model.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double earth_radius = 6371000.0;
constexpr double meters_per_degree = earth_radius * pi / 180.0;
constexpr float critical_battery = 5.0F;

double longitude_difference(double target, double current) {
    return std::remainder(target - current, 360.0);
}
} // namespace

Model::Model(Position start, double horizontal, double vertical, double drain)
    : home_(start), horizontal_speed_(horizontal), vertical_speed_(vertical), battery_drain_(drain) {
    if (!valid_position(start) || !std::isfinite(horizontal) || horizontal <= 0 ||
        !std::isfinite(vertical) || vertical <= 0 || !std::isfinite(drain) || drain < 0) {
        throw std::invalid_argument("Invalid model configuration");
    }
    state_.position = start;
}

Reply Model::handle(const Packet& command) {
    Reply reply{command.message_id, command.sequence, false, Reason::Other};
    switch (command.message_id) {
    case MessageId::Arm:
    case MessageId::Disarm:
        if (!command.payload.empty()) {
            reply.reason = Reason::InvalidArgs;
            return reply;
        }
        if (status() == SystemStatus::Critical) {
            return reply;
        }
        if (command.message_id == MessageId::Arm) {
            if (state_.armed) {
                reply.reason = Reason::Busy;
                return reply;
            }
            state_.armed = true;
            if (state_.mode == Mode::Rtl) {
                target_ = home_;
            }
        } else {
            state_.armed = false;
            target_.reset();
        }
        break;
    case MessageId::SetMode:
        if (command.payload.size() != 1 || !valid_mode(command.payload[0])) {
            reply.reason = Reason::InvalidArgs;
            return reply;
        }
        state_.mode = static_cast<Mode>(command.payload[0]);
        target_.reset();
        if (state_.mode == Mode::Rtl && state_.armed && status() != SystemStatus::Critical) {
            target_ = home_;
        }
        break;
    case MessageId::GoTo: {
        Position target;
        if (!decode_position(command.payload, target)) {
            reply.reason = Reason::InvalidArgs;
            return reply;
        }
        if (!state_.armed) {
            reply.reason = Reason::NotArmed;
            return reply;
        }
        if (state_.mode != Mode::Guided) {
            reply.reason = Reason::BadMode;
            return reply;
        }
        if (status() == SystemStatus::Critical) {
            return reply;
        }
        target_ = target;
        break;
    }
    default:
        return reply;
    }
    reply.accepted = true;
    return reply;
}

void Model::update(double seconds) {
    if (!std::isfinite(seconds) || seconds <= 0 || !state_.armed) {
        return;
    }
    battery_ = std::max(0.0, battery_ - battery_drain_ * seconds);
    state_.battery = static_cast<float>(battery_);
    if (status() == SystemStatus::Critical) {
        target_.reset();
        return;
    }
    if (!target_) {
        return;
    }

    Position& current = state_.position;
    const Position& target = *target_;
    double latitude_delta = target.latitude - current.latitude;
    double longitude_delta = longitude_difference(target.longitude, current.longitude);
    double mean_latitude = (current.latitude + target.latitude) * 0.5 * pi / 180.0;
    double north = latitude_delta * meters_per_degree;
    double east = longitude_delta * meters_per_degree * std::cos(mean_latitude);
    double horizontal_distance = std::hypot(north, east);
    double step = horizontal_speed_ * seconds;
    bool horizontal_arrived = horizontal_distance <= step;

    if (horizontal_distance > 0.000001) {
        double yaw = std::atan2(east, north) * 180.0 / pi;
        if (yaw < 0) {
            yaw += 360.0;
        }
        state_.yaw = static_cast<float>(yaw);
        if (state_.yaw >= 360.0F) {
            state_.yaw = 0.0F;
        }
    }
    if (horizontal_arrived) {
        current.latitude = target.latitude;
        current.longitude = target.longitude;
    } else {
        double fraction = step / horizontal_distance;
        current.latitude += latitude_delta * fraction;
        current.longitude = std::remainder(current.longitude + longitude_delta * fraction, 360.0);
    }

    double altitude_delta = static_cast<double>(target.altitude) - current.altitude;
    double vertical_step = vertical_speed_ * seconds;
    bool vertical_arrived = std::abs(altitude_delta) <= vertical_step;
    if (vertical_arrived) {
        current.altitude = target.altitude;
    } else {
        current.altitude += static_cast<float>(std::copysign(vertical_step, altitude_delta));
    }
    if (horizontal_arrived && vertical_arrived) {
        target_.reset();
    }
}

const Telemetry& Model::telemetry() const { return state_; }

SystemStatus Model::status() const {
    if (state_.battery <= critical_battery) {
        return SystemStatus::Critical;
    }
    if (state_.armed) {
        return SystemStatus::Active;
    }
    return SystemStatus::Standby;
}

bool Model::has_target() const { return target_.has_value(); }

} // namespace drone
