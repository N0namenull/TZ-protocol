#pragma once
#include "common/protocol.h"
#include <string>

namespace drone {
struct Options {
    std::string bind_ip = "127.0.0.1";
    std::string peer_ip = "127.0.0.1";
    std::uint16_t listen_port = 14550;
    std::uint16_t peer_port = 14551;
    Position start;
    double speed = 10.0;
    double vertical_speed = 3.0;
    double duration = 0;
    std::string log_path = "telemetry.csv";
    bool help = false;
};
Options parse_options(int argc, char* argv[], bool ground_station);
std::string options_help(bool ground_station);
}
