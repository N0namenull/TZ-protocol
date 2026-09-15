#include "common/options.h"
#include "gcs/commands.h"
#include <stdexcept>

namespace drone {

Options parse_options(int argc, char* argv[], bool ground_station) {
    Options options;
    if (ground_station) {
        options.listen_port = 14551;
        options.peer_port = 14550;
    }
    for (int index = 1; index < argc; ++index) {
        std::string name = argv[index];
        if (name == "--help") {
            options.help = true;
            continue;
        }
        if (index + 1 >= argc) {
            throw std::invalid_argument("Missing value for " + name);
        }
        std::string value = argv[++index];
        if (name == "--bind") {
            options.bind_ip = value;
        } else if (name == "--peer") {
            options.peer_ip = value;
        } else if (name == "--listen-port" || name == "--peer-port") {
            int port = 0;
            if (!parse_integer(value, port) || port < 1 || port > 65535) {
                throw std::invalid_argument(name + " must be in 1..65535");
            }
            if (name == "--listen-port") {
                options.listen_port = static_cast<std::uint16_t>(port);
            } else {
                options.peer_port = static_cast<std::uint16_t>(port);
            }
        } else if (name == "--log" && ground_station) {
            options.log_path = value;
        } else {
            double number = 0;
            if (!parse_real(value, number)) {
                throw std::invalid_argument("Expected a finite number for " + name);
            }
            if (name == "--duration" && number >= 0 && number <= 86400) {
                options.duration = number;
            } else if (!ground_station && name == "--speed" && number > 0 && number <= 1000) {
                options.speed = number;
            } else if (!ground_station && name == "--vertical-speed" && number > 0 && number <= 1000) {
                options.vertical_speed = number;
            } else if (!ground_station && name == "--start-lat") {
                options.start.latitude = number;
            } else if (!ground_station && name == "--start-lon") {
                options.start.longitude = number;
            } else if (!ground_station && name == "--start-alt" && number >= 0 && number <= 10000) {
                options.start.altitude = static_cast<float>(number);
            } else {
                throw std::invalid_argument("Unknown option or out-of-range value: " + name);
            }
        }
    }
    if (!valid_position(options.start)) {
        throw std::invalid_argument("Invalid starting position");
    }
    if (options.listen_port == options.peer_port && options.bind_ip == options.peer_ip) {
        throw std::invalid_argument("Local and peer endpoints must differ");
    }
    return options;
}

std::string options_help(bool ground_station) {
    std::string text = "--bind <IPv4> --peer <IPv4> --listen-port <1..65535> --peer-port <1..65535>\n"
                       "--duration <seconds, 0=unlimited, max 86400> --help\n";
    if (ground_station) {
        text += "--log <CSV path> (default telemetry.csv; file is replaced on startup)\n";
    } else {
        text += "--speed <m/s> --vertical-speed <m/s> (both >0 and <=1000)\n"
                "--start-lat <-90..90> --start-lon <-180..180> --start-alt <0..10000>\n";
    }
    return text;
}
}
