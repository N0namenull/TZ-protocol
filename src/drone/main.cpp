#include "common/udp_socket.h"
#include "common/console.h"
#include "common/options.h"
#include "drone/server.h"
#include <iostream>

int main(int argc, char* argv[]) {
    try {
        drone::Options options = drone::parse_options(argc, argv, false);
        if (options.help) {
            std::cout << drone::options_help(false);
            return 0;
        }
        drone::WinsockRuntime winsock;
        drone::ShutdownSignal shutdown;
        return drone::run_drone(options);
    } catch (const std::exception& error) {
        std::cerr << "DroneSim error: " << error.what() << '\n';
        return 1;
    }
}
