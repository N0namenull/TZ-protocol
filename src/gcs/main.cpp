#include "common/udp_socket.h"
#include "common/console.h"
#include "common/options.h"
#include "gcs/client.h"
#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char* argv[]) {
    try {
        drone::Options options = drone::parse_options(argc, argv, true);
        if (options.help) {
            std::cout << drone::options_help(true);
            return 0;
        }
        drone::WinsockRuntime winsock;
        drone::ShutdownSignal shutdown;
        drone::Console console;
        drone::ConsoleInput input(console);
        drone::CommandQueue commands;
        std::atomic<bool> stop{false};
        int worker_result = 0;

        // Only this thread owns the socket, CSV, telemetry and pending requests.
        std::thread worker([&]() {
            try {
                drone::run_gcs(options, commands, console, stop);
            } catch (const std::exception& error) {
                console.line(std::string("GCS error: ") + error.what());
                worker_result = 1;
                stop.store(true);
            }
        });

        try {
            bool input_finished = false;
            while (!stop.load() && !drone::ShutdownSignal::requested()) {
                std::string line;
                drone::InputResult result = input.poll(line);
                if (result == drone::InputResult::End) {
                    input_finished = true;
                    // EOF finishes input but lets already accepted requests drain.
                    commands.close();
                } else if (result == drone::InputResult::Line) {
                    if (!commands.push(line)) {
                        console.line("Input queue is full; line rejected.");
                    }
                }
                if (input_finished) {
                    break;
                }
                if (result == drone::InputResult::Waiting) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
            if (drone::ShutdownSignal::requested()) {
                stop.store(true);
            }
        } catch (...) {
            stop.store(true);
            worker.join();
            throw;
        }
        worker.join();
        return worker_result;
    } catch (const std::exception& error) {
        std::cerr << "GCS error: " << error.what() << '\n';
        return 1;
    }
}
