#pragma once
#include "common/protocol.h"
#include <string>

namespace drone {
enum class Action { Empty, Status, Help, Quit, Send, Invalid };
struct UserCommand {
    Action action = Action::Invalid;
    Packet packet;
    std::string error;
};
UserCommand parse_command(const std::string& line);
bool parse_real(const std::string& text, double& value);
bool parse_integer(const std::string& text, int& value);
}
