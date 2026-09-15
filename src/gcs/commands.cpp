#include "gcs/commands.h"
#include <cmath>
#include <limits>
#include <locale>
#include <sstream>
#include <vector>

namespace drone {

bool parse_real(const std::string& text, double& value) {
    std::istringstream input(text);
    input.imbue(std::locale::classic());
    double parsed = 0;
    if (!(input >> parsed) || !input.eof() || !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

bool parse_integer(const std::string& text, int& value) {
    std::istringstream input(text);
    input.imbue(std::locale::classic());
    int parsed = 0;
    if (!(input >> parsed) || !input.eof()) {
        return false;
    }
    value = parsed;
    return true;
}

UserCommand parse_command(const std::string& line) {
    std::istringstream input(line);
    input.imbue(std::locale::classic());
    std::vector<std::string> words;
    std::string word;
    while (input >> word) {
        words.push_back(word);
    }
    UserCommand result;
    result.error = "Use: status | arm | disarm | mode <0|1|2> | goto <lat> <lon> <alt> | help | quit";
    if (words.empty()) {
        result.action = Action::Empty;
        return result;
    }
    const std::string& name = words[0];
    if (words.size() == 1) {
        if (name == "status") {
            result.action = Action::Status;
        } else if (name == "help") {
            result.action = Action::Help;
        } else if (name == "quit") {
            result.action = Action::Quit;
        } else if (name == "arm" || name == "disarm") {
            result.action = Action::Send;
            result.packet.message_id = name == "arm" ? MessageId::Arm : MessageId::Disarm;
        }
        return result;
    }
    if (name == "mode" && words.size() == 2) {
        int mode = 0;
        if (!parse_integer(words[1], mode) || mode < 0 || mode > 2) {
            result.error = "Mode must be 0 (MANUAL), 1 (GUIDED), or 2 (RTL).";
            return result;
        }
        result.action = Action::Send;
        result.packet.message_id = MessageId::SetMode;
        result.packet.payload = {static_cast<std::uint8_t>(mode)};
        return result;
    }
    if (name == "goto" && words.size() == 4) {
        Position position;
        double altitude = 0;
        if (!parse_real(words[1], position.latitude) || !parse_real(words[2], position.longitude) ||
            !parse_real(words[3], altitude) || altitude < 0 || altitude > 10000) {
            result.error = "GOTO requires finite lat, lon, alt; altitude range: 0..10000 m.";
            return result;
        }
        position.altitude = static_cast<float>(altitude);
        if (!valid_position(position)) {
            result.error = "Latitude range: -90..90; longitude: -180..180; altitude: 0..10000 m.";
            return result;
        }
        result.action = Action::Send;
        result.packet.message_id = MessageId::GoTo;
        result.packet.payload = encode_position(position);
    }
    return result;
}

} // namespace drone
