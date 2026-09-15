#pragma once

#include <cstdint>
#include <vector>

namespace drone {

constexpr std::uint32_t protocol_magic = 0x44524F4E;
constexpr std::size_t header_size = 12;
constexpr std::size_t checksum_size = 2;

enum class MessageId : std::uint16_t {
    Heartbeat = 1,
    Telemetry = 2,
    Arm = 10,
    Disarm = 11,
    SetMode = 12,
    GoTo = 13,
    Ack = 20,
    Nack = 21
};

enum class Mode : std::uint8_t { Manual = 0, Guided = 1, Rtl = 2 };
enum class SystemStatus : std::uint8_t { Boot = 0, Standby = 1, Active = 2, Critical = 3 };
enum class Reason : std::uint8_t { NotArmed = 1, BadMode = 2, InvalidArgs = 3, Busy = 4, Other = 5 };

struct Packet {
    MessageId message_id = MessageId::Heartbeat;
    std::uint32_t sequence = 0;
    std::vector<std::uint8_t> payload;
};

struct Position {
    double latitude = 55.75;
    double longitude = 37.61;
    float altitude = 0.0F;
};

struct Telemetry {
    Position position;
    float yaw = 0.0F;
    float battery = 100.0F;
    bool armed = false;
    Mode mode = Mode::Manual;
};

struct Reply {
    MessageId command_id = MessageId::Arm;
    std::uint32_t command_sequence = 0;
    bool accepted = false;
    Reason reason = Reason::Other;
};

std::vector<std::uint8_t> encode_packet(const Packet& packet);
bool decode_packet(const std::vector<std::uint8_t>& bytes, Packet& packet);
std::vector<std::uint8_t> encode_position(const Position& position);
bool decode_position(const std::vector<std::uint8_t>& bytes, Position& position);
std::vector<std::uint8_t> encode_telemetry(const Telemetry& telemetry);
bool decode_telemetry(const std::vector<std::uint8_t>& bytes, Telemetry& telemetry);
std::vector<std::uint8_t> encode_reply(const Reply& reply);
bool decode_reply(const Packet& packet, Reply& reply);
bool valid_position(const Position& position);
bool valid_mode(std::uint8_t mode);
const char* mode_name(Mode mode);
const char* reason_name(Reason reason);
const char* message_name(MessageId message);

} // namespace drone
