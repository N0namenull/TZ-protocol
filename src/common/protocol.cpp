#include "common/protocol.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace drone {
namespace {

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);

class Writer {
public:
    void integer(std::uint64_t value, std::size_t width) {
        for (std::size_t index = 0; index < width; ++index) {
            bytes.push_back(static_cast<std::uint8_t>(value & 0xFF));
            value >>= 8;
        }
    }

    void floating(float value) {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        integer(bits, sizeof(bits));
    }

    void floating(double value) {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        integer(bits, sizeof(bits));
    }

    std::vector<std::uint8_t> bytes;
};

// Callers validate the full message size before reading any fields.
class Reader {
public:
    explicit Reader(const std::vector<std::uint8_t>& bytes) : bytes_(bytes) {}

    std::uint64_t integer(std::size_t width) {
        std::uint64_t value = 0;
        for (std::size_t index = 0; index < width; ++index) {
            value |= static_cast<std::uint64_t>(bytes_.at(offset_)) << (8 * index);
            ++offset_;
        }
        return value;
    }

    float float32() {
        std::uint32_t bits = static_cast<std::uint32_t>(integer(4));
        float value = 0;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    double float64() {
        std::uint64_t bits = integer(8);
        double value = 0;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

private:
    const std::vector<std::uint8_t>& bytes_;
    std::size_t offset_ = 0;
};

std::uint16_t checksum(const std::vector<std::uint8_t>& bytes, std::size_t count) {
    std::uint32_t sum = 0;
    for (std::size_t index = 0; index < count; ++index) {
        sum += bytes[index];
    }
    return static_cast<std::uint16_t>(sum & 0xFFFF);
}

void write_position(Writer& writer, const Position& position) {
    writer.floating(position.latitude);
    writer.floating(position.longitude);
    writer.floating(position.altitude);
}

Position read_position(Reader& reader) {
    Position position;
    position.latitude = reader.float64();
    position.longitude = reader.float64();
    position.altitude = reader.float32();
    return position;
}

} // namespace

std::vector<std::uint8_t> encode_packet(const Packet& packet) {
    // IPv4 UDP carries at most 65507 bytes, including our header and checksum.
    if (packet.payload.size() > 65507 - header_size - checksum_size) {
        throw std::invalid_argument("UDP payload is too large");
    }
    Writer writer;
    writer.integer(protocol_magic, 4);
    writer.integer(static_cast<std::uint16_t>(packet.message_id), 2);
    writer.integer(packet.sequence, 4);
    writer.integer(packet.payload.size(), 2);
    writer.bytes.insert(writer.bytes.end(), packet.payload.begin(), packet.payload.end());
    writer.integer(checksum(writer.bytes, writer.bytes.size()), 2);
    return writer.bytes;
}

bool decode_packet(const std::vector<std::uint8_t>& bytes, Packet& packet) {
    if (bytes.size() < header_size + checksum_size || bytes.size() > 65507) {
        return false;
    }
    Reader reader(bytes);
    if (reader.integer(4) != protocol_magic) {
        return false;
    }
    Packet parsed;
    parsed.message_id = static_cast<MessageId>(reader.integer(2));
    parsed.sequence = static_cast<std::uint32_t>(reader.integer(4));
    std::size_t payload_size = static_cast<std::size_t>(reader.integer(2));
    if (bytes.size() != header_size + payload_size + checksum_size) {
        return false;
    }
    std::size_t checksum_offset = bytes.size() - checksum_size;
    std::uint16_t received_checksum = static_cast<std::uint16_t>(bytes[checksum_offset]);
    received_checksum |= static_cast<std::uint16_t>(bytes[checksum_offset + 1]) << 8;
    if (received_checksum != checksum(bytes, checksum_offset)) {
        return false;
    }
    parsed.payload.assign(bytes.begin() + header_size, bytes.begin() + checksum_offset);
    packet = parsed;
    return true;
}

std::vector<std::uint8_t> encode_position(const Position& position) {
    Writer writer;
    write_position(writer, position);
    return writer.bytes;
}

bool decode_position(const std::vector<std::uint8_t>& bytes, Position& position) {
    if (bytes.size() != 20) {
        return false;
    }
    Reader reader(bytes);
    Position parsed = read_position(reader);
    if (!valid_position(parsed)) {
        return false;
    }
    position = parsed;
    return true;
}

std::vector<std::uint8_t> encode_telemetry(const Telemetry& telemetry) {
    Writer writer;
    write_position(writer, telemetry.position);
    writer.floating(telemetry.yaw);
    writer.floating(telemetry.battery);
    writer.integer(telemetry.armed ? 1 : 0, 1);
    writer.integer(static_cast<std::uint8_t>(telemetry.mode), 1);
    return writer.bytes;
}

bool decode_telemetry(const std::vector<std::uint8_t>& bytes, Telemetry& telemetry) {
    if (bytes.size() != 30) {
        return false;
    }
    Reader reader(bytes);
    Telemetry parsed;
    parsed.position = read_position(reader);
    parsed.yaw = reader.float32();
    parsed.battery = reader.float32();
    std::uint8_t armed = static_cast<std::uint8_t>(reader.integer(1));
    std::uint8_t mode = static_cast<std::uint8_t>(reader.integer(1));
    if (!valid_position(parsed.position) || !std::isfinite(parsed.yaw) ||
        parsed.yaw < 0 || parsed.yaw >= 360 || !std::isfinite(parsed.battery) ||
        parsed.battery < 0 || parsed.battery > 100 || armed > 1 || !valid_mode(mode)) {
        return false;
    }
    parsed.armed = armed == 1;
    parsed.mode = static_cast<Mode>(mode);
    telemetry = parsed;
    return true;
}

std::vector<std::uint8_t> encode_reply(const Reply& reply) {
    Writer writer;
    writer.integer(static_cast<std::uint16_t>(reply.command_id), 2);
    writer.integer(reply.command_sequence, 4);
    if (!reply.accepted) {
        writer.integer(static_cast<std::uint8_t>(reply.reason), 1);
    }
    return writer.bytes;
}

bool decode_reply(const Packet& packet, Reply& reply) {
    bool accepted = packet.message_id == MessageId::Ack;
    if (!accepted && packet.message_id != MessageId::Nack) {
        return false;
    }
    std::size_t expected_size = accepted ? 6 : 7;
    if (packet.payload.size() != expected_size) {
        return false;
    }
    Reader reader(packet.payload);
    Reply parsed;
    parsed.command_id = static_cast<MessageId>(reader.integer(2));
    parsed.command_sequence = static_cast<std::uint32_t>(reader.integer(4));
    parsed.accepted = accepted;
    if (!accepted) {
        std::uint8_t reason = static_cast<std::uint8_t>(reader.integer(1));
        if (reason < 1 || reason > 5) {
            return false;
        }
        parsed.reason = static_cast<Reason>(reason);
    }
    reply = parsed;
    return true;
}

bool valid_position(const Position& position) {
    return std::isfinite(position.latitude) && std::isfinite(position.longitude) &&
           std::isfinite(position.altitude) && position.latitude >= -90 &&
           position.latitude <= 90 && position.longitude >= -180 &&
           position.longitude <= 180 && position.altitude >= 0 && position.altitude <= 10000;
}

bool valid_mode(std::uint8_t mode) { return mode <= 2; }

const char* mode_name(Mode mode) {
    switch (mode) {
    case Mode::Manual: return "MANUAL";
    case Mode::Guided: return "GUIDED";
    case Mode::Rtl: return "RTL";
    }
    return "UNKNOWN";
}

const char* reason_name(Reason reason) {
    switch (reason) {
    case Reason::NotArmed: return "NOT_ARMED";
    case Reason::BadMode: return "BAD_MODE";
    case Reason::InvalidArgs: return "INVALID_ARGS";
    case Reason::Busy: return "BUSY";
    case Reason::Other: return "OTHER";
    }
    return "UNKNOWN";
}

const char* message_name(MessageId message) {
    switch (message) {
    case MessageId::Heartbeat: return "HEARTBEAT";
    case MessageId::Telemetry: return "TELEMETRY";
    case MessageId::Arm: return "ARM";
    case MessageId::Disarm: return "DISARM";
    case MessageId::SetMode: return "SET_MODE";
    case MessageId::GoTo: return "GOTO";
    case MessageId::Ack: return "ACK";
    case MessageId::Nack: return "NACK";
    }
    return "UNKNOWN";
}

} // namespace drone
