#include "common/protocol.h"
#include "drone/model.h"
#include "test_support.h"
#include <limits>

using namespace drone;

void test_packets() {
    const std::vector<std::uint8_t> arm_bytes = {
        0x4E, 0x4F, 0x52, 0x44, 0x0A, 0x00, 0x01, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x3E, 0x01
    };
    Packet arm{MessageId::Arm, 1, {}};
    check(encode_packet(arm) == arm_bytes, "ARM matches independently calculated wire bytes");
    Packet decoded;
    check(decode_packet(arm_bytes, decoded), "decode valid ARM");
    check(decoded.message_id == MessageId::Arm && decoded.sequence == 1 && decoded.payload.empty(),
          "ARM fields preserve order and width");
    for (std::size_t size = 0; size < arm_bytes.size(); ++size) {
        std::vector<std::uint8_t> truncated(arm_bytes.begin(), arm_bytes.begin() + size);
        check(!decode_packet(truncated, decoded), "reject truncated packet " + std::to_string(size));
    }
    for (std::size_t index = 0; index < arm_bytes.size(); ++index) {
        std::vector<std::uint8_t> damaged = arm_bytes;
        damaged[index] ^= 0x01;
        check(!decode_packet(damaged, decoded), "reject damaged byte " + std::to_string(index));
    }
    std::vector<std::uint8_t> extra = arm_bytes;
    extra.push_back(0);
    check(!decode_packet(extra, decoded), "reject trailing byte");

    // A checksum-correct packet must still pass the magic and length checks.
    std::vector<std::uint8_t> wrong_magic = arm_bytes;
    wrong_magic[0] = 0x4F;
    wrong_magic[12] = 0x3F;
    check(!decode_packet(wrong_magic, decoded), "reject wrong magic with correct checksum");
    std::vector<std::uint8_t> wrong_length = arm_bytes;
    wrong_length[10] = 1;
    wrong_length[12] = 0x3F;
    check(!decode_packet(wrong_length, decoded), "reject wrong length with correct checksum");

    Packet large{MessageId::Telemetry, 0x12345678, std::vector<std::uint8_t>(1000, 255)};
    check(decode_packet(encode_packet(large), decoded) && decoded.payload == large.payload &&
          decoded.sequence == 0x12345678, "checksum wraps modulo 65536");
}

void test_payloads() {
    Position position{1.0, -2.0, 3.0F};
    const std::vector<std::uint8_t> position_bytes = {
        0, 0, 0, 0, 0, 0, 0xF0, 0x3F,
        0, 0, 0, 0, 0, 0, 0, 0xC0, 0, 0, 0x40, 0x40
    };
    check(encode_position(position) == position_bytes, "IEEE754 little-endian GOTO bytes");
    Position decoded;
    check(decode_position(position_bytes, decoded) && decoded.latitude == 1.0 &&
          decoded.longitude == -2.0 && decoded.altitude == 3.0F, "decode float and double fields");
    check(!decode_position({}, decoded), "reject empty GOTO");

    Telemetry telemetry{position, 90.0F, 50.0F, true, Mode::Guided};
    std::vector<std::uint8_t> expected = position_bytes;
    expected.insert(expected.end(), {0, 0, 0xB4, 0x42, 0, 0, 0x48, 0x42, 1, 1});
    check(encode_telemetry(telemetry) == expected, "telemetry exact 30-byte layout without padding");
    Telemetry decoded_telemetry;
    check(decode_telemetry(expected, decoded_telemetry) && decoded_telemetry.armed &&
          decoded_telemetry.mode == Mode::Guided && decoded_telemetry.yaw == 90.0F,
          "decode telemetry fields");
    expected[28] = 2;
    check(!decode_telemetry(expected, decoded_telemetry), "reject armed outside 0/1");
    expected[28] = 1;
    expected[29] = 3;
    check(!decode_telemetry(expected, decoded_telemetry), "reject unknown telemetry mode");

    Reply reply{MessageId::GoTo, 0x12345678, false, Reason::NotArmed};
    const std::vector<std::uint8_t> reply_bytes{13, 0, 0x78, 0x56, 0x34, 0x12, 1};
    check(encode_reply(reply) == reply_bytes, "NACK exact wire bytes");
    Reply decoded_reply;
    check(decode_reply({MessageId::Nack, 9, reply_bytes}, decoded_reply) &&
          decoded_reply.command_sequence == 0x12345678 && !decoded_reply.accepted &&
          decoded_reply.reason == Reason::NotArmed, "NACK preserves command correlation");
    check(!decode_reply({MessageId::Ack, 9, reply_bytes}, decoded_reply), "ACK requires six bytes");

    check(valid_position({90, -180, 0}), "accept geographic boundaries");
    check(!valid_position({91, 0, 0}), "reject latitude out of range");
    check(!valid_position({0, 181, 0}), "reject longitude out of range");
    check(!valid_position({0, 0, -1}), "reject negative altitude");
    check(!valid_position({std::numeric_limits<double>::quiet_NaN(), 0, 0}), "reject NaN");
    check(!valid_position({0, 0, std::numeric_limits<float>::infinity()}), "reject infinity");
}

Packet command(MessageId id, std::vector<std::uint8_t> payload = {}) {
    return {id, 42, payload};
}

void test_model() {
    Model model({0, 0, 0}, 10.0, 3.0, 0.1);
    Position target{0.001, 0, 6};
    Packet go_to = command(MessageId::GoTo, encode_position(target));
    check(model.status() == SystemStatus::Standby, "initial state STANDBY");
    check(model.handle(go_to).reason == Reason::NotArmed, "GOTO requires ARM");
    Reply reply = model.handle(command(MessageId::Arm));
    check(reply.accepted && reply.command_sequence == 42 && reply.command_id == MessageId::Arm,
          "ARM transitions and correlates response");
    check(model.telemetry().armed, "ARM changes telemetry");
    check(!model.handle(command(MessageId::Arm)).accepted, "new ARM while armed rejected");
    check(model.handle(go_to).reason == Reason::BadMode, "GOTO requires GUIDED");
    check(model.handle(command(MessageId::SetMode, {1})).accepted, "set GUIDED");
    check(model.handle(go_to).accepted && model.has_target(), "valid GOTO sets target");
    model.update(1.0);
    check(model.telemetry().position.latitude > 0 && model.telemetry().position.latitude < target.latitude,
          "GOTO moves towards target");
    check(near(model.telemetry().position.altitude, 3), "vertical speed is three meters per second");
    check(near(model.telemetry().position.latitude * 111194.9266, 10.0, 0.01),
          "horizontal speed is ten meters per second");
    check(near(model.telemetry().battery, 99.9, 0.001), "battery drains while armed");
    model.update(100.0);
    check(near(model.telemetry().position.latitude, target.latitude) &&
          model.telemetry().position.altitude == target.altitude && !model.has_target(),
          "arrive exactly without overshooting");
    check(model.handle(command(MessageId::SetMode, {2})).accepted && model.has_target(), "RTL targets home");
    model.update(100.0);
    check(near(model.telemetry().position.latitude, 0) && near(model.telemetry().position.altitude, 0),
          "RTL returns to start");
    model.handle(command(MessageId::SetMode, {1}));
    model.handle(go_to);
    check(model.handle(command(MessageId::Disarm)).accepted && !model.has_target(), "DISARM cancels target");
    Position stopped = model.telemetry().position;
    model.update(10);
    check(model.telemetry().position.latitude == stopped.latitude, "disarmed drone stays still");
    check(model.handle(command(MessageId::Arm, {1})).reason == Reason::InvalidArgs,
          "ARM rejects unexpected payload before state change");
    check(model.handle(command(MessageId::SetMode, {9})).reason == Reason::InvalidArgs, "reject invalid mode");
    check(model.handle(command(MessageId::GoTo, {})).reason == Reason::InvalidArgs, "reject empty GOTO");

    Model critical({}, 10, 3, 100);
    critical.handle(command(MessageId::Arm));
    critical.update(1.0);
    check(critical.status() == SystemStatus::Critical && critical.telemetry().battery == 0,
          "battery exhaustion enters CRITICAL and clamps at zero");
    check(!critical.handle(command(MessageId::Disarm)).accepted, "DISARM rejected in CRITICAL");
}

int main() {
    test_packets();
    test_payloads();
    test_model();
    return finish_tests();
}
