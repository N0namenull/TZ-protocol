#include "common/udp_socket.h"
#include "common/console.h"
#include "drone/server.h"
#include "drone/model.h"
#include <chrono>
#include <deque>
#include <iostream>

namespace drone {
namespace {
using Clock = std::chrono::steady_clock;

struct CachedReply {
    Packet request;
    Reply reply;
    Clock::time_point created;
};

class CommandCache {
public:
    Reply handle(const Packet& command, Model& model, Clock::time_point now) {
        while (!entries_.empty() && now - entries_.front().created >= std::chrono::seconds(10)) {
            entries_.pop_front();
        }
        for (const CachedReply& entry : entries_) {
            if (entry.request.sequence == command.sequence) {
                if (entry.request.message_id == command.message_id && entry.request.payload == command.payload) {
                    return entry.reply;
                }
                return {command.message_id, command.sequence, false, Reason::InvalidArgs};
            }
        }
        Reply reply = model.handle(command);
        if (entries_.size() == 128) {
            entries_.pop_front();
        }
        entries_.push_back({command, reply, now});
        return reply;
    }
private:
    std::deque<CachedReply> entries_;
};

void send_message(UdpSocket& socket, const Endpoint& peer, MessageId id,
                  const std::vector<std::uint8_t>& payload, std::uint32_t& sequence) {
    Packet packet{id, sequence++, payload};
    if (!socket.send(encode_packet(packet), peer)) {
        std::cerr << "Send buffer full; packet seq=" << packet.sequence << " dropped\n";
    }
}
}

int run_drone(const Options& options) {
    Endpoint local(options.bind_ip, options.listen_port);
    Endpoint peer(options.peer_ip, options.peer_port);
    UdpSocket socket(local);
    Model model(options.start, options.speed, options.vertical_speed);
    CommandCache cache;
    std::uint32_t sequence = 1;
    std::uint64_t received = 0;
    std::uint64_t invalid = 0;
    std::uint64_t foreign = 0;
    Clock::time_point started = Clock::now();
    Clock::time_point previous_update = started;
    Clock::time_point next_telemetry = started;
    Clock::time_point next_heartbeat = started;
    std::cout << "DroneSim ready: " << options.bind_ip << ':' << options.listen_port
              << " -> " << options.peer_ip << ':' << options.peer_port << '\n' << std::flush;

    while (!ShutdownSignal::requested()) {
        Clock::time_point now = Clock::now();
        if (options.duration > 0 && std::chrono::duration<double>(now - started).count() >= options.duration) {
            break;
        }
        model.update(std::chrono::duration<double>(now - previous_update).count());
        previous_update = now;

        // A flood cannot postpone the model update and heartbeat indefinitely.
        for (int count = 0; count < 64; ++count) {
            std::vector<std::uint8_t> bytes;
            Endpoint sender;
            if (!socket.receive(bytes, sender)) {
                break;
            }
            if (!(sender == peer)) {
                ++foreign;
                continue;
            }
            Packet command;
            if (!decode_packet(bytes, command)) {
                ++invalid;
                continue;
            }
            ++received;
            Reply reply = cache.handle(command, model, now);
            MessageId reply_id = reply.accepted ? MessageId::Ack : MessageId::Nack;
            send_message(socket, peer, reply_id, encode_reply(reply), sequence);
            std::cout << message_name(reply_id) << ' ' << message_name(command.message_id)
                      << " cmd_seq=" << command.sequence;
            if (!reply.accepted) {
                std::cout << " reason=" << reason_name(reply.reason);
            }
            std::cout << '\n' << std::flush;
        }
        if (now >= next_telemetry) {
            send_message(socket, peer, MessageId::Telemetry, encode_telemetry(model.telemetry()), sequence);
            next_telemetry = now + std::chrono::milliseconds(100);
        }
        if (now >= next_heartbeat) {
            send_message(socket, peer, MessageId::Heartbeat,
                         {static_cast<std::uint8_t>(model.status())}, sequence);
            next_heartbeat = now + std::chrono::seconds(1);
        }
        socket.wait_readable(10);
    }
    std::cout << "DroneSim stopped: rx=" << received << " invalid=" << invalid << " foreign=" << foreign << '\n';
    return 0;
}
}
