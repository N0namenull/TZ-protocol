#include "common/udp_socket.h"
#include "gcs/client.h"
#include "gcs/commands.h"
#include "gcs/reliability.h"
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <locale>
#include <optional>
#include <sstream>

namespace drone {
namespace {

bool newer(std::uint32_t candidate, std::uint32_t previous) {
    std::uint32_t delta = candidate - previous;
    return delta != 0 && delta < 0x80000000U;
}

std::string timestamp_utc() {
    auto now = std::chrono::system_clock::now();
    std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    if (gmtime_s(&utc, &seconds) != 0) {
        throw std::runtime_error("Cannot format UTC time");
    }
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::ostringstream text;
    text << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.'
         << std::setfill('0') << std::setw(3) << milliseconds << 'Z';
    return text.str();
}

class Client {
public:
    Client(const Options& options, Console& console)
        : options_(options), console_(console), peer_(options.peer_ip, options.peer_port),
          socket_(Endpoint(options.bind_ip, options.listen_port)) {
        log_.exceptions(std::ios::failbit | std::ios::badbit);
        log_.imbue(std::locale::classic());
        log_.open(options.log_path, std::ios::out | std::ios::trunc);
        log_ << "timestamp,seq,lat,lon,alt,yaw,battery,armed,mode\n";
        log_.flush();
    }

    void run(CommandQueue& commands, std::atomic<bool>& stop) {
        TimePoint started = Clock::now();
        TimePoint next_display = started;
        console_.line("GCS ready: " + options_.bind_ip + ":" + std::to_string(options_.listen_port) +
                      " -> " + options_.peer_ip + ":" + std::to_string(options_.peer_port));
        console_.line("Commands: status | arm | disarm | mode <0|1|2> | goto <lat> <lon> <alt> | help | quit");
        while (!stop.load() && !ShutdownSignal::requested()) {
            TimePoint now = Clock::now();
            if (options_.duration > 0 && std::chrono::duration<double>(now - started).count() >= options_.duration) {
                break;
            }
            receive_packets(now);
            std::string line;
            for (int count = 0; count < 32 && commands.pop(line); ++count) {
                handle_input(line, now, stop);
                if (stop.load()) {
                    break;
                }
            }
            if (stop.load()) {
                break;
            }
            service_commands(now);
            if (commands.finished() && outgoing_.empty() && !pending_.active()) {
                break;
            }
            if (now >= next_display) {
                print_status(now);
                next_display = now + std::chrono::seconds(1);
            }
            socket_.wait_readable(10);
        }
        // close() reports a deferred write error; the outer worker catches it.
        log_.close();
        stop.store(true);
        console_.line("GCS stopped. Log: " + options_.log_path);
    }

private:
    void handle_input(const std::string& line, TimePoint now, std::atomic<bool>& stop) {
        UserCommand command = parse_command(line);
        switch (command.action) {
        case Action::Empty:
            break;
        case Action::Status:
            print_status(now);
            break;
        case Action::Help:
            console_.line("status | arm | disarm | mode 0=MANUAL/1=GUIDED/2=RTL | goto lat lon alt | quit");
            break;
        case Action::Quit:
            stop.store(true);
            break;
        case Action::Invalid:
            console_.line("Input error: " + command.error);
            break;
        case Action::Send:
            if (outgoing_.size() >= 64) {
                console_.line("Command queue is full; command rejected.");
            } else {
                outgoing_.push_back(command.packet);
            }
            break;
        }
    }

    void transmit(const Packet& packet, bool retry) {
        bool sent = socket_.send(encode_packet(packet), peer_);
        std::string prefix = retry ? "RETRY " : "SEND ";
        console_.line(prefix + message_name(packet.message_id) + " cmd_seq=" + std::to_string(packet.sequence));
        if (!sent) {
            console_.line("UDP send buffer full; waiting for retry timeout.");
        }
    }

    void service_commands(TimePoint now) {
        if (pending_.active()) {
            Packet packet = pending_.packet();
            RetryAction action = pending_.tick(now);
            if (action == RetryAction::Retry) {
                transmit(packet, true);
            } else if (action == RetryAction::TimedOut) {
                console_.line(std::string("TIMEOUT ") + message_name(packet.message_id) +
                              " cmd_seq=" + std::to_string(packet.sequence));
            }
        }
        if (!pending_.active() && !outgoing_.empty()) {
            Packet packet = outgoing_.front();
            outgoing_.pop_front();
            packet.sequence = send_sequence_++;
            transmit(packet, false);
            pending_.start(packet, now);
        }
    }

    void receive_packets(TimePoint now) {
        for (int count = 0; count < 64; ++count) {
            Endpoint sender;
            std::vector<std::uint8_t> bytes;
            if (!socket_.receive(bytes, sender)) {
                break;
            }
            if (!(sender == peer_)) {
                ++foreign_;
                continue;
            }
            Packet packet;
            if (!decode_packet(bytes, packet)) {
                ++invalid_;
                continue;
            }
            Telemetry telemetry;
            Reply reply;
            bool valid = false;
            switch (packet.message_id) {
            case MessageId::Heartbeat:
                valid = packet.payload.size() == 1 && packet.payload[0] <= 3;
                break;
            case MessageId::Telemetry:
                valid = decode_telemetry(packet.payload, telemetry);
                break;
            case MessageId::Ack:
            case MessageId::Nack:
                valid = decode_reply(packet, reply);
                break;
            default:
                break;
            }
            if (!valid) {
                ++invalid_;
                continue;
            }
            ++received_;
            SequenceResult result = sequences_.observe(packet.sequence);
            if (result == SequenceResult::Duplicate) {
                ++duplicates_;
                continue;
            }
            if (result == SequenceResult::TooOld) {
                ++stale_;
                continue;
            }
            if (result == SequenceResult::Late) {
                ++late_;
            }
            if (packet.message_id == MessageId::Heartbeat) {
                if (!heartbeat_sequence_ || newer(packet.sequence, *heartbeat_sequence_)) {
                    heartbeat_sequence_ = packet.sequence;
                    last_heartbeat_ = now;
                    system_status_ = packet.payload[0];
                }
            } else if (packet.message_id == MessageId::Telemetry) {
                if (!telemetry_sequence_ || newer(packet.sequence, *telemetry_sequence_)) {
                    telemetry_sequence_ = packet.sequence;
                    latest_ = telemetry;
                    write_log(packet.sequence, telemetry);
                }
            } else if (pending_.accept(reply)) {
                std::string text = reply.accepted ? "ACK " : "NACK ";
                text += message_name(reply.command_id);
                text += " cmd_seq=" + std::to_string(reply.command_sequence);
                if (!reply.accepted) {
                    text += " reason=";
                    text += reason_name(reply.reason);
                }
                console_.line(text);
            }
        }
    }

    void write_log(std::uint32_t sequence, const Telemetry& telemetry) {
        log_ << timestamp_utc() << ',' << sequence << ',' << std::fixed << std::setprecision(9)
             << telemetry.position.latitude << ',' << telemetry.position.longitude << ','
             << std::setprecision(3) << telemetry.position.altitude << ',' << telemetry.yaw << ','
             << telemetry.battery << ',' << (telemetry.armed ? 1 : 0) << ','
             << static_cast<int>(telemetry.mode) << '\n';
        log_.flush();
    }

    void print_status(TimePoint now) {
        std::ostringstream text;
        text.imbue(std::locale::classic());
        text << std::fixed;
        if (latest_) {
            const Telemetry& telemetry = *latest_;
            text << "TELEMETRY seq=" << *telemetry_sequence_ << std::setprecision(7)
                 << " lat=" << telemetry.position.latitude << " lon=" << telemetry.position.longitude
                 << std::setprecision(2) << " alt=" << telemetry.position.altitude
                 << " yaw=" << telemetry.yaw << " battery=" << telemetry.battery
                 << " armed=" << (telemetry.armed ? 1 : 0) << " mode=" << mode_name(telemetry.mode) << '\n';
        } else {
            text << "Waiting for telemetry...\n";
        }
        text << "STATS rx=" << received_ << " unique=" << sequences_.unique()
             << " lost=" << sequences_.lost() << std::setprecision(2) << " loss=" << sequences_.loss_rate()
             << "% invalid=" << invalid_ << " foreign=" << foreign_ << " duplicates=" << duplicates_
             << " late=" << late_ << " stale=" << stale_ << " heartbeat_age=";
        if (last_heartbeat_) {
            text << std::chrono::duration<double>(now - *last_heartbeat_).count() << "s"
                 << " system_status=" << static_cast<int>(system_status_);
        } else {
            text << "never";
        }
        console_.status(text.str());
    }

    const Options& options_;
    Console& console_;
    Endpoint peer_;
    UdpSocket socket_;
    std::ofstream log_;
    std::uint32_t send_sequence_ = 1;
    std::deque<Packet> outgoing_;
    PendingCommand pending_;
    SequenceStats sequences_;
    std::optional<Telemetry> latest_;
    std::optional<std::uint32_t> telemetry_sequence_;
    std::optional<std::uint32_t> heartbeat_sequence_;
    std::optional<TimePoint> last_heartbeat_;
    std::uint8_t system_status_ = 0;
    std::uint64_t received_ = 0;
    std::uint64_t invalid_ = 0;
    std::uint64_t foreign_ = 0;
    std::uint64_t duplicates_ = 0;
    std::uint64_t late_ = 0;
    std::uint64_t stale_ = 0;
};
}

void run_gcs(const Options& options, CommandQueue& commands, Console& console, std::atomic<bool>& stop) {
    Client client(options, console);
    client.run(commands, stop);
}

} // namespace drone
