#include "common/udp_socket.h"
#include <mswsock.h>
#include <array>
#include <stdexcept>

namespace drone {
namespace {
std::runtime_error socket_error(const std::string& operation) {
    return std::runtime_error(operation + " failed (Winsock " + std::to_string(WSAGetLastError()) + ")");
}
}

WinsockRuntime::WinsockRuntime() {
    WSADATA data{};
    int result = WSAStartup(MAKEWORD(2, 2), &data);
    if (result != 0) {
        throw std::runtime_error("WSAStartup failed: " + std::to_string(result));
    }
}

WinsockRuntime::~WinsockRuntime() { WSACleanup(); }

Endpoint::Endpoint(const std::string& ip, std::uint16_t port) {
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1) {
        throw std::invalid_argument("Invalid numeric IPv4 address: " + ip);
    }
}

bool Endpoint::operator==(const Endpoint& other) const {
    return address.sin_addr.s_addr == other.address.sin_addr.s_addr && address.sin_port == other.address.sin_port;
}

UdpSocket::UdpSocket(const Endpoint& local) {
    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) {
        throw socket_error("socket");
    }
    try {
        BOOL exclusive = TRUE;
        if (setsockopt(socket_, SOL_SOCKET, SO_EXCLUSIVEADDRUSE,
                       reinterpret_cast<const char*>(&exclusive), sizeof(exclusive)) == SOCKET_ERROR) {
            throw socket_error("setsockopt");
        }
        if (bind(socket_, reinterpret_cast<const sockaddr*>(&local.address), sizeof(local.address)) == SOCKET_ERROR) {
            throw socket_error("bind (check whether the port is already in use)");
        }
        u_long nonblocking = 1;
        if (ioctlsocket(socket_, FIONBIO, &nonblocking) == SOCKET_ERROR) {
            throw socket_error("ioctlsocket");
        }
        // An absent localhost peer can produce ICMP Port Unreachable. That is
        // expected for UDP during startup and is handled by command timeouts.
        BOOL report_reset = FALSE;
        DWORD returned = 0;
        if (WSAIoctl(socket_, SIO_UDP_CONNRESET, &report_reset, sizeof(report_reset),
                     nullptr, 0, &returned, nullptr, nullptr) == SOCKET_ERROR) {
            throw socket_error("WSAIoctl SIO_UDP_CONNRESET");
        }
    } catch (...) {
        closesocket(socket_);
        socket_ = INVALID_SOCKET;
        throw;
    }
}

UdpSocket::~UdpSocket() {
    if (socket_ != INVALID_SOCKET) {
        closesocket(socket_);
    }
}

bool UdpSocket::send(const std::vector<std::uint8_t>& bytes, const Endpoint& peer) {
    int sent = sendto(socket_, reinterpret_cast<const char*>(bytes.data()), static_cast<int>(bytes.size()), 0,
                      reinterpret_cast<const sockaddr*>(&peer.address), sizeof(peer.address));
    if (sent == SOCKET_ERROR) {
        if (WSAGetLastError() == WSAEWOULDBLOCK) {
            return false;
        }
        throw socket_error("sendto");
    }
    if (static_cast<std::size_t>(sent) != bytes.size()) {
        throw std::runtime_error("Incomplete UDP datagram send");
    }
    return true;
}

bool UdpSocket::receive(std::vector<std::uint8_t>& bytes, Endpoint& peer) {
    std::array<char, 65536> buffer{};
    int address_size = sizeof(peer.address);
    int received = recvfrom(socket_, buffer.data(), static_cast<int>(buffer.size()), 0,
                            reinterpret_cast<sockaddr*>(&peer.address), &address_size);
    if (received == SOCKET_ERROR) {
        if (WSAGetLastError() == WSAEWOULDBLOCK || WSAGetLastError() == WSAECONNRESET) {
            return false;
        }
        throw socket_error("recvfrom");
    }
    // A zero-byte UDP datagram is data, not EOF; the protocol parser rejects it.
    bytes.assign(buffer.begin(), buffer.begin() + received);
    return true;
}

void UdpSocket::wait_readable(int milliseconds) {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(socket_, &read_set);
    timeval timeout{};
    timeout.tv_sec = milliseconds / 1000;
    timeout.tv_usec = (milliseconds % 1000) * 1000;
    if (select(0, &read_set, nullptr, nullptr, &timeout) == SOCKET_ERROR) {
        throw socket_error("select");
    }
}

} // namespace drone
