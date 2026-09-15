#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdint>
#include <string>
#include <vector>

namespace drone {

class WinsockRuntime {
public:
    WinsockRuntime();
    ~WinsockRuntime();
    WinsockRuntime(const WinsockRuntime&) = delete;
    WinsockRuntime& operator=(const WinsockRuntime&) = delete;
};

struct Endpoint {
    sockaddr_in address{};
    Endpoint() = default;
    Endpoint(const std::string& ip, std::uint16_t port);
    bool operator==(const Endpoint& other) const;
};

class UdpSocket {
public:
    explicit UdpSocket(const Endpoint& local);
    ~UdpSocket();
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;
    bool send(const std::vector<std::uint8_t>& bytes, const Endpoint& peer);
    bool receive(std::vector<std::uint8_t>& bytes, Endpoint& peer);
    void wait_readable(int milliseconds);
private:
    SOCKET socket_ = INVALID_SOCKET;
};

} // namespace drone
