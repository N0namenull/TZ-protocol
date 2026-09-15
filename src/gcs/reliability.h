#pragma once
#include "common/protocol.h"
#include <chrono>
#include <optional>
#include <unordered_set>

namespace drone {
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

enum class SequenceResult { Newest, Late, Duplicate, TooOld };
class SequenceStats {
public:
    SequenceResult observe(std::uint32_t sequence);
    std::uint64_t expected() const;
    std::uint64_t unique() const;
    std::uint64_t lost() const;
    double loss_rate() const;
private:
    bool initialized_ = false;
    std::uint64_t first_ = 0;
    std::uint64_t highest_ = 0;
    std::uint64_t unique_ = 0;
    std::unordered_set<std::uint64_t> seen_;
};

enum class RetryAction { None, Retry, TimedOut };
class PendingCommand {
public:
    void start(const Packet& packet, TimePoint now);
    bool active() const;
    const Packet& packet() const;
    RetryAction tick(TimePoint now);
    bool accept(const Reply& reply);
private:
    std::optional<Packet> packet_;
    TimePoint deadline_;
    int retries_ = 0;
};
}
