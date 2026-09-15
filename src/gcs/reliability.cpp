#include "gcs/reliability.h"
#include <stdexcept>

namespace drone {
namespace {
constexpr std::uint64_t reorder_window = 4096;
constexpr std::uint32_t half_sequence_space = 0x80000000U;
constexpr auto reply_timeout = std::chrono::milliseconds(750);
constexpr int maximum_retries = 2;
}

SequenceResult SequenceStats::observe(std::uint32_t sequence) {
    if (!initialized_) {
        initialized_ = true;
        first_ = sequence;
        highest_ = sequence;
        unique_ = 1;
        seen_.insert(highest_);
        return SequenceResult::Newest;
    }
    std::uint32_t forward = sequence - static_cast<std::uint32_t>(highest_);
    if (forward == 0) {
        return SequenceResult::Duplicate;
    }
    if (forward < half_sequence_space) {
        highest_ += forward;
        ++unique_;
        for (auto iterator = seen_.begin(); iterator != seen_.end();) {
            if (highest_ - *iterator >= reorder_window) {
                iterator = seen_.erase(iterator);
            } else {
                ++iterator;
            }
        }
        seen_.insert(highest_);
        return SequenceResult::Newest;
    }
    std::uint32_t behind = static_cast<std::uint32_t>(highest_) - sequence;
    if (behind >= reorder_window || behind > highest_ - first_) {
        return SequenceResult::TooOld;
    }
    std::uint64_t extended = highest_ - behind;
    if (seen_.find(extended) != seen_.end()) {
        return SequenceResult::Duplicate;
    }
    seen_.insert(extended);
    ++unique_;
    return SequenceResult::Late;
}

std::uint64_t SequenceStats::expected() const {
    if (!initialized_) {
        return 0;
    }
    return highest_ - first_ + 1;
}

std::uint64_t SequenceStats::unique() const { return unique_; }
std::uint64_t SequenceStats::lost() const { return expected() - unique_; }

double SequenceStats::loss_rate() const {
    if (expected() == 0) {
        return 0;
    }
    return 100.0 * static_cast<double>(lost()) / static_cast<double>(expected());
}

void PendingCommand::start(const Packet& packet, TimePoint now) {
    if (packet_) {
        throw std::logic_error("A command is already pending");
    }
    packet_ = packet;
    retries_ = 0;
    deadline_ = now + reply_timeout;
}

bool PendingCommand::active() const { return packet_.has_value(); }
const Packet& PendingCommand::packet() const { return packet_.value(); }

RetryAction PendingCommand::tick(TimePoint now) {
    if (!packet_ || now < deadline_) {
        return RetryAction::None;
    }
    if (retries_ >= maximum_retries) {
        packet_.reset();
        return RetryAction::TimedOut;
    }
    ++retries_;
    deadline_ = now + reply_timeout;
    return RetryAction::Retry;
}

bool PendingCommand::accept(const Reply& reply) {
    if (!packet_ || reply.command_sequence != packet_->sequence || reply.command_id != packet_->message_id) {
        return false;
    }
    packet_.reset();
    return true;
}

} // namespace drone
