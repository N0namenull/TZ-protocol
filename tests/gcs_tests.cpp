#include "gcs/commands.h"
#include "gcs/reliability.h"
#include "test_support.h"
#include <limits>

using namespace drone;
using namespace std::chrono_literals;

void test_cli() {
    check(parse_command(" ").action == Action::Empty, "ignore whitespace");
    check(parse_command("arm").packet.message_id == MessageId::Arm &&
          parse_command("arm").action == Action::Send, "parse ARM");
    check(parse_command("status").action == Action::Status, "parse status");
    check(parse_command("quit").action == Action::Quit, "parse quit");
    check(parse_command("arm extra").action == Action::Invalid, "reject extra argument");
    check(parse_command("mode 1").packet.payload == std::vector<std::uint8_t>{1}, "encode mode");
    check(parse_command("mode 1.5").action == Action::Invalid, "reject fractional mode");
    check(parse_command("mode 3").action == Action::Invalid, "reject undefined mode");
    check(parse_command("mode 999999999999999999999").action == Action::Invalid, "reject integer overflow");
    check(parse_command("goto 55.751 37.61 20").action == Action::Send, "parse GOTO");
    for (const std::string text : {"goto nan 0 0", "goto 0 inf 0", "goto 0 0 -1", "goto 0 0 1e99",
                                   "goto 91 0 0", "goto 0 0", "goto 0 0 0 extra", "status extra"}) {
        check(parse_command(text).action == Action::Invalid, "reject malformed CLI: " + text);
    }
}

void test_sequences() {
    SequenceStats stats;
    check(stats.observe(10) == SequenceResult::Newest, "first packet initializes session");
    stats.observe(12);
    check(stats.expected() == 3 && stats.unique() == 2 && stats.lost() == 1,
          "gap counted across all message types");
    check(near(stats.loss_rate(), 100.0 / 3.0), "loss percentage uses expected count");
    check(stats.observe(11) == SequenceResult::Late && stats.lost() == 0, "late arrival repairs loss");
    check(stats.observe(11) == SequenceResult::Duplicate && stats.unique() == 3,
          "duplicate does not inflate unique count");
    check(stats.observe(9) == SequenceResult::TooOld && stats.lost() == 0, "ignore pre-session packets");
    SequenceStats wrap;
    wrap.observe(std::numeric_limits<std::uint32_t>::max() - 1);
    wrap.observe(0);
    check(wrap.lost() == 1, "wrap preserves a gap");
    wrap.observe(std::numeric_limits<std::uint32_t>::max());
    check(wrap.unique() == 3 && wrap.lost() == 0, "late packet across wrap fills gap");
    stats.observe(10000);
    check(stats.observe(12) == SequenceResult::TooOld, "bounded reorder window rejects stale packet");
}

void test_retries() {
    PendingCommand pending;
    TimePoint start{};
    Packet packet{MessageId::Arm, 42, {}};
    pending.start(packet, start);
    check(pending.active(), "starting command enters waiting state");
    check(pending.tick(start + 749ms) == RetryAction::None, "no early retry");
    check(pending.tick(start + 750ms) == RetryAction::Retry && pending.packet().sequence == 42,
          "first retry preserves sequence");
    check(pending.tick(start + 1500ms) == RetryAction::Retry, "second retry");
    check(pending.tick(start + 2250ms) == RetryAction::TimedOut && !pending.active(),
          "bounded retries stop after three attempts");
    pending.start(packet, start);
    check(!pending.accept({MessageId::Arm, 43, true, Reason::Other}) && pending.active(),
          "wrong sequence cannot complete command");
    check(!pending.accept({MessageId::Disarm, 42, true, Reason::Other}), "wrong command ID ignored");
    check(pending.accept({MessageId::Arm, 42, false, Reason::Busy}) && !pending.active(),
          "matching NACK completes without retry");
    check(pending.tick(start + 3000ms) == RetryAction::None, "completed request never retried");
}

int main() {
    test_cli();
    test_sequences();
    test_retries();
    return finish_tests();
}
