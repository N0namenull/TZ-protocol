#pragma once

#include <windows.h>
#include <deque>
#include <string>

namespace drone {

// Console serializes all calls to this view with its existing mutex.
class TerminalView {
public:
    ~TerminalView();
    bool enable();
    bool active() const;
    void close();
    void set_status(const std::string& text);
    void add_event(const std::string& text);
    void set_input(const std::string& text);
    void refresh(bool force = false);
    const std::string& last_event() const;

    TerminalView() = default;
    TerminalView(const TerminalView&) = delete;
    TerminalView& operator=(const TerminalView&) = delete;

private:
    HANDLE original_output_ = INVALID_HANDLE_VALUE;
    HANDLE screen_ = INVALID_HANDLE_VALUE;
    SMALL_RECT previous_window_{};
    bool rendered_ = false;
    std::string status_ = "Waiting for telemetry...";
    std::string input_;
    std::string last_event_;
    std::deque<std::string> events_;
};

} // namespace drone
