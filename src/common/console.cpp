#include "common/console.h"
#include <iostream>
#include <stdexcept>

namespace drone {

Console::~Console() {
    if (view_.active()) {
        view_.close();
        // Keep the final error/stop message visible after restoring the shell.
        std::cout << view_.last_event() << '\n' << std::flush;
    }
}

void Console::line(const std::string& text) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (view_.active()) {
        view_.add_event(text);
    } else {
        std::cout << text << '\n' << std::flush;
    }
}

void Console::status(const std::string& text) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (view_.active()) {
        view_.set_status(text);
    } else {
        std::cout << text << '\n' << std::flush;
    }
}

void Console::enable_dashboard() {
    std::lock_guard<std::mutex> lock(mutex_);
    view_.enable();
}

void Console::set_input(const std::string& text) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (view_.active()) {
        view_.set_input(text);
    } else {
        // ConsoleInput appends characters or removes them from the end.
        // Keep manual echo working if Windows cannot activate the dashboard.
        for (std::size_t index = text.size(); index < fallback_input_.size(); ++index) {
            std::cout << "\b \b";
        }
        if (text.size() > fallback_input_.size()) {
            std::cout << text.substr(fallback_input_.size());
        }
        std::cout << std::flush;
        fallback_input_ = text;
    }
}

void Console::submit_input(const std::string& text) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (view_.active()) {
        view_.set_input("");
        view_.add_event("gcs> " + text);
    } else {
        std::cout << '\n' << std::flush;
        fallback_input_.clear();
    }
}

void Console::refresh() {
    std::lock_guard<std::mutex> lock(mutex_);
    view_.refresh();
}

bool CommandQueue::push(const std::string& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_ || commands_.size() >= 128) {
        return false;
    }
    commands_.push_back(command);
    return true;
}

bool CommandQueue::pop(std::string& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (commands_.empty()) {
        return false;
    }
    command = commands_.front();
    commands_.pop_front();
    return true;
}

void CommandQueue::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
}

bool CommandQueue::finished() {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_ && commands_.empty();
}

std::atomic<bool> ShutdownSignal::requested_{false};

BOOL WINAPI ShutdownSignal::handler(DWORD event) {
    if (event == CTRL_C_EVENT || event == CTRL_BREAK_EVENT) {
        requested_.store(true);
        return TRUE;
    }
    return FALSE;
}

ShutdownSignal::ShutdownSignal() {
    requested_.store(false);
    if (!SetConsoleCtrlHandler(handler, TRUE)) {
        throw std::runtime_error("Cannot register Ctrl+C handler");
    }
    // Child processes can inherit "ignore Ctrl+C" even in a new console.
    // Registering our handler does not clear that separate Windows attribute.
    if (!SetConsoleCtrlHandler(nullptr, FALSE)) {
        SetConsoleCtrlHandler(handler, FALSE);
        throw std::runtime_error("Cannot enable Ctrl+C handling");
    }
}

ShutdownSignal::~ShutdownSignal() { SetConsoleCtrlHandler(handler, FALSE); }
bool ShutdownSignal::requested() { return requested_.load(); }

ConsoleInput::ConsoleInput(Console& console) : console_(console) {
    input_ = GetStdHandle(STD_INPUT_HANDLE);
    if (input_ == nullptr || input_ == INVALID_HANDLE_VALUE) {
        ended_ = true;
        return;
    }
    file_type_ = GetFileType(input_);
    is_console_ = GetConsoleMode(input_, &original_mode_) != 0;
    if (is_console_) {
        DWORD mode = original_mode_;
        mode &= ~(ENABLE_LINE_INPUT | ENABLE_ECHO_INPUT | ENABLE_QUICK_EDIT_MODE);
        mode |= ENABLE_EXTENDED_FLAGS | ENABLE_PROCESSED_INPUT;
        if (!SetConsoleMode(input_, mode)) {
            throw std::runtime_error("Cannot configure console input");
        }
        console_.enable_dashboard();
    }
}

ConsoleInput::~ConsoleInput() {
    if (is_console_) {
        SetConsoleMode(input_, original_mode_);
    }
}

InputResult ConsoleInput::accept_character(char character, std::string& line) {
    if (character == '\r') {
        if (!is_console_) {
            return InputResult::Waiting;
        }
        character = '\n';
    }
    if (character == '\n') {
        if (discarded_) {
            line = "__invalid_or_oversized_input_line__";
        } else {
            line = buffer_;
        }
        buffer_.clear();
        discarded_ = false;
        if (is_console_) {
            console_.submit_input(line);
        }
        return InputResult::Line;
    }
    if (is_console_ && character == '\b') {
        if (!buffer_.empty() && !discarded_) {
            buffer_.pop_back();
            console_.set_input(buffer_);
        }
        return InputResult::Waiting;
    }
    if ((character >= 32 && character <= 126) || character == '\t') {
        if (buffer_.size() < 1024 && !discarded_) {
            buffer_.push_back(character);
            if (is_console_) {
                console_.set_input(buffer_);
            }
        } else {
            discarded_ = true;
        }
    } else if (!is_console_ || character != 0) {
        // Never delete invalid bytes and accidentally turn them into a command.
        // Console navigation keys with no text (AsciiChar == 0) are just ignored.
        discarded_ = true;
    }
    return InputResult::Waiting;
}

InputResult ConsoleInput::poll(std::string& line) {
    console_.refresh();
    if (ended_) {
        return InputResult::End;
    }
    // Bound the work per call so stop requests cannot be starved by a large pipe.
    for (int count = 0; count < 256; ++count) {
        char character = 0;
        if (is_console_) {
            if (remaining_repeats_ > 0) {
                character = repeated_character_;
                --remaining_repeats_;
            } else {
                INPUT_RECORD event{};
                DWORD available = 0;
                if (!PeekConsoleInputA(input_, &event, 1, &available)) {
                    throw std::runtime_error("PeekConsoleInput failed");
                }
                if (available == 0) {
                    return InputResult::Waiting;
                }
                if (!ReadConsoleInputA(input_, &event, 1, &available)) {
                    throw std::runtime_error("ReadConsoleInput failed");
                }
                if (event.EventType != KEY_EVENT || !event.Event.KeyEvent.bKeyDown) {
                    continue;
                }
                character = event.Event.KeyEvent.uChar.AsciiChar;
                repeated_character_ = character;
                if (event.Event.KeyEvent.wRepeatCount > 1) {
                    remaining_repeats_ = event.Event.KeyEvent.wRepeatCount - 1;
                }
            }
        } else {
            if (file_type_ == FILE_TYPE_PIPE) {
                DWORD available = 0;
                if (!PeekNamedPipe(input_, nullptr, 0, nullptr, &available, nullptr)) {
                    if (GetLastError() != ERROR_BROKEN_PIPE) {
                        throw std::runtime_error("PeekNamedPipe failed");
                    }
                    ended_ = true;
                } else if (available == 0) {
                    return InputResult::Waiting;
                }
            }
            DWORD received = 0;
            if (!ended_ && !ReadFile(input_, &character, 1, &received, nullptr)) {
                if (GetLastError() != ERROR_BROKEN_PIPE) {
                    throw std::runtime_error("ReadFile stdin failed");
                }
                ended_ = true;
            }
            if (received == 0) {
                ended_ = true;
            }
            if (ended_) {
                if (!buffer_.empty() || discarded_) {
                    return accept_character('\n', line);
                }
                return InputResult::End;
            }
        }
        InputResult result = accept_character(character, line);
        if (result == InputResult::Line) {
            return result;
        }
    }
    return InputResult::Waiting;
}

} // namespace drone
