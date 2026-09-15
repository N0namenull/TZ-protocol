#pragma once
#include <windows.h>
#include "common/terminal_view.h"
#include <atomic>
#include <deque>
#include <mutex>
#include <string>

namespace drone {

class Console {
public:
    ~Console();
    void line(const std::string& text);
    void status(const std::string& text);
    void enable_dashboard();
    void set_input(const std::string& text);
    void submit_input(const std::string& text);
    void refresh();
private:
    std::mutex mutex_;
    TerminalView view_;
    std::string fallback_input_;
};

class CommandQueue {
public:
    bool push(const std::string& command);
    bool pop(std::string& command);
    void close();
    bool finished();
private:
    std::mutex mutex_;
    std::deque<std::string> commands_;
    bool closed_ = false;
};

class ShutdownSignal {
public:
    ShutdownSignal();
    ~ShutdownSignal();
    static bool requested();
    ShutdownSignal(const ShutdownSignal&) = delete;
    ShutdownSignal& operator=(const ShutdownSignal&) = delete;
private:
    static BOOL WINAPI handler(DWORD event);
    static std::atomic<bool> requested_;
};

enum class InputResult { Waiting, Line, End };

class ConsoleInput {
public:
    explicit ConsoleInput(Console& console);
    ~ConsoleInput();
    InputResult poll(std::string& line);
    ConsoleInput(const ConsoleInput&) = delete;
    ConsoleInput& operator=(const ConsoleInput&) = delete;
private:
    InputResult accept_character(char character, std::string& line);
    Console& console_;
    HANDLE input_ = INVALID_HANDLE_VALUE;
    DWORD original_mode_ = 0;
    DWORD file_type_ = FILE_TYPE_UNKNOWN;
    bool is_console_ = false;
    bool ended_ = false;
    bool discarded_ = false;
    std::string buffer_;
    char repeated_character_ = 0;
    unsigned int remaining_repeats_ = 0;
};

} // namespace drone
