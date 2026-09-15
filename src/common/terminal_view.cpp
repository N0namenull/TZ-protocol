#include "common/terminal_view.h"

#include <algorithm>
#include <sstream>
#include <vector>

namespace drone {
namespace {

std::vector<std::string> wrap_lines(const std::string& text, std::size_t width) {
    std::vector<std::string> lines;
    std::istringstream input(text);
    std::string paragraph;
    while (std::getline(input, paragraph)) {
        std::replace(paragraph.begin(), paragraph.end(), '\t', ' ');
        while (paragraph.size() > width) {
            std::size_t split = paragraph.rfind(' ', width);
            if (split == std::string::npos || split == 0) {
                split = width;
            }
            lines.push_back(paragraph.substr(0, split));
            paragraph.erase(0, split);
            if (!paragraph.empty() && paragraph.front() == ' ') {
                paragraph.erase(0, 1);
            }
        }
        lines.push_back(paragraph);
    }
    return lines;
}

void put_row(std::vector<CHAR_INFO>& cells, int width, int row,
             const std::string& text, WORD attributes) {
    // Leave the final column free so the input cursor cannot wrap to another row.
    int count = std::min(static_cast<int>(text.size()), width - 1);
    for (int column = 0; column < count; ++column) {
        CHAR_INFO& cell = cells[static_cast<std::size_t>(row * width + column)];
        cell.Char.AsciiChar = text[static_cast<std::size_t>(column)];
        cell.Attributes = attributes;
    }
}

} // namespace

TerminalView::~TerminalView() { close(); }

bool TerminalView::enable() {
    if (active()) {
        return true;
    }
    original_output_ = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    if (!GetConsoleMode(original_output_, &mode)) {
        return false; // Files and pipes keep the ordinary line-oriented output.
    }
    screen_ = CreateConsoleScreenBuffer(GENERIC_READ | GENERIC_WRITE,
                                        FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                                        CONSOLE_TEXTMODE_BUFFER, nullptr);
    if (screen_ == INVALID_HANDLE_VALUE) {
        return false;
    }
    if (!SetConsoleActiveScreenBuffer(screen_)) {
        CloseHandle(screen_);
        screen_ = INVALID_HANDLE_VALUE;
        return false;
    }
    refresh(true);
    return true;
}

bool TerminalView::active() const { return screen_ != INVALID_HANDLE_VALUE; }

void TerminalView::close() {
    if (active()) {
        SetConsoleActiveScreenBuffer(original_output_);
        CloseHandle(screen_);
        screen_ = INVALID_HANDLE_VALUE;
    }
}

void TerminalView::set_status(const std::string& text) {
    status_ = text;
    refresh(true);
}

void TerminalView::add_event(const std::string& text) {
    last_event_ = text;
    if (events_.size() == 8) {
        events_.pop_front();
    }
    events_.push_back(text);
    refresh(true);
}

void TerminalView::set_input(const std::string& text) {
    input_ = text;
    refresh(true);
}

const std::string& TerminalView::last_event() const { return last_event_; }

void TerminalView::refresh(bool force) {
    CONSOLE_SCREEN_BUFFER_INFO info{};
    if (!active() || !GetConsoleScreenBufferInfo(screen_, &info)) {
        return;
    }
    const SMALL_RECT& window = info.srWindow;
    bool same_window = window.Left == previous_window_.Left && window.Right == previous_window_.Right &&
                       window.Top == previous_window_.Top && window.Bottom == previous_window_.Bottom;
    if (!force && rendered_ && same_window) {
        return;
    }
    int width = window.Right - window.Left + 1;
    int height = window.Bottom - window.Top + 1;
    if (width < 3 || height < 4) {
        return;
    }
    WORD normal = info.wAttributes;
    WORD bright = normal | FOREGROUND_INTENSITY;
    CHAR_INFO blank{};
    blank.Char.AsciiChar = ' ';
    blank.Attributes = normal;
    std::vector<CHAR_INFO> cells(static_cast<std::size_t>(width * height), blank);
    put_row(cells, width, 0, "GCS  |  status / help  |  quit or Ctrl+C to exit", bright);

    int status_rows = std::min(6, height - 4);
    std::vector<std::string> status_lines = wrap_lines(status_, static_cast<std::size_t>(width - 1));
    for (int row = 0; row < status_rows && row < static_cast<int>(status_lines.size()); ++row) {
        put_row(cells, width, row + 1, status_lines[static_cast<std::size_t>(row)], normal);
    }
    int events_start = status_rows + 2;
    put_row(cells, width, events_start - 1, "Recent events", bright);
    std::vector<std::string> event_lines;
    for (const std::string& event : events_) {
        std::vector<std::string> wrapped = wrap_lines(event, static_cast<std::size_t>(width - 1));
        event_lines.insert(event_lines.end(), wrapped.begin(), wrapped.end());
    }
    int available_events = height - 1 - events_start;
    int first_event = std::max(0, static_cast<int>(event_lines.size()) - available_events);
    for (int index = first_event; index < static_cast<int>(event_lines.size()); ++index) {
        put_row(cells, width, events_start + index - first_event,
                event_lines[static_cast<std::size_t>(index)], normal);
    }

    std::string prompt = width >= 10 ? "gcs> " : "> ";
    std::string visible_input = input_;
    std::replace(visible_input.begin(), visible_input.end(), '\t', ' ');
    std::size_t input_width = static_cast<std::size_t>(width - 1) - prompt.size();
    if (visible_input.size() > input_width) {
        visible_input.erase(0, visible_input.size() - input_width);
    }
    prompt += visible_input;
    put_row(cells, width, height - 1, prompt, bright);

    COORD size{static_cast<SHORT>(width), static_cast<SHORT>(height)};
    COORD origin{0, 0};
    SMALL_RECT destination = window;
    if (WriteConsoleOutputA(screen_, cells.data(), size, origin, &destination)) {
        COORD cursor{static_cast<SHORT>(window.Left + prompt.size()), window.Bottom};
        SetConsoleCursorPosition(screen_, cursor);
        previous_window_ = window;
        rendered_ = true;
    }
}

} // namespace drone
