#pragma once
#include "common/options.h"
#include "common/console.h"
#include <atomic>

namespace drone {
void run_gcs(const Options& options, CommandQueue& commands, Console& console, std::atomic<bool>& stop);
}
