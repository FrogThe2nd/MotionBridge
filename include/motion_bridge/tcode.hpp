#pragma once

#include "motion_bridge/types.hpp"

#include <chrono>
#include <string>

namespace motion_bridge {

[[nodiscard]] std::string encode_tcode(const Axes& axes, std::chrono::milliseconds interval);

} // namespace motion_bridge
