#pragma once

#include "motion_bridge/types.hpp"

#include <array>
#include <chrono>
#include <string>

namespace motion_bridge {

using AxisMask = std::array<bool, 6>;

[[nodiscard]] std::string encode_tcode(const Axes& axes, std::chrono::milliseconds interval);
[[nodiscard]] std::string encode_tcode(const Axes& axes, std::chrono::milliseconds interval, const AxisMask& included_axes);

} // namespace motion_bridge
