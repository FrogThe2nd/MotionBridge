#include "motion_bridge/tcode.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace motion_bridge {

std::string encode_tcode(const Axes& axes, const std::chrono::milliseconds interval) {
    AxisMask included_axes;
    included_axes.fill(true);
    return encode_tcode(axes, interval, included_axes);
}

std::string encode_tcode(
    const Axes& axes,
    const std::chrono::milliseconds interval,
    const AxisMask& included_axes) {
    const auto interval_ms = std::clamp(interval.count(), 1LL, 999LL);
    std::string result;
    for (std::size_t index = 0; index < axes.values.size(); ++index) {
        if (!included_axes[index]) continue;
        const auto value = std::clamp(axes[index], 0.0, 1.0);
        const auto payload = static_cast<int>(std::floor(value * 9999.0 + 0.5));
        if (!result.empty()) result += ' ';
        char command[32]{};
        std::snprintf(command, sizeof(command), "%s%04dI%03lld", kAxisNames[index], payload, interval_ms);
        result += command;
    }
    return result.empty() ? result : result + '\n';
}

} // namespace motion_bridge
