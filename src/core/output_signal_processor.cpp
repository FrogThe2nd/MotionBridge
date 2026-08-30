#include "motion_bridge/output_signal_processor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace motion_bridge {
namespace {

constexpr double kMinimumRemapInputSeparation = 0.01;

[[nodiscard]] double smoothstep(const double value) {
    const auto clamped = std::clamp(value, 0.0, 1.0);
    return clamped * clamped * (3.0 - 2.0 * clamped);
}

[[nodiscard]] double seconds_between(
    const std::chrono::microseconds later,
    const std::chrono::microseconds earlier) {
    return std::max(0.0, static_cast<double>((later - earlier).count()) / 1'000'000.0);
}

void normalize_remap_curve(AxisSignalRemap& remap) {
    remap.lower_input = std::clamp(remap.lower_input, 0.0, 1.0);
    remap.lower_factor = std::clamp(remap.lower_factor, 0.0, 1.0);
    remap.upper_input = std::clamp(remap.upper_input, 0.0, 1.0);
    remap.upper_factor = std::clamp(remap.upper_factor, 0.0, 1.0);

    if (remap.lower_input > remap.upper_input) {
        std::swap(remap.lower_input, remap.upper_input);
        std::swap(remap.lower_factor, remap.upper_factor);
    }

    if (remap.upper_input - remap.lower_input < kMinimumRemapInputSeparation) {
        const auto midpoint = (remap.lower_input + remap.upper_input) * 0.5;
        remap.lower_input = std::clamp(midpoint - kMinimumRemapInputSeparation * 0.5,
                                       0.0,
                                       1.0 - kMinimumRemapInputSeparation);
        remap.upper_input = remap.lower_input + kMinimumRemapInputSeparation;
    }
}

[[nodiscard]] double remap_factor(const AxisSignalRemap& remap, const double input) {
    if (input <= remap.lower_input) return remap.lower_factor;
    if (input >= remap.upper_input) return remap.upper_factor;
    const auto position = (input - remap.lower_input) / (remap.upper_input - remap.lower_input);
    return remap.lower_factor + (remap.upper_factor - remap.lower_factor) * position;
}

} // namespace

OutputSignalProcessor::OutputSignalProcessor(OutputSignalConfig config) { set_config(std::move(config)); }

void OutputSignalProcessor::set_config(OutputSignalConfig config) {
    config.soft_start_for = std::clamp(config.soft_start_for, std::chrono::milliseconds{0}, std::chrono::milliseconds{3000});
    for (std::size_t index = 0; index < config.max_speed_per_second.size(); ++index) {
        config.axis_safe_value[index] = std::clamp(config.axis_safe_value[index], 0.0, 1.0);
        config.max_speed_per_second[index] = std::clamp(config.max_speed_per_second[index], 0.25, 10.0);
        auto& remap = config.remap[index];
        remap.target_value = std::clamp(remap.target_value, 0.0, 1.0);
        normalize_remap_curve(remap);
    }
    config_ = std::move(config);
}

const OutputSignalConfig& OutputSignalProcessor::config() const noexcept { return config_; }

void OutputSignalProcessor::arm(const std::chrono::microseconds now) {
    for (std::size_t index = 0; index < current_.values.size(); ++index) {
        current_[index] = config_.axis_safe_value[index];
    }
    armed_ = true;
    live_started_ = false;
    live_started_at_ = {};
    last_process_at_ = now;
}

void OutputSignalProcessor::disarm() {
    for (std::size_t index = 0; index < current_.values.size(); ++index) {
        current_[index] = config_.axis_safe_value[index];
    }
    armed_ = false;
    live_started_ = false;
    live_started_at_ = {};
    last_process_at_ = {};
}

Axes OutputSignalProcessor::process(
    const Axes& target,
    const std::chrono::microseconds now,
    const bool live_motion) {
    if (!armed_) return target;

    // Before the first usable frame the hardware remains centred. Once live
    // motion has started, Holding is also treated as live by the caller.
    if (!live_started_) {
        if (!live_motion) {
            last_process_at_ = now;
            return current_;
        }
        live_started_ = true;
        live_started_at_ = now;
        last_process_at_ = now;
    }

    // Engine safety owns stream-loss returning. Do not let a user speed limit
    // stretch its established 600 ms return or interfere with emergency stop.
    if (!live_motion) {
        current_ = target;
        for (std::size_t index = 0; index < current_.values.size(); ++index) {
            if (!config_.axis_output_enabled[index]) current_[index] = config_.axis_safe_value[index];
        }
        last_process_at_ = now;
        return current_;
    }

    auto candidate = target;
    if (config_.soft_start_enabled && config_.soft_start_for.count() > 0) {
        const auto elapsed_ms = static_cast<double>((now - live_started_at_).count()) / 1000.0;
        const auto progress = smoothstep(elapsed_ms / static_cast<double>(config_.soft_start_for.count()));
        for (std::size_t index = 0; index < candidate.values.size(); ++index) {
            const auto safe_value = config_.axis_safe_value[index];
            candidate[index] = safe_value + (target[index] - safe_value) * progress;
        }
    }

    // Every axis reads its own value from one pre-remap snapshot, so applying
    // one curve cannot influence another axis in the same output tick.
    const auto remap_inputs = candidate;
    for (std::size_t index = 0; index < candidate.values.size(); ++index) {
        // A disabled axis is still emitted at its configured safe position.
        // Omitting the command would leave hardware holding the last value.
        if (!config_.axis_output_enabled[index]) {
            candidate[index] = config_.axis_safe_value[index];
            continue;
        }
        const auto& remap = config_.remap[index];
        if (!remap.enabled) continue;
        const auto input = std::clamp(remap_inputs[index], 0.0, 1.0);
        const auto factor = std::clamp(remap_factor(remap, input), 0.0, 1.0);
        if (remap.mode == SignalRemapMode::Value) {
            candidate[index] = remap.target_value + (candidate[index] - remap.target_value) * factor;
        } else {
            candidate[index] = current_[index] + (candidate[index] - current_[index]) * std::pow(factor, 4.0);
        }
    }

    // A delayed event loop must not turn one late tick into an unlimited jump.
    // 100 ms still lets enabled axes catch up without freezing.
    const auto elapsed = std::min(seconds_between(now, last_process_at_), 0.1);
    for (std::size_t index = 0; index < candidate.values.size(); ++index) {
        if (config_.speed_limit_enabled[index]) {
            const auto maximum_delta = config_.max_speed_per_second[index] * elapsed;
            candidate[index] = std::clamp(candidate[index], current_[index] - maximum_delta, current_[index] + maximum_delta);
        }
    }

    for (auto& value : candidate.values) value = std::clamp(value, 0.0, 1.0);
    current_ = candidate;
    last_process_at_ = now;
    return current_;
}

const Axes& OutputSignalProcessor::current() const noexcept { return current_; }

} // namespace motion_bridge
