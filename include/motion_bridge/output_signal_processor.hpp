#pragma once

#include "motion_bridge/types.hpp"

#include <array>
#include <chrono>

namespace motion_bridge {

enum class SignalRemapMode { Value, Speed };

struct AxisSignalRemap {
    bool enabled{};
    SignalRemapMode mode{SignalRemapMode::Value};
    double target_value{0.5};
    // X is this axis' own pre-remap input and Y is the retained-output factor.
    // Outside the two control points the nearest endpoint factor is held.
    double lower_input{};
    double lower_factor{1.0};
    double upper_input{1.0};
    double upper_factor{};
};

struct OutputSignalConfig {
    bool soft_start_enabled{true};
    std::chrono::milliseconds soft_start_for{600};
    std::array<bool, 6> axis_output_enabled{true, true, true, true, true, true};
    std::array<double, 6> axis_safe_value{0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    std::array<AxisSignalRemap, 6> remap;
    std::array<bool, 6> speed_limit_enabled{};
    std::array<double, 6> max_speed_per_second{4.0, 4.0, 4.0, 4.0, 4.0, 4.0};
};

// Converts the tuned device target into the value that is actually sent.
// It deliberately lives after MotionEngine so output protection never changes
// game-space contact calculations, participant routing, gain or range tuning.
class OutputSignalProcessor {
public:
    explicit OutputSignalProcessor(OutputSignalConfig config = {});

    void set_config(OutputSignalConfig config);
    [[nodiscard]] const OutputSignalConfig& config() const noexcept;

    // Every arm begins at each axis' safe position. Soft start begins only
    // after the first live target arrives.
    void arm(std::chrono::microseconds now);
    void disarm();

    [[nodiscard]] Axes process(const Axes& target, std::chrono::microseconds now, bool live_motion);
    [[nodiscard]] const Axes& current() const noexcept;

private:
    OutputSignalConfig config_;
    Axes current_;
    bool armed_{};
    bool live_started_{};
    std::chrono::microseconds live_started_at_{};
    std::chrono::microseconds last_process_at_{};
};

} // namespace motion_bridge
