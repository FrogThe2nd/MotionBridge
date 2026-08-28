#pragma once

#include "motion_bridge/types.hpp"

#include <chrono>
#include <optional>

namespace motion_bridge {

class MotionEngine {
public:
    explicit MotionEngine(ContactConfig contact = {}, SafetyConfig safety = {});

    void set_contact_config(ContactConfig contact);
    void set_axis_tuning(std::array<AxisTuning, 6> tuning);
    [[nodiscard]] const ContactConfig& contact_config() const noexcept;
    [[nodiscard]] const std::array<AxisTuning, 6>& axis_tuning() const noexcept;

    [[nodiscard]] EngineSnapshot process(const MotionFrame& frame);
    [[nodiscard]] EngineSnapshot process_missing(std::chrono::microseconds now);

private:
    [[nodiscard]] std::optional<EngineSnapshot> calculate(const MotionFrame& frame);
    [[nodiscard]] Axes tune(const Axes& raw, const std::array<bool, 6>& active_axes, const std::string& binding_key);
    [[nodiscard]] EngineSnapshot apply_safety(EngineSnapshot next, std::chrono::microseconds now);

    ContactConfig contact_;
    SafetyConfig safety_;
    std::array<AxisTuning, 6> tuning_{};
    std::optional<EngineSnapshot> last_valid_;
    std::chrono::microseconds last_valid_time_{};
    std::string angle_binding_key_;
    std::optional<double> twist_baseline_;
    // Gain expands an animation about its observed neutral position, rather
    // than the arbitrary global 0.5.  The envelope is reset only when the
    // reference/target/action binding changes, so it cannot drift while a
    // loop is playing.
    std::string gain_binding_key_;
    std::array<double, 6> gain_min_{};
    std::array<double, 6> gain_max_{};
    std::array<bool, 6> gain_envelope_valid_{};
    std::string l0_activity_binding_key_;
    double l0_activity_min_{};
    double l0_activity_max_{};
    double l0_activity_last_{};
    int l0_activity_direction_{};
    unsigned int l0_activity_reversals_{};
    bool l0_activity_has_sample_{};
    std::optional<std::chrono::microseconds> l0_activity_ready_at_;
};

[[nodiscard]] const char* to_string(MotionState state) noexcept;
[[nodiscard]] double clamp01(double value) noexcept;

} // namespace motion_bridge
