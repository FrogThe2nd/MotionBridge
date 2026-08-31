#pragma once

#include "motion_bridge/types.hpp"

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace motion_bridge {

class MotionEngine {
public:
    explicit MotionEngine(ContactConfig contact = {}, SafetyConfig safety = {});

    void set_contact_config(ContactConfig contact);
    void set_axis_tuning(std::array<AxisTuning, 6> tuning);
    void set_l0_travel_preference(L0TravelPreferenceConfig config);
    void reset_l0_travel_learning();
    [[nodiscard]] const ContactConfig& contact_config() const noexcept;
    [[nodiscard]] const std::array<AxisTuning, 6>& axis_tuning() const noexcept;
    [[nodiscard]] const L0TravelPreferenceConfig& l0_travel_preference() const noexcept;

    [[nodiscard]] EngineSnapshot process(const MotionFrame& frame);
    [[nodiscard]] EngineSnapshot process_missing(std::chrono::microseconds now);

private:
    [[nodiscard]] std::optional<EngineSnapshot> calculate(const MotionFrame& frame);
    [[nodiscard]] Axes tune(const Axes& raw, const std::array<bool, 6>& active_axes, const std::string& binding_key);
    [[nodiscard]] EngineSnapshot apply_safety(EngineSnapshot next, std::chrono::microseconds now);
    [[nodiscard]] double optimize_l0(double value, const std::string& profile_key, std::chrono::microseconds now);
    void reset_l0_travel_live_state(const std::string& profile_key);
    void record_l0_turning_point(double value, std::chrono::microseconds now);

    struct L0TravelProfile {
        double center{0.5};
        double travel{};
    };

    ContactConfig contact_;
    SafetyConfig safety_;
    std::array<AxisTuning, 6> tuning_{};
    std::optional<EngineSnapshot> last_valid_;
    std::chrono::microseconds last_valid_time_{};
    std::string angle_binding_key_;
    std::optional<double> twist_baseline_;
    std::optional<double> roll_baseline_;
    std::optional<double> pitch_baseline_;
    // Gain expands an animation about its observed neutral position, rather
    // than the arbitrary global 0.5.  The envelope is reset only when the
    // reference/target/action binding changes, so it cannot drift while a
    // loop is playing.
    std::string gain_binding_key_;
    std::array<double, 6> gain_min_{};
    std::array<double, 6> gain_max_{};
    std::array<bool, 6> gain_envelope_valid_{};
    L0TravelPreferenceConfig l0_travel_config_;
    L0TravelStatus l0_travel_status_;
    std::unordered_map<std::string, L0TravelProfile> l0_travel_cache_;
    std::string l0_travel_profile_key_;
    std::optional<L0TravelProfile> l0_travel_profile_;
    std::optional<std::chrono::microseconds> l0_travel_transition_at_;
    bool l0_travel_has_sample_{};
    double l0_travel_anchor_{};
    double l0_travel_extremum_{};
    int l0_travel_direction_{};
    std::optional<double> l0_travel_last_turning_point_;
    std::vector<std::pair<double, double>> l0_travel_half_strokes_;
    std::optional<double> l0_optimized_center_;
};

[[nodiscard]] const char* to_string(MotionState state) noexcept;
[[nodiscard]] const char* to_string(L0TravelState state) noexcept;
[[nodiscard]] double clamp01(double value) noexcept;

} // namespace motion_bridge
