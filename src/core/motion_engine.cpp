#include "motion_bridge/motion_engine.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace motion_bridge {
namespace {

constexpr double kEpsilon = 1e-8;
constexpr double kActivityDirectionThresholdMeters = 0.00075;
constexpr double kActivityMinimumStrokeMeters = 0.005;

[[nodiscard]] Vec3 add(const Vec3 left, const Vec3 right) { return {left.x + right.x, left.y + right.y, left.z + right.z}; }
[[nodiscard]] Vec3 subtract(const Vec3 left, const Vec3 right) { return {left.x - right.x, left.y - right.y, left.z - right.z}; }
[[nodiscard]] Vec3 scale(const Vec3 value, const double scalar) { return {value.x * scalar, value.y * scalar, value.z * scalar}; }
[[nodiscard]] double dot(const Vec3 left, const Vec3 right) { return left.x * right.x + left.y * right.y + left.z * right.z; }
[[nodiscard]] Vec3 cross(const Vec3 left, const Vec3 right) {
    return {left.y * right.z - left.z * right.y, left.z * right.x - left.x * right.z, left.x * right.y - left.y * right.x};
}
[[nodiscard]] double magnitude(const Vec3 value) { return std::sqrt(dot(value, value)); }
[[nodiscard]] std::optional<Vec3> normalize(const Vec3 value) {
    const auto length = magnitude(value);
    return length <= kEpsilon ? std::nullopt : std::optional<Vec3>{scale(value, 1.0 / length)};
}
[[nodiscard]] Vec3 project_on_plane(const Vec3 value, const Vec3 normal) {
    const auto normalized = normalize(normal);
    return normalized ? subtract(value, scale(*normalized, dot(value, *normalized))) : value;
}
[[nodiscard]] Quaternion normalized(const Quaternion value) {
    const auto length = std::sqrt(value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
    return length <= kEpsilon ? Quaternion{} : Quaternion{value.w / length, value.x / length, value.y / length, value.z / length};
}
[[nodiscard]] Quaternion multiply(const Quaternion left, const Quaternion right) {
    return {
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
    };
}
[[nodiscard]] Vec3 rotate(const Quaternion rotation, const Vec3 vector) {
    const auto q = normalized(rotation);
    const auto rotated = multiply(multiply(q, Quaternion{0.0, vector.x, vector.y, vector.z}), Quaternion{q.w, -q.x, -q.y, -q.z});
    return {rotated.x, rotated.y, rotated.z};
}
[[nodiscard]] Vec3 local_axis(const Quaternion rotation, const std::string& name) {
    const auto sign = !name.empty() && name.front() == '-' ? -1.0 : 1.0;
    if (name.ends_with("local_y")) return rotate(rotation, {0.0, sign, 0.0});
    if (name.ends_with("local_z")) return rotate(rotation, {0.0, 0.0, sign});
    return rotate(rotation, {sign, 0.0, 0.0});
}
[[nodiscard]] double signed_angle_degrees(const Vec3 start, const Vec3 end, const Vec3 axis) {
    const auto a = normalize(start); const auto b = normalize(end); const auto n = normalize(axis);
    if (!a || !b || !n) return 0.0;
    return std::atan2(dot(*n, cross(*a, *b)), std::clamp(dot(*a, *b), -1.0, 1.0)) * 180.0 / std::numbers::pi;
}
[[nodiscard]] double range01(const double value, const double minimum, const double maximum) {
    return maximum <= minimum + kEpsilon ? 0.5 : clamp01((value - minimum) / (maximum - minimum));
}
[[nodiscard]] double symmetric01(const double value, const double maximum) {
    return maximum <= kEpsilon ? 0.5 : clamp01(0.5 + value / (2.0 * maximum));
}
[[nodiscard]] const Participant* participant(const MotionFrame& frame, const std::string& key, const std::string& required_bone) {
    if (!key.empty()) {
        const auto found = std::find_if(frame.participants.begin(), frame.participants.end(), [&key](const Participant& item) { return item.stable_key == key; });
        return found == frame.participants.end() ? nullptr : &*found;
    }
    const auto found = std::find_if(frame.participants.begin(), frame.participants.end(), [&required_bone](const Participant& item) {
        return item.bones.contains(required_bone);
    });
    return found == frame.participants.end() ? nullptr : &*found;
}
[[nodiscard]] const BonePose* bone(const Participant* item, const std::string& name) {
    if (item == nullptr) return nullptr;
    const auto found = item->bones.find(name);
    return found == item->bones.end() ? nullptr : &found->second;
}
struct PelvisPlane {
    Vec3 normal;
    Vec3 tangent;
};
[[nodiscard]] std::optional<PelvisPlane> plane_from_landmarks(
    const Participant* item,
    const std::string& center_name,
    const std::string& forward_name,
    const std::string& left_name,
    const std::string& right_name) {
    const auto* center = bone(item, center_name);
    const auto* forward = bone(item, forward_name);
    const auto* left = bone(item, left_name);
    const auto* right = bone(item, right_name);
    if (!center || !forward || !left || !right) return std::nullopt;
    const auto lateral = normalize(subtract(right->position, left->position));
    const auto vertical = normalize(subtract(forward->position, center->position));
    if (!lateral || !vertical) return std::nullopt;
    const auto normal = normalize(cross(*lateral, *vertical));
    if (!normal) return std::nullopt;
    const auto tangent = normalize(project_on_plane(*lateral, *normal));
    if (!tangent) return std::nullopt;
    return PelvisPlane{*normal, *tangent};
}
[[nodiscard]] std::optional<PelvisPlane> reference_plane(const MotionFrame& frame, const Participant* item) {
    if (frame.reference_plane) {
        const auto& plane = *frame.reference_plane;
        if (const auto custom = plane_from_landmarks(item, plane.center_bone, plane.forward_bone, plane.left_bone, plane.right_bone)) {
            return custom;
        }
    }
    return plane_from_landmarks(item, "M_Hips", "M_Spine1", "L_Thigh", "R_Thigh");
}
[[nodiscard]] double shape(const double value, const MotionCurve curve) {
    switch (curve) {
    case MotionCurve::Smoothstep: return value * value * (3.0 - 2.0 * value);
    case MotionCurve::Smootherstep: return value * value * value * (value * (value * 6.0 - 15.0) + 10.0);
    default: return value;
    }
}
[[nodiscard]] double tune_value(double value, const AxisTuning& tuning, const double gain_center) {
    value = clamp01(value);
    const auto center = std::clamp(tuning.center, 0.0, 1.0);
    const auto gain = std::clamp(tuning.gain, 0.25, 4.0);
    const auto dead_zone = std::clamp(tuning.dead_zone, 0.0, 0.4);
    const auto positive = value >= center;
    const auto span = positive ? std::max(1.0 - center, kEpsilon) : std::max(center, kEpsilon);
    auto progress = positive ? (value - center) / span : (center - value) / span;
    progress = std::max(0.0, (progress - dead_zone) / std::max(1.0 - dead_zone, kEpsilon));
    const auto shaped = shape(progress, tuning.curve);
    const auto normalized_value = positive ? center + span * shaped : center - span * shaped;
    const auto lower = std::clamp(std::min(tuning.output_min, tuning.output_max), 0.0, 1.0);
    const auto upper = std::clamp(std::max(tuning.output_min, tuning.output_max), 0.0, 1.0);
    // The geometric signal is already normalised by the game adapter.  Gain
    // must therefore enlarge the *observed motion* around its own neutral
    // value, not around a fixed 0.5.  A fixed midpoint clipped short L0
    // strokes that happened to live entirely in one half of the range.
    const auto center_output = lower + std::clamp(gain_center, 0.0, 1.0) * (upper - lower);
    const auto unscaled_output = lower + normalized_value * (upper - lower);
    // Gain changes the device's excursion around its selected center.  Apply
    // it after dead-zone and curve shaping so it controls travel, not input
    // sensitivity or the shape of the response curve.
    auto output = center_output + (unscaled_output - center_output) * gain;
    output = std::clamp(output, lower, upper);
    if (tuning.inverted) output = lower + upper - output;
    return tuning.enabled ? clamp01(output) : 0.5;
}

[[nodiscard]] double shortest_angle_delta(const double current, const double baseline) {
    // R0 is a relative actuator, not an accumulated turn counter.  A moving
    // reference axis can cross the signed-angle seam during ordinary motion;
    // retain the equivalent turn closest to the baseline so that seam
    // crossings cannot make a toy spin through multiple revolutions.
    return std::remainder(current - baseline, 360.0);
}

} // namespace

double clamp01(const double value) noexcept { return std::clamp(value, 0.0, 1.0); }

MotionEngine::MotionEngine(ContactConfig contact, SafetyConfig safety) : contact_(std::move(contact)), safety_(safety) {}
void MotionEngine::set_contact_config(ContactConfig contact) {
    contact_ = std::move(contact);
    last_valid_.reset();
    angle_binding_key_.clear();
    twist_baseline_.reset();
    roll_baseline_.reset();
    pitch_baseline_.reset();
    gain_binding_key_.clear();
    gain_envelope_valid_.fill(false);
    l0_activity_binding_key_.clear();
    l0_activity_has_sample_ = false;
    l0_activity_ready_at_.reset();
}
void MotionEngine::set_axis_tuning(std::array<AxisTuning, 6> tuning) { tuning_ = std::move(tuning); }
const ContactConfig& MotionEngine::contact_config() const noexcept { return contact_; }
const std::array<AxisTuning, 6>& MotionEngine::axis_tuning() const noexcept { return tuning_; }

std::optional<EngineSnapshot> MotionEngine::calculate(const MotionFrame& frame) {
    const auto* reference = participant(frame, contact_.reference_participant, contact_.origin_bone);
    const auto* target_owner = participant(frame, contact_.target_participant, contact_.target_bone);
    const auto* origin = bone(reference, contact_.origin_bone);
    const auto* direction = bone(reference, contact_.direction_bone);
    const auto* tip = bone(reference, contact_.tip_bone);
    const auto* support = bone(reference, contact_.support_bone);
    const auto* target = bone(target_owner, contact_.target_bone);
    if (!frame.action_active || !origin || !direction || !tip || !support || !target) return std::nullopt;

    const auto axis = normalize(subtract(direction->position, origin->position));
    const auto length = magnitude(subtract(tip->position, origin->position));
    if (!axis || length <= kEpsilon) return std::nullopt;
    const auto body_plane = reference_plane(frame, reference);
    auto reference_right = body_plane
        ? normalize(project_on_plane(body_plane->tangent, *axis))
        : std::optional<Vec3>{};
    if (!reference_right) {
        reference_right = normalize(project_on_plane(local_axis(support->rotation, contact_.support_right_axis), *axis));
    }
    if (!reference_right) reference_right = normalize(project_on_plane(local_axis(support->rotation, contact_.support_up_axis), *axis));
    if (!reference_right) return std::nullopt;
    const auto reference_forward = normalize(cross(*reference_right, *axis));
    if (!reference_forward) return std::nullopt;

    const auto delta = subtract(target->position, origin->position);
    const auto axial = dot(delta, *axis);
    const auto closest = add(origin->position, scale(*axis, std::clamp(axial, 0.0, length)));
    const auto radial = subtract(target->position, closest);
    const auto radial_distance = magnitude(radial);
    const auto radius = length * contact_.radius_scale;
    const auto contact_valid = radial_distance <= radius && axial >= -radius && axial <= length + radius;
    if (contact_.require_contact && !contact_valid) return std::nullopt;

    const auto* secondary_target = contact_.target_secondary_bone.empty()
        ? nullptr
        : bone(target_owner, contact_.target_secondary_bone);
    const auto bilateral = secondary_target != nullptr;
    const auto target_right = bilateral
        ? project_on_plane(subtract(target->position, secondary_target->position), *axis)
        : project_on_plane(local_axis(target->rotation, contact_.target_right_axis), *axis);
    const auto target_up = bilateral ? *axis : local_axis(target->rotation, contact_.target_up_axis);
    if (magnitude(target_right) <= kEpsilon) return std::nullopt;
    const auto wrapped_twist = signed_angle_degrees(*reference_right, target_right, *axis);
    const auto wrapped_roll = -signed_angle_degrees(*axis, project_on_plane(target_up, *reference_forward), *reference_forward);
    const auto wrapped_pitch = signed_angle_degrees(*axis, project_on_plane(target_up, *reference_right), *reference_right);
    const auto binding_key = reference->stable_key + "|" + target_owner->stable_key + "|" + frame.action_id + "|"
        + contact_.target_bone + "|" + contact_.target_secondary_bone + "|" + contact_.target_up_axis + "|" + contact_.target_right_axis;
    const auto binding_changed = binding_key != angle_binding_key_;
    if (binding_changed || !twist_baseline_ || !roll_baseline_ || !pitch_baseline_) {
        twist_baseline_ = wrapped_twist;
        roll_baseline_ = wrapped_roll;
        pitch_baseline_ = wrapped_pitch;
    }
    angle_binding_key_ = binding_key;
    const auto twist = shortest_angle_delta(wrapped_twist, *twist_baseline_);
    // Signed angles are periodic. A physically continuous tilt can cross the
    // -180/+180 representation seam, which previously changed R1/R2 by almost
    // a full revolution in one frame. Like twist, express both tilts as the
    // shortest displacement from this binding's activation orientation.
    const auto roll = shortest_angle_delta(wrapped_roll, *roll_baseline_);
    const auto pitch = shortest_angle_delta(wrapped_pitch, *pitch_baseline_);

    Axes raw;
    const auto absolute_l0 = frame.direct_l0_min_meters && frame.direct_l0_max_meters
        ? range01(axial, *frame.direct_l0_min_meters, *frame.direct_l0_max_meters)
        : (bilateral || frame.l0_reference_length)
            ? range01(axial, 0.0, length)
            : range01(axial, contact_.l0_min_meters, contact_.l0_max_meters);
    raw[0] = absolute_l0;
    if (frame.l0_activity_window) {
        // Nonhuman appendages can be far longer than the part which actually
        // moves in a loop.  Learn the loop's real axial travel instead of
        // treating the full mesh chain as a toy stroke range.
        if (l0_activity_binding_key_ != binding_key) {
            l0_activity_binding_key_ = binding_key;
            l0_activity_min_ = axial;
            l0_activity_max_ = axial;
            l0_activity_last_ = axial;
            l0_activity_direction_ = 0;
            l0_activity_reversals_ = 0;
            l0_activity_has_sample_ = true;
            l0_activity_ready_at_.reset();
        } else if (l0_activity_has_sample_) {
            l0_activity_min_ = std::min(l0_activity_min_, axial);
            l0_activity_max_ = std::max(l0_activity_max_, axial);
            const auto delta = axial - l0_activity_last_;
            const auto direction = delta > kActivityDirectionThresholdMeters ? 1
                : delta < -kActivityDirectionThresholdMeters ? -1 : 0;
            if (direction != 0) {
                if (l0_activity_direction_ != 0 && direction != l0_activity_direction_) {
                    ++l0_activity_reversals_;
                }
                l0_activity_direction_ = direction;
            }
            l0_activity_last_ = axial;
            if (!l0_activity_ready_at_
                && l0_activity_reversals_ >= 2
                && l0_activity_max_ - l0_activity_min_ >= kActivityMinimumStrokeMeters) {
                l0_activity_ready_at_ = frame.monotonic_time;
            }
        }
        if (l0_activity_ready_at_) {
            const auto learned_l0 = range01(axial, l0_activity_min_, l0_activity_max_);
            // One short blend keeps the source's initial absolute reading from
            // becoming a hard jump when a complete in-game cycle is known.
            const auto elapsed = frame.monotonic_time - *l0_activity_ready_at_;
            const auto blend = std::clamp(static_cast<double>(elapsed.count()) / 300000.0, 0.0, 1.0);
            raw[0] = absolute_l0 + (learned_l0 - absolute_l0) * blend;
        }
    }
    // Per-action calibration describes the game skeleton's local direction;
    // the global control remains a user override, so two inversions cancel.
    if (contact_.invert_l0 != frame.direct_l0_inverted) raw[0] = 1.0 - raw[0];
    const auto plane_delta = body_plane ? project_on_plane(delta, body_plane->normal) : radial;
    raw[1] = symmetric01(dot(plane_delta, *reference_forward), contact_.lateral_range_meters);
    raw[2] = symmetric01(dot(plane_delta, *reference_right), contact_.lateral_range_meters);
    raw[3] = symmetric01(twist, contact_.twist_range_degrees);
    raw[4] = symmetric01(roll, contact_.tilt_range_degrees);
    raw[5] = symmetric01(pitch, contact_.tilt_range_degrees);
    for (std::size_t index = 0; index < raw.values.size(); ++index) {
        if (!frame.active_axes[index]) raw[index] = 0.5;
    }
    return EngineSnapshot{frame.sequence, frame.monotonic_time, MotionState::Active, raw, tune(raw, frame.active_axes, binding_key),
        {true, contact_valid, contact_valid ? "ok" : "outside_contact_radius", length, radius, axial, radial_distance, twist, roll, pitch,
            bilateral ? "bilateral_reference_axis" : "single_bone"},
        frame.action_id, frame.action_category};
}

Axes MotionEngine::tune(const Axes& raw, const std::array<bool, 6>& active_axes, const std::string& binding_key) {
    if (gain_binding_key_ != binding_key) {
        gain_binding_key_ = binding_key;
        gain_envelope_valid_.fill(false);
    }
    Axes result;
    for (std::size_t index = 0; index < result.values.size(); ++index) {
        if (active_axes[index]) {
            if (!gain_envelope_valid_[index]) {
                gain_min_[index] = raw[index];
                gain_max_[index] = raw[index];
                gain_envelope_valid_[index] = true;
            } else {
                gain_min_[index] = std::min(gain_min_[index], raw[index]);
                gain_max_[index] = std::max(gain_max_[index], raw[index]);
            }
        }
        const auto gain_center = gain_envelope_valid_[index]
            ? (gain_min_[index] + gain_max_[index]) * 0.5
            : tuning_[index].center;
        result[index] = tune_value(raw[index], tuning_[index], gain_center);
    }
    return result;
}

EngineSnapshot MotionEngine::apply_safety(EngineSnapshot next, const std::chrono::microseconds now) {
    if (next.contact.valid) {
        last_valid_ = next;
        last_valid_time_ = now;
        return next;
    }
    return process_missing(now);
}

EngineSnapshot MotionEngine::process(const MotionFrame& frame) {
    const auto now = frame.monotonic_time;
    if (const auto calculated = calculate(frame)) return apply_safety(*calculated, now);
    return process_missing(now);
}

EngineSnapshot MotionEngine::process_missing(const std::chrono::microseconds now) {
    if (!last_valid_) return {0, now, MotionState::Idle, {}, {}, {false, false, "no_active_contact"}, {}, {}};
    const auto elapsed = now - last_valid_time_;
    if (elapsed <= safety_.hold_for) {
        auto snapshot = *last_valid_; snapshot.monotonic_time = now; snapshot.state = MotionState::Holding; snapshot.contact.valid = false; snapshot.contact.reason = "hold"; return snapshot;
    }
    const auto return_elapsed = elapsed - safety_.hold_for;
    const auto ratio = safety_.return_for.count() <= 0 ? 1.0 : std::clamp(static_cast<double>(return_elapsed.count()) / (safety_.return_for.count() * 1000.0), 0.0, 1.0);
    auto snapshot = *last_valid_;
    snapshot.monotonic_time = now;
    snapshot.state = ratio >= 1.0 ? MotionState::Idle : MotionState::Returning;
    snapshot.contact.valid = false;
    snapshot.contact.reason = ratio >= 1.0 ? "idle" : "returning";
    for (std::size_t index = 0; index < 6; ++index) {
        snapshot.raw_axes[index] = snapshot.raw_axes[index] + (0.5 - snapshot.raw_axes[index]) * ratio;
        snapshot.device_axes[index] = snapshot.device_axes[index] + (0.5 - snapshot.device_axes[index]) * ratio;
    }
    return snapshot;
}

const char* to_string(const MotionState state) noexcept {
    switch (state) {
    case MotionState::Active: return "active";
    case MotionState::Holding: return "holding";
    case MotionState::Returning: return "returning";
    case MotionState::Fault: return "fault";
    default: return "idle";
    }
}

} // namespace motion_bridge
