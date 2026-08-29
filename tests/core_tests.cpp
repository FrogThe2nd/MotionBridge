#include "motion_bridge/functional_target_selector.hpp"
#include "motion_bridge/motion_engine.hpp"
#include "motion_bridge/tcode.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace motion_bridge;

namespace {

BonePose pose(std::string name, Vec3 position, Quaternion rotation = {}) { return {std::move(name), position, rotation}; }

MotionFrame orthogonal_frame(std::chrono::microseconds time = std::chrono::microseconds{0}) {
    Participant reference{"male", "male", "fallen-doll"};
    reference.bones.emplace("Penis01", pose("Penis01", {0, 0, 0}));
    reference.bones.emplace("Penis02", pose("Penis02", {0, 1, 0}));
    reference.bones.emplace("Penis09", pose("Penis09", {0, 1, 0}));
    reference.bones.emplace("M_Hips", pose("M_Hips", {0, 0, 0}));
    Participant target{"female", "female", "fallen-doll"};
    // Maps the Fallen Doll default target basis (-local_y/+local_z) onto the
    // synthetic reference axis (+Y) and right axis (-X).
    target.bones.emplace("M_Gen", pose("M_Gen", {0, 0.5, 0}, {0, 0.7071067811865476, 0, -0.7071067811865476}));
    return {"motion-frame/v1", "fallen-doll", 1, time, true, "Test", "vaginal", {reference, target}};
}

void require_close(const double actual, const double expected) { assert(std::abs(actual - expected) < 1e-6); }

void test_contact_and_tcode() {
    MotionEngine engine;
    auto frame = orthogonal_frame();
    auto snapshot = engine.process(frame);
    assert(snapshot.state == MotionState::Active);
    require_close(snapshot.raw_axes[0], 1.0);
    require_close(snapshot.raw_axes[1], 0.5);
    require_close(snapshot.raw_axes[2], 0.5);
    assert(encode_tcode(snapshot.device_axes, std::chrono::milliseconds{20}) == "L09999I020 L15000I020 L25000I020 R05000I020 R15000I020 R25000I020\n");
}

void test_gain_scales_output_travel() {
    MotionEngine engine;
    auto tuning = engine.axis_tuning();
    tuning[0] = {.gain = 2.0};
    engine.set_axis_tuning(tuning);
    auto frame = orthogonal_frame();
    frame.l0_reference_length = true;
    // An action may occupy only one side of the global 0..1 L0 coordinate.
    // Gain must expand its observed travel about that action's neutral point,
    // not clamp both samples against a fixed 0.5 midpoint.
    frame.participants[1].bones["M_Gen"].position.y = 0.2;
    require_close(engine.process(frame).device_axes[0], 0.2);
    frame.participants[1].bones["M_Gen"].position.y = 0.4;
    const auto snapshot = engine.process(frame);
    require_close(snapshot.raw_axes[0], 0.4);
    require_close(snapshot.device_axes[0], 0.5);
}

void test_hold_and_return() {
    MotionEngine engine;
    const auto initial = engine.process(orthogonal_frame(std::chrono::milliseconds{1000}));
    assert(initial.state == MotionState::Active);
    assert(engine.process_missing(std::chrono::milliseconds{1200}).state == MotionState::Holding);
    const auto returning = engine.process_missing(std::chrono::milliseconds{1550});
    assert(returning.state == MotionState::Returning);
    require_close(returning.device_axes[0], 0.75);
    const auto idle = engine.process_missing(std::chrono::milliseconds{1900});
    assert(idle.state == MotionState::Idle);
    require_close(idle.device_axes[0], 0.5);
}

void test_bilateral_contact_uses_reference_depth() {
    MotionEngine engine;
    auto config = engine.contact_config();
    config.target_bone = "R_Foot";
    config.target_secondary_bone = "L_Foot";
    engine.set_contact_config(config);
    auto frame = orthogonal_frame();
    frame.participants[1].bones.erase("M_Gen");
    frame.participants[1].bones.emplace("R_Foot", pose("R_Foot", {0.08, 0.4, 0}));
    frame.participants[1].bones.emplace("L_Foot", pose("L_Foot", {-0.08, 0.4, 0}));
    const auto snapshot = engine.process(frame);
    assert(snapshot.contact.target_mode == "bilateral_reference_axis");
    require_close(snapshot.raw_axes[0], 0.4);
    require_close(snapshot.raw_axes[4], 0.5);
    require_close(snapshot.raw_axes[5], 0.5);
}

void test_direct_profile_uses_reference_length_and_axis_mask() {
    MotionEngine engine;
    auto frame = orthogonal_frame();
    frame.l0_reference_length = true;
    frame.active_axes = {true, false, false, false, false, false};
    frame.participants[0].bones["Penis09"].position.y = 2.0;
    frame.participants[1].bones["M_Gen"].position.y = 0.5;
    const auto snapshot = engine.process(frame);
    require_close(snapshot.raw_axes[0], 0.25);
    for (std::size_t index = 1; index < 6; ++index) require_close(snapshot.raw_axes[index], 0.5);
}

void test_direct_profile_can_calibrate_signed_l0_range() {
    MotionEngine engine;
    auto frame = orthogonal_frame();
    frame.l0_reference_length = true;
    frame.direct_l0_min_meters = -0.20;
    frame.direct_l0_max_meters = 0.80;
    frame.participants[1].bones["M_Gen"].position.y = 0.30;
    const auto snapshot = engine.process(frame);
    require_close(snapshot.raw_axes[0], 0.5);
}

void test_direct_profile_can_invert_l0_without_flipping_global_setting() {
    MotionEngine engine;
    auto frame = orthogonal_frame();
    frame.l0_reference_length = true;
    frame.direct_l0_min_meters = 0.0;
    frame.direct_l0_max_meters = 1.0;
    frame.direct_l0_inverted = true;
    frame.participants[1].bones["M_Gen"].position.y = 0.25;
    require_close(engine.process(frame).raw_axes[0], 0.75);
}

void test_nonhuman_activity_window_uses_observed_axial_travel() {
    MotionEngine engine;
    auto frame = orthogonal_frame();
    frame.action_id = "NonhumanLoop";
    frame.l0_reference_length = true;
    frame.l0_activity_window = true;

    // The complete reference chain is 1 m in this synthetic frame, but the
    // action only moves through 60 cm. Its output must follow that activity,
    // rather than occupying only 60% because of the chain's total length.
    frame.participants[1].bones["M_Gen"].position.y = 0.2;
    frame.monotonic_time = std::chrono::milliseconds{0};
    (void)engine.process(frame);
    frame.participants[1].bones["M_Gen"].position.y = 0.8;
    frame.monotonic_time = std::chrono::milliseconds{100};
    (void)engine.process(frame);
    frame.participants[1].bones["M_Gen"].position.y = 0.2;
    frame.monotonic_time = std::chrono::milliseconds{200};
    (void)engine.process(frame);
    frame.participants[1].bones["M_Gen"].position.y = 0.8;
    frame.monotonic_time = std::chrono::milliseconds{300};
    (void)engine.process(frame);
    frame.participants[1].bones["M_Gen"].position.y = 0.2;
    frame.monotonic_time = std::chrono::milliseconds{650};
    const auto low = engine.process(frame);
    require_close(low.raw_axes[0], 0.0);
    frame.participants[1].bones["M_Gen"].position.y = 0.5;
    frame.monotonic_time = std::chrono::milliseconds{750};
    const auto middle = engine.process(frame);
    require_close(middle.raw_axes[0], 0.5);
}

void test_humanoid_pelvis_plane_overrides_single_support_rotation() {
    MotionEngine engine;
    auto frame = orthogonal_frame();
    auto& reference = frame.participants[0];
    reference.bones.emplace("M_Spine1", pose("M_Spine1", {0, 1, 0}));
    reference.bones.emplace("L_Thigh", pose("L_Thigh", {-1, 0, 0}));
    reference.bones.emplace("R_Thigh", pose("R_Thigh", {1, 0, 0}));
    frame.participants[1].bones["M_Gen"].position = {0.10, 0.5, 0};
    const auto snapshot = engine.process(frame);
    // The default support mapping uses -local X. The validated pelvis plane
    // uses the named left/right landmarks, so positive world X is L2-positive.
    assert(snapshot.raw_axes[2] > 0.5);
}

void test_profile_plane_uses_native_nonhuman_landmarks() {
    MotionEngine engine;
    auto frame = orthogonal_frame();
    auto& reference = frame.participants[0];
    reference.bones.emplace("Root_M", pose("Root_M", {0, 0, 0}));
    reference.bones.emplace("Chest_M", pose("Chest_M", {0, 1, 0}));
    reference.bones.emplace("Hip_L", pose("Hip_L", {-1, 0, 0}));
    reference.bones.emplace("Hip_R", pose("Hip_R", {1, 0, 0}));
    frame.reference_plane = BodyReferencePlane{"quadruped_trunk", "Root_M", "Chest_M", "Hip_L", "Hip_R"};
    frame.participants[1].bones["M_Gen"].position = {0.10, 0.5, 0};
    const auto snapshot = engine.process(frame);
    assert(snapshot.raw_axes[2] > 0.5);
}

void test_twist_remains_relative_when_reference_crosses_a_turn() {
    MotionEngine engine;
    auto frame = orthogonal_frame();
    // The target's local Z axis travels through more than one signed-angle
    // revolution. R0 must stay the shortest displacement from its activation
    // baseline, rather than becoming a continuously increasing turn counter.
    frame.participants[1].bones["M_Gen"].rotation = {1, 0, 0, 0};
    (void)engine.process(frame);
    frame.participants[1].bones["M_Gen"].rotation = {-0.1736481776669303, 0, 0.984807753012208, 0};
    (void)engine.process(frame);
    frame.participants[1].bones["M_Gen"].rotation = {-0.9396926207859084, 0, -0.3420201433256687, 0};
    const auto snapshot = engine.process(frame);
    assert(std::abs(snapshot.contact.twist_degrees) <= 180.0 + 1e-6);
}

void test_functional_target_priority_stays_locked_during_an_action() {
    FunctionalTargetSelector selector;
    auto frame = orthogonal_frame();
    frame.action_id = "HandAction";
    auto& target = frame.participants[1];
    target.bones.erase("M_Gen");
    target.bones.emplace("R_Hand", pose("R_Hand", {0.2, 0.6, 0}));
    target.bones.emplace("L_Hand", pose("L_Hand", {0.0, 0.4, 0}));

    assert(selector.alias_target(frame, {"R_Hand", "L_Hand"}));
    require_close(frame.participants[1].bones.at("M_Gen").position.x, 0.2);
    assert(selector.selected_bone() == "R_Hand");

    // Even when the other hand becomes much closer, the selected functional
    // target must remain stable until the action or explicit priority changes.
    frame.participants[1].bones["R_Hand"].position = {1.0, 1.0, 0};
    frame.participants[1].bones["L_Hand"].position = {0.0, 0.1, 0};
    assert(selector.alias_target(frame, {"R_Hand", "L_Hand"}));
    require_close(frame.participants[1].bones.at("M_Gen").position.x, 1.0);
    assert(selector.selected_bone() == "R_Hand");
}

void test_functional_target_releases_only_after_missing_grace() {
    FunctionalTargetSelector selector{2};
    auto frame = orthogonal_frame();
    frame.action_id = "HandAction";
    auto& target = frame.participants[1];
    target.bones.erase("M_Gen");
    target.bones.emplace("R_Hand", pose("R_Hand", {0.2, 0.6, 0}));
    target.bones.emplace("L_Hand", pose("L_Hand", {0.0, 0.4, 0}));
    assert(selector.alias_target(frame, {"R_Hand", "L_Hand"}));

    target.bones.erase("R_Hand");
    target.bones.erase("M_Gen");
    assert(!selector.alias_target(frame, {"R_Hand", "L_Hand"}));
    assert(!selector.alias_target(frame, {"R_Hand", "L_Hand"}));
    assert(selector.alias_target(frame, {"R_Hand", "L_Hand"}));
    assert(selector.selected_bone() == "L_Hand");
}

} // namespace

int main() {
    test_contact_and_tcode();
    test_gain_scales_output_travel();
    test_hold_and_return();
    test_bilateral_contact_uses_reference_depth();
    test_direct_profile_uses_reference_length_and_axis_mask();
    test_direct_profile_can_calibrate_signed_l0_range();
    test_direct_profile_can_invert_l0_without_flipping_global_setting();
    test_nonhuman_activity_window_uses_observed_axial_travel();
    test_humanoid_pelvis_plane_overrides_single_support_rotation();
    test_profile_plane_uses_native_nonhuman_landmarks();
    test_twist_remains_relative_when_reference_crosses_a_turn();
    test_functional_target_priority_stays_locked_during_an_action();
    test_functional_target_releases_only_after_missing_grace();
    std::cout << "motion_bridge_core_tests: OK\n";
}
