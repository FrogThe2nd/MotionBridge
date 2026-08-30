#include "motion_bridge/functional_target_selector.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace motion_bridge {
namespace {

[[nodiscard]] double distance_squared(const Vec3 left, const Vec3 right) {
    const auto x = left.x - right.x;
    const auto y = left.y - right.y;
    const auto z = left.z - right.z;
    return x * x + y * y + z * z;
}

[[nodiscard]] const BonePose* find_bone(const Participant& participant, const std::string& name) {
    const auto found = participant.bones.find(name);
    return found == participant.bones.end() ? nullptr : &found->second;
}

void insert_alias(Participant& participant, const BonePose& source, const std::string& alias_name) {
    auto alias = source;
    alias.name = alias_name;
    participant.bones.insert_or_assign(alias_name, std::move(alias));
}

void select_contact_frame(MotionFrame& frame, const Participant& participant, const std::string& source_bone) {
    frame.target_frame.reset();
    const auto found = std::find_if(participant.target_frames.begin(), participant.target_frames.end(),
        [&source_bone](const TargetContactFrame& item) { return item.source_bone == source_bone; });
    if (found != participant.target_frames.end()) frame.target_frame = *found;
}

} // namespace

FunctionalTargetSelector::FunctionalTargetSelector(const std::size_t missing_grace_frames)
    : missing_grace_frames_(missing_grace_frames) {}

bool FunctionalTargetSelector::alias_target(
    MotionFrame& frame,
    const std::vector<std::string>& candidate_bones,
    const std::string& reference_origin_bone,
    const std::string& alias_bone,
    const std::string& preferred_reference_participant,
    const std::unordered_map<std::string, std::vector<std::string>>& candidate_bones_by_reference) {
    frame.target_frame.reset();
    if (!frame.action_active || frame.action_id.empty() || candidate_bones.empty()) {
        reset();
        return false;
    }
    if (action_id_ != frame.action_id) {
        reset();
        action_id_ = frame.action_id;
    }

    Participant* reference = nullptr;
    const BonePose* origin = nullptr;
    for (auto& participant : frame.participants) {
        if (!preferred_reference_participant.empty() && participant.stable_key != preferred_reference_participant) continue;
        if (const auto* candidate = find_bone(participant, reference_origin_bone)) {
            reference = &participant;
            origin = candidate;
            break;
        }
    }
    if (reference == nullptr || origin == nullptr) return false;

    const auto exact_candidates = candidate_bones_by_reference.find(reference->stable_key);
    const auto& active_candidates = exact_candidates != candidate_bones_by_reference.end()
        && !exact_candidates->second.empty()
        ? exact_candidates->second
        : candidate_bones;

    const auto selected_is_still_allowed = std::find(active_candidates.begin(), active_candidates.end(), bone_name_)
        != active_candidates.end();
    if (!participant_key_.empty() && !bone_name_.empty() && selected_is_still_allowed) {
        for (auto& participant : frame.participants) {
            if (participant.stable_key != participant_key_) continue;
            if (const auto* selected = find_bone(participant, bone_name_)) {
                insert_alias(participant, *selected, alias_bone);
                select_contact_frame(frame, participant, bone_name_);
                missing_frames_ = 0;
                return true;
            }
            break;
        }
        // A single incomplete frame must not make the bridge jump to another
        // hand, foot or participant. The realtime safety path holds output
        // during this short grace period.
        if (++missing_frames_ <= missing_grace_frames_) return false;
        participant_key_.clear();
        bone_name_.clear();
        missing_frames_ = 0;
    }

    // The game adapter sends functional bones in user-visible priority order
    // (for example R_Hand before L_Hand). Honour that order. Distance only
    // resolves the same functional bone across multiple participants.
    for (const auto& candidate_name : active_candidates) {
        Participant* selected_owner = nullptr;
        const BonePose* selected = nullptr;
        auto best_distance = std::numeric_limits<double>::infinity();
        for (auto& participant : frame.participants) {
            if (&participant == reference) continue;
            const auto* candidate = find_bone(participant, candidate_name);
            if (candidate == nullptr) continue;
            const auto current_distance = distance_squared(origin->position, candidate->position);
            if (current_distance < best_distance) {
                best_distance = current_distance;
                selected_owner = &participant;
                selected = candidate;
            }
        }
        if (selected_owner == nullptr || selected == nullptr) continue;
        participant_key_ = selected_owner->stable_key;
        bone_name_ = candidate_name;
        insert_alias(*selected_owner, *selected, alias_bone);
        select_contact_frame(frame, *selected_owner, candidate_name);
        return true;
    }
    return false;
}

void FunctionalTargetSelector::reset() {
    missing_frames_ = 0;
    action_id_.clear();
    participant_key_.clear();
    bone_name_.clear();
}

const std::string& FunctionalTargetSelector::selected_participant() const noexcept { return participant_key_; }
const std::string& FunctionalTargetSelector::selected_bone() const noexcept { return bone_name_; }

} // namespace motion_bridge
