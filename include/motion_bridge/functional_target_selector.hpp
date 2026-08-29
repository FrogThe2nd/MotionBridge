#pragma once

#include "motion_bridge/types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace motion_bridge {

// Keeps an adapter-declared functional target stable for the lifetime of an
// action. Candidate order is priority order; distance is used only to choose
// between participants that expose the same candidate bone.
class FunctionalTargetSelector {
public:
    explicit FunctionalTargetSelector(std::size_t missing_grace_frames = 12);

    [[nodiscard]] bool alias_target(
        MotionFrame& frame,
        const std::vector<std::string>& candidate_bones,
        const std::string& reference_origin_bone = "Penis01",
        const std::string& alias_bone = "M_Gen");
    void reset();

    [[nodiscard]] const std::string& selected_participant() const noexcept;
    [[nodiscard]] const std::string& selected_bone() const noexcept;

private:
    std::size_t missing_grace_frames_{};
    std::size_t missing_frames_{};
    std::string action_id_;
    std::string participant_key_;
    std::string bone_name_;
};

} // namespace motion_bridge
