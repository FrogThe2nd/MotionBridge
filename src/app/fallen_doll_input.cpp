#include "fallen_doll_input.hpp"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using namespace motion_bridge;

namespace {

constexpr qint64 kFreshFrameTimeoutMs = 1000;

std::optional<double> number(const QJsonValue& value) {
    return value.isDouble() ? std::optional<double>{value.toDouble()} : std::nullopt;
}

std::optional<Vec3> vec3(const QJsonValue& value) {
    const auto values = value.toArray();
    if (values.size() != 3) return std::nullopt;
    const auto x = number(values[0]); const auto y = number(values[1]); const auto z = number(values[2]);
    if (!x || !y || !z) return std::nullopt;
    return Vec3{*x, *y, *z};
}

std::optional<Quaternion> quaternion(const QJsonValue& value) {
    const auto values = value.toArray();
    if (values.size() != 4) return std::nullopt;
    const auto w = number(values[0]); const auto x = number(values[1]); const auto y = number(values[2]); const auto z = number(values[3]);
    if (!w || !x || !y || !z) return std::nullopt;
    return Quaternion{*w, *x, *y, *z};
}

Vec3 add_scaled(const Vec3 value, const Vec3 direction, const double scale) {
    return {value.x + direction.x * scale, value.y + direction.y * scale, value.z + direction.z * scale};
}

Vec3 rotate(const Quaternion value, const Vec3 vector) {
    const auto magnitude = std::sqrt(value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
    if (magnitude <= 1e-9) return vector;
    const auto w = value.w / magnitude;
    const auto x = value.x / magnitude;
    const auto y = value.y / magnitude;
    const auto z = value.z / magnitude;
    const auto tx = 2.0 * (y * vector.z - z * vector.y);
    const auto ty = 2.0 * (z * vector.x - x * vector.z);
    const auto tz = 2.0 * (x * vector.y - y * vector.x);
    return {
        vector.x + w * tx + (y * tz - z * ty),
        vector.y + w * ty + (z * tx - x * tz),
        vector.z + w * tz + (x * ty - y * tx),
    };
}

std::vector<std::string> contact_target_candidates(const QStringList& confirmed_bones) {
    // `contactBones` is emitted only by the Lua target contract, which was
    // resolved from the edition-specific unpacked skeleton catalog. No
    // game-specific name guessing is allowed in this bridge layer.
    std::vector<std::string> candidates;
    for (const auto& value : confirmed_bones) {
        const auto name = value.toStdString();
        if (!name.empty() && std::find(candidates.begin(), candidates.end(), name) == candidates.end()) {
            candidates.push_back(name);
        }
    }
    return candidates;
}

} // namespace

FallenDollInput::FallenDollInput(QObject* parent) : QObject(parent) {
    watcher_ = new QFileSystemWatcher(this);
    stream_file_ = new QFile(this);
    poll_timer_ = new QTimer(this);
    poll_timer_->setInterval(20);
    coalesce_timer_ = new QTimer(this);
    coalesce_timer_->setSingleShot(true);
    coalesce_timer_->setInterval(1);
    connect(watcher_, &QFileSystemWatcher::fileChanged, this, &FallenDollInput::read_appended);
    connect(watcher_, &QFileSystemWatcher::directoryChanged, this, &FallenDollInput::read_appended);
    connect(poll_timer_, &QTimer::timeout, this, &FallenDollInput::read_appended);
    connect(coalesce_timer_, &QTimer::timeout, this, &FallenDollInput::emit_pending_frame);
}

void FallenDollInput::set_spool_path(const QString& path) {
    if (spool_path_ == path) return;
    if (!watcher_->files().isEmpty()) watcher_->removePaths(watcher_->files());
    if (!watcher_->directories().isEmpty()) watcher_->removePaths(watcher_->directories());
    spool_path_ = QDir::cleanPath(path);
    if (stream_file_->isOpen()) stream_file_->close();
    stream_file_->setFileName(spool_path_);
    offset_ = 0;
    partial_line_.clear();
    target_selector_.reset();
    initial_sync_pending_ = true;
    valid_frame_seen_ = false;
    valid_frame_timer_.invalidate();
    connection_known_ = false;
    watch_current_path();
}

QString FallenDollInput::spool_path() const { return spool_path_; }

void FallenDollInput::start() {
    watch_current_path();
    if (initial_sync_pending_) {
        const QFileInfo info(spool_path_);
        if (info.exists()) {
            // An existing spool may contain many minutes of history. Real-time
            // startup begins at its current end; only newly appended frames are
            // useful and this prevents thousands of stale UI updates at launch.
            offset_ = info.size();
            partial_line_.clear();
            initial_sync_pending_ = false;
        }
    }
    read_appended();
    if (!poll_timer_->isActive()) poll_timer_->start();
}

void FallenDollInput::watch_current_path() {
    if (spool_path_.isEmpty()) return;
    const QFileInfo info(spool_path_);
    const auto directory = info.absolutePath();
    if (QFileInfo::exists(directory) && !watcher_->directories().contains(directory)) watcher_->addPath(directory);
    if (info.exists() && !watcher_->files().contains(spool_path_)) watcher_->addPath(spool_path_);
}

void FallenDollInput::read_appended() {
    watch_current_path();
    if (!QFileInfo::exists(spool_path_)) {
        if (stream_file_->isOpen()) stream_file_->close();
        publish_connection(false, tr("Waiting for Fallen Doll bone stream"));
        return;
    }
    if (!stream_file_->isOpen()) {
        stream_file_->setFileName(spool_path_);
        if (!stream_file_->open(QIODevice::ReadOnly)) {
            publish_connection(false, tr("Cannot read bone stream"));
            return;
        }
    }
    if (stream_file_->size() < offset_) {
        offset_ = 0;
        partial_line_.clear();
    }
    if (!stream_file_->seek(offset_)) return;
    partial_line_ += stream_file_->readAll();
    offset_ = stream_file_->pos();
    while (true) {
        const auto newline = partial_line_.indexOf('\n');
        if (newline < 0) break;
        const auto line = partial_line_.left(newline);
        partial_line_.remove(0, newline + 1);
        consume_line(line);
    }
    publish_freshness();
}

void FallenDollInput::set_reference_participant(const QString& reference) {
    const auto next_reference = reference.trimmed().toStdString();
    if (preferred_reference_participant_ == next_reference) return;
    preferred_reference_participant_ = next_reference;
    target_selector_.reset();
}

void FallenDollInput::record_valid_frame() {
    valid_frame_timer_.restart();
    valid_frame_seen_ = true;
    publish_connection(true, tr("Fallen Doll stream connected"));
}

void FallenDollInput::publish_freshness() {
    const auto fresh = valid_frame_seen_ && valid_frame_timer_.isValid()
        && valid_frame_timer_.elapsed() <= kFreshFrameTimeoutMs;
    publish_connection(fresh, fresh ? tr("Fallen Doll stream connected")
                                   : tr("Waiting for fresh Fallen Doll bone frames"));
}

void FallenDollInput::publish_connection(const bool connected, const QString& detail) {
    if (connection_known_ && connected_ == connected && connection_detail_ == detail) return;
    connection_known_ = true;
    connected_ = connected;
    connection_detail_ = detail;
    emit connection_changed(connected, detail);
}

void FallenDollInput::consume_line(const QByteArray& line) {
    const auto document = QJsonDocument::fromJson(line);
    if (!document.isObject()) return;
    const auto packet = document.object();
    if (packet.value("schema").toString() == u"motion-frame/v1") {
        consume_motion_frame(packet);
        return;
    }
    if (packet.value("type").toString() != u"skeleton_binary") return;
    const auto timestamp = packet.value("timestampMs").toVariant().toLongLong();
    if (timestamp <= 0) return;
    const auto trailer = packet.value("trailer").toObject();
    if (pending_timestamp_ >= 0 && pending_timestamp_ != timestamp) emit_pending_frame();
    if (pending_timestamp_ < 0) {
        pending_timestamp_ = timestamp;
        pending_frame_ = {};
        pending_frame_.game_id = "fallen-doll";
        pending_frame_.schema = "motion-frame/v1";
        pending_frame_.sequence = ++sequence_;
        pending_frame_.monotonic_time = std::chrono::milliseconds{timestamp};
        pending_frame_.action_active = trailer.value("hanimeActive").toBool();
        pending_frame_.action_id = trailer.value("hanimeId").toString().toStdString();
        pending_frame_.action_category = trailer.value("hanimeCategory").toString().toStdString();
        pending_confirmed_target_bones_.clear();
        pending_participant_slots_.clear();
        pending_target_bones_by_slot_.clear();
    }
    const auto direct_geometry = trailer.value("directGeometry").toObject();
    const auto plane = direct_geometry.value("referencePlane").toObject();
    const auto center_bone = plane.value("centerBone").toString();
    const auto forward_bone = plane.value("forwardBone").toString();
    const auto left_bone = plane.value("leftBone").toString();
    const auto right_bone = plane.value("rightBone").toString();
    if (!center_bone.isEmpty() && !forward_bone.isEmpty() && !left_bone.isEmpty() && !right_bone.isEmpty()) {
        pending_frame_.reference_plane = motion_bridge::BodyReferencePlane{
            plane.value("mode").toString().toStdString(),
            center_bone.toStdString(),
            forward_bone.toStdString(),
            left_bone.toStdString(),
            right_bone.toStdString(),
        };
    }
    const auto target_semantic = direct_geometry.value("targetSemantic").toString();
    if (!target_semantic.isEmpty()) {
        pending_frame_.l0_reference_length = true;
        pending_frame_.l0_activity_window = direct_geometry.value("l0Normalization").toString() == u"activity_window";
        const auto l0_min = direct_geometry.value("l0MinMeters");
        const auto l0_max = direct_geometry.value("l0MaxMeters");
        if (l0_min.isDouble() && l0_max.isDouble() && l0_max.toDouble() > l0_min.toDouble()) {
            pending_frame_.direct_l0_min_meters = l0_min.toDouble();
            pending_frame_.direct_l0_max_meters = l0_max.toDouble();
        }
        pending_frame_.direct_l0_inverted = direct_geometry.value("l0Inverted").toBool();
        const auto output_axes = direct_geometry.value("outputAxes").toArray();
        if (!output_axes.isEmpty()) {
            pending_frame_.active_axes.fill(false);
            constexpr std::array<const char*, 6> names{"L0", "L1", "L2", "R0", "R1", "R2"};
            for (const auto& raw_axis : output_axes) {
                const auto axis = raw_axis.toString();
                for (std::size_t index = 0; index < names.size(); ++index) {
                    if (axis == QString::fromLatin1(names[index])) pending_frame_.active_axes[index] = true;
                }
            }
        }
    }
    Participant participant;
    participant.stable_key = packet.value("stableKey").toString().toStdString();
    participant.skeleton_id = packet.value("modelName").toString().toStdString();
    participant.role = trailer.value("role").toString().toStdString();
    for (const auto& raw_bone_name : trailer.value("contactBones").toArray()) {
        const auto bone_name = raw_bone_name.toString();
        if (!bone_name.isEmpty() && !pending_confirmed_target_bones_.contains(bone_name)) {
            pending_confirmed_target_bones_.push_back(bone_name);
        }
    }
    for (const auto& raw_pair : trailer.value("contactPairs").toArray()) {
        const auto pair = raw_pair.toObject();
        const auto reference_slot = pair.value("reference").toObject().value("participantSlot").toString().trimmed().toStdString();
        const auto target_bone = pair.value("target").toObject().value("bone").toString().trimmed().toStdString();
        if (reference_slot.empty() || target_bone.empty()) continue;
        auto& bones = pending_target_bones_by_slot_[reference_slot];
        if (std::find(bones.begin(), bones.end(), target_bone) == bones.end()) bones.push_back(target_bone);
    }
    for (const auto& raw_bone : packet.value("bones").toArray()) {
        const auto object = raw_bone.toObject();
        const auto position = vec3(object.value("pos"));
        const auto rotation = quaternion(object.value("rot"));
        const auto name = object.value("name").toString();
        if (name.isEmpty() || !position || !rotation) continue;
        participant.bones.emplace(name.toStdString(), BonePose{name.toStdString(), *position, *rotation});
    }
    const auto participant_slot = trailer.value("participantSlot").toString().trimmed().toStdString();
    if (!participant.stable_key.empty() && !participant_slot.empty()) {
        pending_participant_slots_.insert_or_assign(participant.stable_key, participant_slot);
    }
    const auto axis_fallback = direct_geometry.value("axisFallback").toObject();
    if (axis_fallback.value("mode").toString() == u"origin_local_x_reference_length") {
        const auto length_meters = axis_fallback.value("lengthCm").toDouble() / 100.0;
        const auto origin = participant.bones.find("Penis01");
        if (origin != participant.bones.end() && length_meters > 1e-5) {
            const auto axis = rotate(origin->second.rotation, {1.0, 0.0, 0.0});
            auto virtual_pose = origin->second;
            virtual_pose.position = add_scaled(origin->second.position, axis, length_meters);
            virtual_pose.name = "Penis02";
            participant.bones.insert_or_assign("Penis02", virtual_pose);
            virtual_pose.name = "Penis09";
            participant.bones.insert_or_assign("Penis09", virtual_pose);
        }
    }
    if (!participant.bones.empty()) pending_frame_.participants.push_back(std::move(participant));
    if (!coalesce_timer_->isActive()) coalesce_timer_->start();
}

void FallenDollInput::consume_motion_frame(const QJsonObject& packet) {
    // The public adapter protocol is deliberately accepted by the same spool
    // reader as Fallen Doll's compact stream. An external game only needs to
    // append one complete motion-frame/v1 JSON object per line; no DLL loading
    // or game-specific code is ever run inside Motion Bridge.
    emit_pending_frame();
    MotionFrame frame;
    frame.schema = "motion-frame/v1";
    frame.game_id = packet.value("gameId").toString().toStdString();
    frame.sequence = packet.value("sequence").toVariant().toULongLong();
    frame.monotonic_time = std::chrono::microseconds{packet.value("monotonicUs").toVariant().toLongLong()};
    const auto action = packet.value("action").toObject();
    frame.action_active = action.value("active").toBool();
    frame.action_id = action.value("id").toString().toStdString();
    frame.action_category = action.value("category").toString().toStdString();
    const auto direct_geometry = packet.value("directGeometry").toObject();
    frame.direct_l0_inverted = direct_geometry.value("l0Inverted").toBool();
    frame.l0_activity_window = direct_geometry.value("l0Normalization").toString() == u"activity_window";
    for (const auto& raw_participant : packet.value("participants").toArray()) {
        const auto raw = raw_participant.toObject();
        Participant participant;
        participant.stable_key = raw.value("stableKey").toString().toStdString();
        participant.role = raw.value("role").toString().toStdString();
        participant.skeleton_id = raw.value("skeletonId").toString().toStdString();
        for (const auto& raw_bone : raw.value("bones").toArray()) {
            const auto bone = raw_bone.toObject();
            const auto name = bone.value("name").toString();
            const auto position = vec3(bone.value("pos"));
            const auto rotation = quaternion(bone.value("rot"));
            if (name.isEmpty() || !position || !rotation) continue;
            participant.bones.emplace(name.toStdString(), BonePose{name.toStdString(), *position, *rotation});
        }
        if (!participant.bones.empty()) frame.participants.push_back(std::move(participant));
    }
    if (frame.game_id.empty() || frame.participants.empty()) return;
    if (frame.sequence == 0) frame.sequence = ++sequence_;
    record_valid_frame();
    emit frame_ready(std::move(frame));
}

void FallenDollInput::emit_pending_frame() {
    if (pending_timestamp_ < 0 || pending_frame_.participants.empty()) return;
    // The UE4SS stream keeps each participant's real bone names.  MotionEngine
    // deliberately consumes a stable canonical target name, so resolve the
    // action/profile semantic once all same-timestamp participants have been
    // coalesced and alias the highest-priority stable contact bone to M_Gen.
    std::unordered_map<std::string, std::vector<std::string>> candidate_bones_by_reference;
    for (const auto& [participant_key, participant_slot] : pending_participant_slots_) {
        const auto target_bones = pending_target_bones_by_slot_.find(participant_slot);
        if (target_bones != pending_target_bones_by_slot_.end() && !target_bones->second.empty()) {
            candidate_bones_by_reference.emplace(participant_key, target_bones->second);
        }
    }
    (void)target_selector_.alias_target(
        pending_frame_,
        contact_target_candidates(pending_confirmed_target_bones_),
        "Penis01",
        "M_Gen",
        preferred_reference_participant_,
        candidate_bones_by_reference);
    record_valid_frame();
    emit frame_ready(std::move(pending_frame_));
    pending_frame_ = {};
    pending_timestamp_ = -1;
    pending_confirmed_target_bones_.clear();
    pending_participant_slots_.clear();
    pending_target_bones_by_slot_.clear();
}
