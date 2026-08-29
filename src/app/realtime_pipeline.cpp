#include "realtime_pipeline.hpp"
#include "motion_bridge_settings.hpp"

#include <QDir>

#include <algorithm>
#include <cmath>

using namespace motion_bridge;

namespace {

QVariantList axes_to_variant(const Axes& axes) {
    QVariantList result;
    for (const auto value : axes.values) result.push_back(value);
    return result;
}

DeviceRouter::Mode parse_mode(const QString& value) {
    if (value == u"usb") return DeviceRouter::Mode::Usb;
    if (value == u"wifi") return DeviceRouter::Mode::Wifi;
    if (value == u"intiface") return DeviceRouter::Mode::Intiface;
    return DeviceRouter::Mode::None;
}

QString mode_name(const DeviceRouter::Mode mode) {
    switch (mode) {
    case DeviceRouter::Mode::Usb: return QStringLiteral("usb");
    case DeviceRouter::Mode::Wifi: return QStringLiteral("wifi");
    case DeviceRouter::Mode::Intiface: return QStringLiteral("intiface");
    default: return QStringLiteral("none");
    }
}

} // namespace

RealtimePipeline::RealtimePipeline(QObject* parent) : QObject(parent) {
    // These objects perform all of their work on the realtime thread.  Make
    // them children of the pipeline before it is moved so that Qt migrates
    // their timers, sockets and file watcher with the pipeline as one unit.
    input_ = new FallenDollInput(this);
    device_ = new DeviceRouter(this);
    heartbeat_ = new QTimer(this);
    heartbeat_->setInterval(20);
}

void RealtimePipeline::start() {
    clock_.start();
    load_settings();
    connect(input_, &FallenDollInput::frame_ready, this, &RealtimePipeline::on_frame);
    connect(input_, &FallenDollInput::connection_changed, this, [this](const bool connected, const QString& detail) { emit stream_status_changed(connected, detail); });
    connect(device_, &DeviceRouter::status_changed, this, [this](const QString& status, const bool) { emit output_status_changed(status, device_->armed(), mode_name(device_->mode())); });
    connect(heartbeat_, &QTimer::timeout, this, &RealtimePipeline::on_heartbeat);
    heartbeat_->start();
    input_->start();
    emit output_status_changed(device_->armed() ? tr("Output armed") : tr("Output disarmed"), device_->armed(), mode_name(device_->mode()));
}

void RealtimePipeline::stop() { heartbeat_->stop(); device_->emergency_stop(); }
void RealtimePipeline::set_armed(const bool armed) { device_->set_armed(armed); save_settings(); }
void RealtimePipeline::emergency_stop() { device_->emergency_stop(); }
void RealtimePipeline::set_output_mode(const QString& mode) { device_->set_mode(parse_mode(mode)); save_settings(); }
void RealtimePipeline::set_usb_port(const QString& port) { usb_port_ = port.trimmed(); device_->set_usb_port(usb_port_); auto settings = motion_bridge_settings(); settings.setValue("device/usbPort", usb_port_); publish_connection_settings(); }
void RealtimePipeline::set_wifi_endpoint(const QString& host, const int port) { wifi_host_ = host.trimmed(); wifi_port_ = std::clamp(port, 1, 65535); device_->set_wifi_endpoint(wifi_host_, static_cast<quint16>(wifi_port_)); auto settings = motion_bridge_settings(); settings.setValue("device/wifiHost", wifi_host_); settings.setValue("device/wifiPort", wifi_port_); publish_connection_settings(); }
void RealtimePipeline::set_intiface_url(const QString& url) { intiface_url_ = url.trimmed(); device_->set_intiface_url(QUrl(intiface_url_)); auto settings = motion_bridge_settings(); settings.setValue("device/intifaceUrl", intiface_url_); publish_connection_settings(); }

void RealtimePipeline::set_axis_gain(const int axis, const double value) {
    if (axis < 0 || axis >= 6) return;
    auto tuning = engine_.axis_tuning();
    auto& item = tuning[static_cast<std::size_t>(axis)];
    const auto next = std::clamp(value, 0.25, 4.0);
    if (std::abs(item.gain - next) < 0.0001) return;
    item.gain = next;
    engine_.set_axis_tuning(tuning);
    auto settings = motion_bridge_settings();
    settings.setValue(QString("motion/%1/gain").arg(QString::fromLatin1(kAxisNames[static_cast<std::size_t>(axis)])), item.gain);
    publish_axis_gains();
}

void RealtimePipeline::set_axis_range(const int axis, const double minimum, const double maximum) {
    if (axis < 0 || axis >= 6) return;
    auto tuning = engine_.axis_tuning();
    auto& item = tuning[static_cast<std::size_t>(axis)];
    const auto next_minimum = std::clamp(std::min(minimum, maximum), 0.0, 1.0);
    const auto next_maximum = std::clamp(std::max(minimum, maximum), 0.0, 1.0);
    if (std::abs(item.output_min - next_minimum) < 0.0001 &&
        std::abs(item.output_max - next_maximum) < 0.0001) return;
    item.output_min = next_minimum;
    item.output_max = next_maximum;
    engine_.set_axis_tuning(tuning);
    auto settings = motion_bridge_settings();
    const auto axis_name = QString::fromLatin1(kAxisNames[static_cast<std::size_t>(axis)]);
    settings.setValue(QString("motion/%1/min").arg(axis_name), item.output_min);
    settings.setValue(QString("motion/%1/max").arg(axis_name), item.output_max);
    publish_axis_ranges();
}

void RealtimePipeline::set_stream_path(const QString& path) {
    spool_path_ = path;
    input_->set_spool_path(path);
    input_->start();
    reset_participant_cache();
    emit reference_participants_changed(reference_participants_);
    auto settings = motion_bridge_settings();
    settings.setValue("input/spoolPath", path);
    emit spool_path_changed(path);
}

void RealtimePipeline::set_theme(const QString& theme) {
    const auto normalized = theme.compare("light", Qt::CaseInsensitive) == 0 ? QStringLiteral("light") : QStringLiteral("dark");
    if (theme_ == normalized) return;
    theme_ = normalized;
    auto settings = motion_bridge_settings();
    settings.setValue("ui/theme", theme_);
    emit theme_changed(theme_);
}

void RealtimePipeline::set_reference_participant(const QString& reference) {
    auto contact = engine_.contact_config();
    const auto next_reference = reference.trimmed().toStdString();
    if (contact.reference_participant == next_reference) return;
    // Changing the source actor switches all six axes. Stop output first, then
    // require an explicit arm after a fresh frame confirms the new route.
    device_->emergency_stop();
    contact.reference_participant = next_reference;
    engine_.set_contact_config(contact);
    input_->set_reference_participant(reference);
    auto settings = motion_bridge_settings();
    settings.setValue("contact/referenceParticipant", reference.trimmed());
    if (cache_reference_participant(reference.trimmed())) emit reference_participants_changed(reference_participants_);
    publish_reference_participant();
}

void RealtimePipeline::on_frame(MotionFrame frame) {
    reference_plane_label_.clear();
    if (frame.reference_plane) {
        const auto& plane = *frame.reference_plane;
        reference_plane_label_ = QString::fromStdString(plane.mode);
        if (!plane.center_bone.empty()) {
            reference_plane_label_ += " · " + QString::fromStdString(plane.center_bone) + " / "
                + QString::fromStdString(plane.left_bone) + " / " + QString::fromStdString(plane.right_bone);
        }
    }
    publish_participant_choices(frame);
    frame.monotonic_time = now();
    last_input_time_ = frame.monotonic_time;
    snapshot_ = engine_.process(frame);
    if (device_->armed()) device_->send(snapshot_.device_axes);
    publish_snapshot();
}

void RealtimePipeline::on_heartbeat() {
    const auto current = now();
    if (current - last_input_time_ < std::chrono::milliseconds{25}) return;
    snapshot_ = engine_.process_missing(current);
    if (device_->armed()) device_->send(snapshot_.device_axes);
    publish_snapshot();
}

void RealtimePipeline::publish_snapshot() {
    if (last_ui_snapshot_) {
        bool changed = last_ui_snapshot_->state != snapshot_.state || last_ui_snapshot_->action_id != snapshot_.action_id;
        for (std::size_t index = 0; index < snapshot_.device_axes.values.size() && !changed; ++index) {
            changed = std::abs(last_ui_snapshot_->device_axes[index] - snapshot_.device_axes[index]) > 0.0001 ||
                std::abs(last_ui_snapshot_->raw_axes[index] - snapshot_.raw_axes[index]) > 0.0001;
        }
        if (!changed) return;
    }
    last_ui_snapshot_ = snapshot_;
    emit snapshot_ready(QString::fromLatin1(to_string(snapshot_.state)), QString::fromStdString(snapshot_.action_id), reference_plane_label_, axes_to_variant(snapshot_.raw_axes), axes_to_variant(snapshot_.device_axes));
}

void RealtimePipeline::save_settings() {
    auto settings = motion_bridge_settings();
    settings.setValue("device/mode", mode_name(device_->mode()));
    settings.setValue("device/armed", false);
}

void RealtimePipeline::load_settings() {
    auto settings = motion_bridge_settings();
    theme_ = settings.value("ui/theme", "dark").toString().toLower() == u"light" ? QStringLiteral("light") : QStringLiteral("dark");
    const auto default_path = QDir::homePath() + "/.f8/studio/games/fallen-doll/runtime/fd-skeleton.ndjson";
    spool_path_ = settings.value("input/spoolPath", default_path).toString();
    input_->set_spool_path(spool_path_);
    device_->set_mode(parse_mode(settings.value("device/mode", "none").toString()));
    usb_port_ = settings.value("device/usbPort").toString();
    wifi_host_ = settings.value("device/wifiHost", "tcode.local").toString();
    wifi_port_ = settings.value("device/wifiPort", 8000).toInt();
    intiface_url_ = settings.value("device/intifaceUrl", "ws://127.0.0.1:12345").toString();
    device_->set_usb_port(usb_port_);
    device_->set_wifi_endpoint(wifi_host_, static_cast<quint16>(std::clamp(wifi_port_, 1, 65535)));
    device_->set_intiface_url(QUrl(intiface_url_));
    auto contact = engine_.contact_config();
    const auto contact_text = [&settings](const char* key, const std::string& fallback) { return settings.value(QString("contact/%1").arg(key), QString::fromStdString(fallback)).toString().toStdString(); };
    const auto contact_number = [&settings](const char* key, const double fallback) { return settings.value(QString("contact/%1").arg(key), fallback).toDouble(); };
    contact.origin_bone = contact_text("originBone", contact.origin_bone); contact.direction_bone = contact_text("directionBone", contact.direction_bone);
    contact.tip_bone = contact_text("tipBone", contact.tip_bone); contact.support_bone = contact_text("supportBone", contact.support_bone);
    contact.support_right_axis = contact_text("supportRightAxis", contact.support_right_axis); contact.support_up_axis = contact_text("supportUpAxis", contact.support_up_axis);
    contact.target_up_axis = contact_text("targetUpAxis", contact.target_up_axis); contact.target_right_axis = contact_text("targetRightAxis", contact.target_right_axis);
    contact.l0_min_meters = contact_number("l0MinMeters", contact.l0_min_meters); contact.l0_max_meters = contact_number("l0MaxMeters", contact.l0_max_meters);
    contact.lateral_range_meters = contact_number("lateralRangeMeters", contact.lateral_range_meters); contact.twist_range_degrees = contact_number("twistRangeDegrees", contact.twist_range_degrees);
    contact.tilt_range_degrees = contact_number("tiltRangeDegrees", contact.tilt_range_degrees); contact.radius_scale = contact_number("radiusScale", contact.radius_scale);
    contact.invert_l0 = settings.value("contact/invertL0", contact.invert_l0).toBool(); contact.require_contact = settings.value("contact/requireContact", contact.require_contact).toBool();
    contact.reference_participant = contact_text("referenceParticipant", contact.reference_participant);
    contact.target_participant.clear();
    engine_.set_contact_config(contact);
    input_->set_reference_participant(QString::fromStdString(contact.reference_participant));
    settings.remove("contact/targetParticipant");
    reset_participant_cache();
    auto tuning = engine_.axis_tuning();
    for (int index = 0; index < 6; ++index) {
        auto& item = tuning[static_cast<std::size_t>(index)]; const auto axis_name = QString::fromLatin1(kAxisNames[static_cast<std::size_t>(index)]);
        item.gain = settings.value(QString("motion/%1/gain").arg(axis_name), 1.0).toDouble();
        item.center = settings.value(QString("motion/%1/center").arg(axis_name), item.center).toDouble();
        item.dead_zone = settings.value(QString("motion/%1/deadZone").arg(axis_name), item.dead_zone).toDouble();
        item.output_min = settings.value(QString("motion/%1/min").arg(axis_name), item.output_min).toDouble();
        item.output_max = settings.value(QString("motion/%1/max").arg(axis_name), item.output_max).toDouble();
        const auto curve = settings.value(QString("motion/%1/curve").arg(axis_name), "LINEAR").toString().toUpper();
        item.curve = curve == u"SMOOTHERSTEP" ? MotionCurve::Smootherstep : curve == u"SMOOTHSTEP" ? MotionCurve::Smoothstep : MotionCurve::Linear;
    }
    engine_.set_axis_tuning(tuning);
    publish_connection_settings();
    publish_axis_gains();
    publish_axis_ranges();
    publish_reference_participant();
    emit reference_participants_changed(reference_participants_);
    emit theme_changed(theme_);
    emit spool_path_changed(spool_path_);
}

void RealtimePipeline::publish_connection_settings() {
    emit connection_settings_changed(usb_port_, wifi_host_, wifi_port_, intiface_url_);
}

void RealtimePipeline::publish_axis_gains() {
    QVariantList gains;
    for (const auto& item : engine_.axis_tuning()) gains.push_back(item.gain);
    emit axis_gains_changed(gains);
}

void RealtimePipeline::publish_axis_ranges() {
    QVariantList minimums;
    QVariantList maximums;
    for (const auto& item : engine_.axis_tuning()) {
        minimums.push_back(item.output_min);
        maximums.push_back(item.output_max);
    }
    emit axis_ranges_changed(minimums, maximums);
}

void RealtimePipeline::publish_participant_choices(const MotionFrame& frame) {
    const auto contact = engine_.contact_config();
    auto changed = false;
    const auto action_id = QString::fromStdString(frame.action_id);
    if (action_id != participant_cache_action_id_) {
        reset_participant_cache();
        participant_cache_action_id_ = action_id;
        changed = true;
    }
    for (const auto& participant : frame.participants) {
        const auto key = QString::fromStdString(participant.stable_key);
        if (key.isEmpty()) continue;
        const auto label = QString::fromStdString(participant.display_name);
        if (participant.bones.contains(contact.origin_bone)) changed = cache_reference_participant(key, label) || changed;
    }
    if (changed) emit reference_participants_changed(reference_participants_);
}

void RealtimePipeline::publish_reference_participant() {
    const auto contact = engine_.contact_config();
    emit reference_participant_changed(QString::fromStdString(contact.reference_participant));
}

void RealtimePipeline::reset_participant_cache() {
    participant_cache_action_id_.clear();
    reference_participants_ = {QVariantMap{{"key", ""}, {"label", tr("Automatic")}}};
}

bool RealtimePipeline::cache_reference_participant(const QString& key, const QString& label) {
    const auto normalized_key = key.trimmed();
    if (normalized_key.isEmpty()) return false;
    const auto normalized_label = label.isEmpty() ? normalized_key : label;
    for (auto& value : reference_participants_) {
        auto option = value.toMap();
        if (option.value("key").toString() != normalized_key) continue;
        if (option.value("label").toString() == normalized_label) return false;
        option.insert("label", normalized_label);
        value = option;
        return true;
    }
    reference_participants_.push_back(QVariantMap{{"key", normalized_key}, {"label", normalized_label}});
    return true;
}

std::chrono::microseconds RealtimePipeline::now() const { return std::chrono::microseconds{clock_.nsecsElapsed() / 1000}; }
