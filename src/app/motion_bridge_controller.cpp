#include "motion_bridge_controller.hpp"

#include <QGuiApplication>
#include <QMetaObject>
#include <QScreen>
#include <QSerialPortInfo>

#include <algorithm>

MotionBridgeController::MotionBridgeController(QObject* parent) : QObject(parent), pipeline_(new RealtimePipeline) {
    pipeline_->moveToThread(&realtime_thread_);
    connect(&realtime_thread_, &QThread::finished, pipeline_, &QObject::deleteLater);
    connect(pipeline_, &RealtimePipeline::snapshot_ready, this, [this](const QString& state, const QString& action, const QString& reference_plane, const QVariantList& raw, const QVariantList& device) {
        motion_state_ = state; action_name_ = action; reference_plane_ = reference_plane; raw_axes_ = raw; device_axes_ = device; emit snapshotChanged();
    });
    connect(pipeline_, &RealtimePipeline::stream_status_changed, this, [this](const bool connected, const QString& status) {
        stream_connected_ = connected; stream_status_ = status; emit statusChanged();
    });
    connect(pipeline_, &RealtimePipeline::output_status_changed, this, [this](const QString& status, const bool armed, const QString& mode) {
        output_status_ = status; armed_ = armed; output_mode_ = mode; emit statusChanged();
    });
    connect(pipeline_, &RealtimePipeline::spool_path_changed, this, [this](const QString& path) { spool_path_ = path; emit statusChanged(); });
    connect(pipeline_, &RealtimePipeline::connection_settings_changed, this, [this](const QString& usb_port, const QString& wifi_host, const int wifi_port, const QString& intiface_url) {
        usb_port_ = usb_port; wifi_host_ = wifi_host; wifi_port_ = wifi_port; intiface_url_ = intiface_url; emit settingsChanged();
    });
    connect(pipeline_, &RealtimePipeline::axis_gains_changed, this, [this](const QVariantList& gains) { axis_gains_ = gains; emit settingsChanged(); });
    connect(pipeline_, &RealtimePipeline::axis_ranges_changed, this, [this](const QVariantList& minimums, const QVariantList& maximums) {
        axis_minimums_ = minimums;
        axis_maximums_ = maximums;
        emit settingsChanged();
    });
    connect(pipeline_, &RealtimePipeline::theme_changed, this, [this](const QString& theme) { theme_ = theme; emit themeChanged(); });
    usb_scan_timer_.setInterval(1500);
    connect(&usb_scan_timer_, &QTimer::timeout, this, &MotionBridgeController::refresh_usb_ports);
    refresh_usb_ports();
    usb_scan_timer_.start();
    realtime_thread_.start(QThread::HighPriority);
    QMetaObject::invokeMethod(pipeline_, "start", Qt::QueuedConnection);
}

MotionBridgeController::~MotionBridgeController() {
    if (pipeline_ != nullptr && realtime_thread_.isRunning()) {
        QMetaObject::invokeMethod(pipeline_, "stop", Qt::BlockingQueuedConnection);
        realtime_thread_.quit();
        realtime_thread_.wait();
    }
}

QString MotionBridgeController::stream_status() const { return stream_status_; }
bool MotionBridgeController::stream_connected() const { return stream_connected_; }
QString MotionBridgeController::output_status() const { return output_status_; }
QString MotionBridgeController::motion_state() const { return motion_state_; }
QString MotionBridgeController::action_name() const { return action_name_; }
QString MotionBridgeController::reference_plane() const { return reference_plane_; }
QVariantList MotionBridgeController::raw_axes() const { return raw_axes_; }
QVariantList MotionBridgeController::device_axes() const { return device_axes_; }
bool MotionBridgeController::armed() const { return armed_; }
QString MotionBridgeController::output_mode() const { return output_mode_; }
QString MotionBridgeController::spool_path() const { return spool_path_; }
QString MotionBridgeController::usb_port() const { return usb_port_; }
QString MotionBridgeController::wifi_host() const { return wifi_host_; }
int MotionBridgeController::wifi_port() const { return wifi_port_; }
QString MotionBridgeController::intiface_url() const { return intiface_url_; }
QVariantList MotionBridgeController::axis_gains() const { return axis_gains_; }
QVariantList MotionBridgeController::axis_minimums() const { return axis_minimums_; }
QVariantList MotionBridgeController::axis_maximums() const { return axis_maximums_; }
QStringList MotionBridgeController::usb_ports() const { return usb_ports_; }
QString MotionBridgeController::theme() const { return theme_; }

void MotionBridgeController::set_armed(const bool armed) { QMetaObject::invokeMethod(pipeline_, "set_armed", Qt::QueuedConnection, Q_ARG(bool, armed)); }
void MotionBridgeController::emergency_stop() { QMetaObject::invokeMethod(pipeline_, "emergency_stop", Qt::QueuedConnection); }
void MotionBridgeController::set_output_mode(const QString& mode) { QMetaObject::invokeMethod(pipeline_, "set_output_mode", Qt::QueuedConnection, Q_ARG(QString, mode)); }
void MotionBridgeController::set_usb_port(const QString& port) { QMetaObject::invokeMethod(pipeline_, "set_usb_port", Qt::QueuedConnection, Q_ARG(QString, port)); }
void MotionBridgeController::set_wifi_endpoint(const QString& host, const int port) { QMetaObject::invokeMethod(pipeline_, "set_wifi_endpoint", Qt::QueuedConnection, Q_ARG(QString, host), Q_ARG(int, port)); }
void MotionBridgeController::set_intiface_url(const QString& url) { QMetaObject::invokeMethod(pipeline_, "set_intiface_url", Qt::QueuedConnection, Q_ARG(QString, url)); }
void MotionBridgeController::set_axis_gain(const int axis, const double value) { QMetaObject::invokeMethod(pipeline_, "set_axis_gain", Qt::QueuedConnection, Q_ARG(int, axis), Q_ARG(double, value)); }
void MotionBridgeController::set_axis_range(const int axis, const double minimum, const double maximum) { QMetaObject::invokeMethod(pipeline_, "set_axis_range", Qt::QueuedConnection, Q_ARG(int, axis), Q_ARG(double, minimum), Q_ARG(double, maximum)); }
void MotionBridgeController::set_stream_path(const QString& path) { QMetaObject::invokeMethod(pipeline_, "set_stream_path", Qt::QueuedConnection, Q_ARG(QString, path)); }

QVariantMap MotionBridgeController::primary_screen_available_geometry() const {
    const auto* screen = QGuiApplication::primaryScreen();
    const auto area = screen ? screen->availableGeometry() : QRect{};
    return {{"x", area.x()}, {"y", area.y()}, {"width", area.width()}, {"height", area.height()}};
}

void MotionBridgeController::refresh_usb_ports() {
    QStringList ports;
    for (const auto& info : QSerialPortInfo::availablePorts()) {
        const auto name = info.portName().trimmed();
        if (!name.isEmpty() && !ports.contains(name, Qt::CaseInsensitive)) ports.push_back(name);
    }
    std::sort(ports.begin(), ports.end(), [](const QString& left, const QString& right) {
        return QString::localeAwareCompare(left, right) < 0;
    });
    if (ports == usb_ports_) return;
    usb_ports_ = ports;
    emit usbPortsChanged();
}

void MotionBridgeController::set_theme(const QString& theme) {
    QMetaObject::invokeMethod(pipeline_, "set_theme", Qt::QueuedConnection, Q_ARG(QString, theme));
}
