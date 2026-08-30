#include "device_router.hpp"

#include "motion_bridge/tcode.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>

using namespace motion_bridge;

DeviceRouter::DeviceRouter(QObject* parent) : QObject(parent) {
    serial_ = new QSerialPort(this);
    udp_ = new QUdpSocket(this);
    intiface_ = new QWebSocket(QString(), QWebSocketProtocol::VersionLatest, this);
    serial_->setBaudRate(115200);
    serial_->setDataBits(QSerialPort::Data8);
    serial_->setParity(QSerialPort::NoParity);
    serial_->setStopBits(QSerialPort::OneStop);
    serial_->setFlowControl(QSerialPort::NoFlowControl);
    connect(intiface_, &QWebSocket::textMessageReceived, this, &DeviceRouter::on_intiface_message);
    connect(intiface_, &QWebSocket::errorOccurred, this, &DeviceRouter::on_intiface_error);
    connect(intiface_, &QWebSocket::connected, this, [this] {
        const auto request = QJsonObject{{"RequestServerInfo", QJsonObject{
            {"Id", intiface_request_id_++}, {"ClientName", "Motion Bridge"}, {"MessageVersion", 3}
        }}};
        intiface_->sendTextMessage(QString::fromUtf8(QJsonDocument(QJsonArray{request}).toJson(QJsonDocument::Compact)));
    });
}

void DeviceRouter::set_mode(const Mode mode) { if (mode_ != mode) { emergency_stop(); mode_ = mode; reset_output_tracking(); ensure_transport(); } }
void DeviceRouter::set_usb_port(const QString& port) { if (usb_port_ != port) { emergency_stop(); usb_port_ = port; reset_output_tracking(); ensure_transport(); } }
void DeviceRouter::set_wifi_endpoint(const QString& host, const quint16 port) { wifi_host_ = host; wifi_port_ = port; }
void DeviceRouter::set_intiface_url(const QUrl& url) { if (intiface_url_ != url) { intiface_->close(); intiface_url_ = url; reset_output_tracking(); ensure_transport(); } }
void DeviceRouter::set_armed(const bool armed) {
    if (!armed) {
        emergency_stop();
        return;
    }
    armed_ = true;
    reset_output_tracking();
    ensure_transport();
}
DeviceRouter::Mode DeviceRouter::mode() const noexcept { return mode_; }
bool DeviceRouter::armed() const noexcept { return armed_; }

void DeviceRouter::ensure_transport() {
    if (mode_ != Mode::Usb && serial_->isOpen()) serial_->close();
    if (mode_ != Mode::Intiface && intiface_->state() != QAbstractSocket::UnconnectedState) intiface_->close();
    if (!armed_) { emit status_changed(tr("Output disarmed"), false); return; }
    if (mode_ == Mode::Usb) {
        if (usb_port_.isEmpty()) { emit status_changed(tr("Select a USB port"), false); return; }
        if (!serial_->isOpen()) {
            serial_->setPortName(usb_port_);
            if (!serial_->open(QIODevice::WriteOnly)) { emit status_changed(serial_->errorString(), false); return; }
            // Match MultiFunPlayer's defaults. Set the control lines after opening because
            // QSerialPort cannot change them while the port is closed.
            serial_->setDataTerminalReady(true);
            serial_->setRequestToSend(true);
        }
        emit status_changed(tr("USB armed: %1").arg(usb_port_), true);
    } else if (mode_ == Mode::Wifi) {
        if (udp_->peerName() != wifi_host_ || udp_->peerPort() != wifi_port_) {
            udp_->abort();
            udp_->connectToHost(wifi_host_, wifi_port_);
        }
        emit status_changed(tr("Wi-Fi armed: %1:%2").arg(wifi_host_).arg(wifi_port_), true);
    } else if (mode_ == Mode::Intiface) {
        if (intiface_->state() == QAbstractSocket::UnconnectedState) intiface_->open(intiface_url_);
        emit status_changed(tr("Connecting to Intiface Desktop"), false);
    } else {
        emit status_changed(tr("Select an output method"), false);
    }
}

void DeviceRouter::send(const Axes& axes, const std::chrono::milliseconds interval) {
    if (armed_) send_output(axes, interval);
}

void DeviceRouter::send_output(
    const Axes& axes,
    const std::chrono::milliseconds interval,
    const bool force_full) {
    if (mode_ == Mode::Usb) {
        if (!serial_->isOpen()) return;
        AxisMask dirty_axes{};
        for (std::size_t index = 0; index < dirty_axes.size(); ++index) {
            const auto current = static_cast<int>(std::floor(std::clamp(axes[index], 0.0, 1.0) * 9999.0 + 0.5));
            const auto previous = static_cast<int>(std::floor(std::clamp(last_sent_axes_[index], 0.0, 1.0) * 9999.0 + 0.5));
            dirty_axes[index] = force_full || !last_sent_valid_[index] || current != previous;
        }
        const auto payload = QByteArray::fromStdString(encode_tcode(axes, interval, dirty_axes));
        if (!payload.isEmpty() && serial_->write(payload) >= 0) {
            for (std::size_t index = 0; index < dirty_axes.size(); ++index) {
                if (!dirty_axes[index]) continue;
                last_sent_axes_[index] = axes[index];
                last_sent_valid_[index] = true;
            }
        }
        return;
    }
    if (mode_ == Mode::Wifi) {
        // UDP remains a complete state packet. A lost dirty-only datagram must
        // not leave one axis stale until that axis changes again.
        udp_->write(QByteArray::fromStdString(encode_tcode(axes, interval)));
        return;
    }
    if (mode_ == Mode::Intiface && intiface_->state() == QAbstractSocket::ConnectedState && intiface_device_index_ >= 0) {
        if (!force_full && last_sent_valid_[0] && std::abs(last_sent_axes_[0] - axes[0]) < 0.005) return;
        // Intiface only receives explicitly supported scalar features. Initial mapping uses
        // the first Linear actuator for L0; richer per-feature mapping belongs in Settings.
        const auto message = QJsonObject{{"ScalarCmd", QJsonObject{
            {"Id", intiface_request_id_++}, {"DeviceIndex", intiface_device_index_},
            {"Scalars", QJsonArray{QJsonObject{{"Index", 0}, {"Scalar", axes[0]}, {"ActuatorType", "Linear"}}}}
        }}};
        intiface_->sendTextMessage(QString::fromUtf8(QJsonDocument(QJsonArray{message}).toJson(QJsonDocument::Compact)));
        last_sent_axes_[0] = axes[0];
        last_sent_valid_[0] = true;
    }
}

void DeviceRouter::emergency_stop() {
    const Axes center{};
    if (armed_) send_output(center, std::chrono::milliseconds{20}, true);
    send_intiface_zero();
    armed_ = false;
    reset_output_tracking();
    if (serial_->isOpen()) serial_->close();
    emit status_changed(tr("Output disarmed and centered"), false);
}

void DeviceRouter::send_intiface_zero() {
    if (intiface_->state() != QAbstractSocket::ConnectedState || intiface_device_index_ < 0) return;
    const auto message = QJsonObject{{"StopDeviceCmd", QJsonObject{{"Id", intiface_request_id_++}, {"DeviceIndex", intiface_device_index_}}}};
    intiface_->sendTextMessage(QString::fromUtf8(QJsonDocument(QJsonArray{message}).toJson(QJsonDocument::Compact)));
}

void DeviceRouter::on_intiface_message(const QString& message) {
    const auto document = QJsonDocument::fromJson(message.toUtf8());
    const auto messages = document.isArray() ? document.array() : QJsonArray{document.object()};
    for (const auto& value : messages) {
        const auto object = value.toObject();
        if (object.contains("ServerInfo")) {
            const auto request = QJsonObject{{"StartScanning", QJsonObject{{"Id", intiface_request_id_++}}}};
            intiface_->sendTextMessage(QString::fromUtf8(QJsonDocument(QJsonArray{request}).toJson(QJsonDocument::Compact)));
        }
        if (object.contains("DeviceAdded")) {
            const auto device = object.value("DeviceAdded").toObject();
            const auto features = device.value("DeviceMessages").toObject().value("ScalarCmd").toArray();
            const auto has_linear = std::any_of(features.begin(), features.end(), [](const QJsonValue& feature) {
                return feature.toObject().value("ActuatorType").toString().compare("Linear", Qt::CaseInsensitive) == 0;
            });
            intiface_device_index_ = has_linear ? device.value("DeviceIndex").toInt(-1) : -1;
            if (intiface_device_index_ >= 0) {
                reset_output_tracking();
                emit status_changed(tr("Intiface armed: %1 (L0 → first Linear feature)").arg(device.value("DeviceName").toString()), true);
            } else {
                emit status_changed(tr("Intiface device has no Linear feature; TCode output remains disarmed"), false);
            }
        }
    }
}

void DeviceRouter::reset_output_tracking() { last_sent_valid_.fill(false); }

void DeviceRouter::on_intiface_error(const QAbstractSocket::SocketError) { emit status_changed(intiface_->errorString(), false); }
