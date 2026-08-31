#include "device_router.hpp"

#include "motion_bridge/tcode.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkRequest>

#include <algorithm>
#include <cmath>

using namespace motion_bridge;

namespace {

constexpr auto kHandyBaseUrl = "https://www.handyfeeling.com/api/handy/v2";
constexpr int kHandyMinimumStrokePercent = 5;
constexpr qint64 kHandyRequestIntervalMs = 250;

QUrl handy_url(const QString& path) {
    return QUrl(QString::fromLatin1(kHandyBaseUrl) + path);
}

} // namespace

DeviceRouter::DeviceRouter(QObject* parent) : QObject(parent) {
    serial_ = new QSerialPort(this);
    udp_ = new QUdpSocket(this);
    intiface_ = new QWebSocket(QString(), QWebSocketProtocol::VersionLatest, this);
    handy_api_ = new QNetworkAccessManager(this);
    handy_clock_.start();
    serial_->setBaudRate(115200);
    serial_->setDataBits(QSerialPort::Data8);
    serial_->setParity(QSerialPort::NoParity);
    serial_->setStopBits(QSerialPort::OneStop);
    serial_->setFlowControl(QSerialPort::NoFlowControl);
    connect(intiface_, &QWebSocket::textMessageReceived, this, &DeviceRouter::on_intiface_message);
    connect(intiface_, &QWebSocket::errorOccurred, this, &DeviceRouter::on_intiface_error);
    connect(intiface_, &QWebSocket::connected, this, [this] {
        const auto request = QJsonObject{{"RequestServerInfo", QJsonObject{
            {"Id", intiface_request_id_++}, {"ClientName", "Motion Bridge"},
            {"ProtocolVersionMajor", 4}, {"ProtocolVersionMinor", 0}
        }}};
        intiface_->sendTextMessage(QString::fromUtf8(QJsonDocument(QJsonArray{request}).toJson(QJsonDocument::Compact)));
    });
}

void DeviceRouter::set_mode(const Mode mode) {
    if (mode_ == mode) return;
    emergency_stop();
    mode_ = mode;
    reset_output_tracking();
    ensure_transport();
}

void DeviceRouter::set_usb_port(const QString& port) {
    if (usb_port_ == port) return;
    emergency_stop();
    usb_port_ = port;
    reset_output_tracking();
    ensure_transport();
}

void DeviceRouter::set_wifi_endpoint(const QString& host, const quint16 port) {
    wifi_host_ = host;
    wifi_port_ = port;
}

void DeviceRouter::set_intiface_url(const QUrl& url) {
    if (intiface_url_ == url) return;
    intiface_->close();
    intiface_url_ = url;
    reset_output_tracking();
    ensure_transport();
}

void DeviceRouter::set_handy_connection_key(const QString& key) {
    const auto trimmed = key.trimmed();
    if (handy_connection_key_ == trimmed) return;
    emergency_stop();
    handy_connection_key_ = trimmed;
}

void DeviceRouter::set_armed(const bool armed) {
    if (!armed) {
        emergency_stop();
        return;
    }
    if (mode_ == Mode::Handy && handy_connection_key_.isEmpty()) {
        armed_ = false;
        emit status_changed(tr("Enter a Handy connection key"), false);
        return;
    }
    armed_ = true;
    handy_active_connection_key_ = handy_connection_key_;
    reset_output_tracking();
    reset_handy_motion_tracking();
    ensure_transport();
}

DeviceRouter::Mode DeviceRouter::mode() const noexcept { return mode_; }
bool DeviceRouter::armed() const noexcept { return armed_; }

void DeviceRouter::ensure_transport() {
    if (mode_ != Mode::Usb && serial_->isOpen()) serial_->close();
    if (mode_ != Mode::Intiface && intiface_->state() != QAbstractSocket::UnconnectedState) intiface_->close();
    if (!armed_) {
        emit status_changed(tr("Output disarmed"), false);
        return;
    }
    if (mode_ == Mode::Usb) {
        if (usb_port_.isEmpty()) {
            emit status_changed(tr("Select a USB port"), false);
            return;
        }
        if (!serial_->isOpen()) {
            serial_->setPortName(usb_port_);
            if (!serial_->open(QIODevice::WriteOnly)) {
                emit status_changed(serial_->errorString(), false);
                return;
            }
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
    } else if (mode_ == Mode::Handy) {
        request_handy_connection_check();
    } else {
        emit status_changed(tr("Select an output method"), false);
    }
}

void DeviceRouter::send(const Axes& axes, const std::chrono::milliseconds interval) {
    if (armed_) send_output(axes, interval);
}

void DeviceRouter::send_output(const Axes& axes, const std::chrono::milliseconds interval, const bool force_full) {
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
        udp_->write(QByteArray::fromStdString(encode_tcode(axes, interval)));
        return;
    }
    if (mode_ == Mode::Intiface && intiface_->state() == QAbstractSocket::ConnectedState &&
        intiface_device_index_ >= 0 && intiface_feature_index_ >= 0) {
        if (!force_full && last_sent_valid_[0] && std::abs(last_sent_axes_[0] - axes[0]) < 0.005) return;
        const auto position = static_cast<int>(std::lround(intiface_position_min_ +
            std::clamp(axes[0], 0.0, 1.0) * (intiface_position_max_ - intiface_position_min_)));
        const auto message = QJsonObject{{"OutputCmd", QJsonObject{
            {"Id", intiface_request_id_++}, {"DeviceIndex", intiface_device_index_},
            {"FeatureIndex", intiface_feature_index_},
            {"Command", QJsonObject{{"Position", QJsonObject{{"Value", position}}}}}
        }}};
        intiface_->sendTextMessage(QString::fromUtf8(QJsonDocument(QJsonArray{message}).toJson(QJsonDocument::Compact)));
        last_sent_axes_[0] = axes[0];
        last_sent_valid_[0] = true;
        return;
    }
    if (mode_ == Mode::Handy) send_handy_output(axes, interval);
}

void DeviceRouter::emergency_stop() {
    const Axes center{};
    if (armed_ && mode_ != Mode::Handy) send_output(center, std::chrono::milliseconds{20}, true);
    send_intiface_zero();
    if (mode_ == Mode::Handy) request_handy_stop();
    armed_ = false;
    reset_output_tracking();
    if (serial_->isOpen()) serial_->close();
    emit status_changed(tr("Output disarmed safely"), false);
}

void DeviceRouter::send_intiface_zero() {
    if (intiface_->state() != QAbstractSocket::ConnectedState || intiface_device_index_ < 0) return;
    const auto message = QJsonObject{{"StopCmd", QJsonObject{
        {"Id", intiface_request_id_++}, {"DeviceIndex", intiface_device_index_}, {"Inputs", false}, {"Outputs", true}
    }}};
    intiface_->sendTextMessage(QString::fromUtf8(QJsonDocument(QJsonArray{message}).toJson(QJsonDocument::Compact)));
}

void DeviceRouter::send_handy_output(const Axes& axes, const std::chrono::milliseconds interval) {
    const auto position = std::clamp(axes[0], 0.0, 1.0);
    handy_observed_min_ = std::min(handy_observed_min_, position);
    handy_observed_max_ = std::max(handy_observed_max_, position);
    const auto interval_ms = std::max<int64_t>(1, interval.count());
    if (handy_has_position_) {
        const auto speed = std::abs(position - handy_previous_position_) * 100.0 /
                           static_cast<double>(interval_ms) * 500.0;
        handy_desired_velocity_ = std::clamp(static_cast<int>(std::lround(speed)), 5, 100);
    }
    handy_previous_position_ = position;
    handy_has_position_ = true;

    const auto observed_stroke = static_cast<int>(std::ceil((handy_observed_max_ - handy_observed_min_) * 100.0));
    if (observed_stroke < kHandyMinimumStrokePercent) {
        if (!handy_waiting_for_motion_ && handy_state_ == HandyState::Ready) {
            handy_waiting_for_motion_ = true;
            emit status_changed(tr("Handy armed: waiting for L0 motion"), true);
        }
        return;
    }
    handy_waiting_for_motion_ = false;
    handy_desired_min_ = std::clamp(static_cast<int>(std::floor(handy_observed_min_ * 100.0)), 0, 100);
    handy_desired_max_ = std::clamp(static_cast<int>(std::ceil(handy_observed_max_ * 100.0)), 0, 100);
    request_handy_motion();
}

void DeviceRouter::request_handy_connection_check() {
    if (!armed_ || mode_ != Mode::Handy) return;
    if (handy_connection_key_.isEmpty()) {
        armed_ = false;
        emit status_changed(tr("Enter a Handy connection key"), false);
        return;
    }
    if (handy_reply_ != nullptr) return;
    handy_state_ = HandyState::CheckingConnection;
    emit status_changed(tr("Checking Handy connection"), false);
    send_handy_request(HandyRequest::CheckConnection, QStringLiteral("/connected"));
}

void DeviceRouter::send_handy_request(const HandyRequest operation, const QString& path, const QJsonObject& body) {
    if (handy_reply_ != nullptr || handy_active_connection_key_.isEmpty()) return;
    QNetworkRequest request(handy_url(path));
    request.setRawHeader("X-Connection-Key", handy_active_connection_key_.toUtf8());
    request.setTransferTimeout(8000);
    QNetworkReply* reply = nullptr;
    if (operation == HandyRequest::CheckConnection) {
        reply = handy_api_->get(request);
    } else {
        request.setHeader(QNetworkRequest::ContentTypeHeader, QStringLiteral("application/json"));
        reply = handy_api_->put(request, QJsonDocument(body).toJson(QJsonDocument::Compact));
    }
    handy_reply_ = reply;
    connect(reply, &QNetworkReply::finished, this, [this, reply, operation] {
        const auto status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        const auto error = reply->error();
        const auto error_text = reply->errorString();
        const auto response = reply->readAll();
        if (handy_reply_ == reply) {
            handy_reply_ = nullptr;
        }
        reply->deleteLater();
        on_handy_reply(operation, status, error, error_text, response);
    });
}

void DeviceRouter::on_handy_reply(const HandyRequest operation, const int http_status,
                                  const QNetworkReply::NetworkError error, const QString& error_text,
                                  const QByteArray& response) {
    const auto document = QJsonDocument::fromJson(response);
    const auto response_object = document.object();
    const auto response_error = response_object.value("error");
    const auto api_failed = response_object.value("result").toInt() == -1 ||
                            (!response_error.isUndefined() && !response_error.isNull());
    const auto api_error_text = response_error.isObject()
        ? response_error.toObject().value("message").toString()
        : response_error.toString();
    const auto request_failed = error != QNetworkReply::NoError || http_status < 200 || http_status >= 300 || api_failed;
    if (request_failed) {
        if (operation == HandyRequest::StopHamp && !armed_) return;
        handy_state_ = HandyState::Failed;
        handy_started_ = false;
        armed_ = false;
        emit status_changed(tr("Handy request failed: %1").arg(api_error_text.isEmpty() ? error_text : api_error_text), false);
        return;
    }
    if (handy_stop_requested_) {
        const auto may_have_started = handy_started_ || operation == HandyRequest::StartHamp;
        handy_stop_requested_ = false;
        handy_started_ = may_have_started;
        if (may_have_started) request_handy_stop();
        return;
    }
    if (!armed_ || mode_ != Mode::Handy) return;

    switch (operation) {
    case HandyRequest::CheckConnection: {
        if (!document.isObject() || !document.object().value("connected").toBool()) {
            handy_state_ = HandyState::Failed;
            armed_ = false;
            emit status_changed(tr("Handy is not connected in Handyverse"), false);
            return;
        }
        handy_state_ = HandyState::Preparing;
        send_handy_request(HandyRequest::SetHampMode, QStringLiteral("/mode"), QJsonObject{{"mode", 0}});
        break;
    }
    case HandyRequest::SetHampMode:
        handy_state_ = HandyState::Ready;
        emit status_changed(tr("Handy armed: direct connection (HAMP)"), true);
        break;
    case HandyRequest::SetSlide:
        handy_active_min_ = handy_desired_min_;
        handy_active_max_ = handy_desired_max_;
        send_handy_request(HandyRequest::StartHamp, QStringLiteral("/hamp/start"));
        break;
    case HandyRequest::StartHamp:
        handy_started_ = true;
        send_handy_request(HandyRequest::SetVelocity, QStringLiteral("/hamp/velocity"),
                           QJsonObject{{"velocity", handy_desired_velocity_}});
        break;
    case HandyRequest::SetVelocity:
        handy_active_velocity_ = handy_desired_velocity_;
        handy_last_motion_request_ms_ = handy_clock_.elapsed();
        emit status_changed(tr("Handy moving: direct connection (HAMP)"), true);
        break;
    case HandyRequest::StopHamp:
        handy_started_ = false;
        handy_active_connection_key_.clear();
        break;
    case HandyRequest::None:
        break;
    }
}

void DeviceRouter::request_handy_motion() {
    if (!armed_ || mode_ != Mode::Handy || handy_state_ != HandyState::Ready || handy_reply_ != nullptr) return;
    const auto range_changed = handy_desired_min_ != handy_active_min_ || handy_desired_max_ != handy_active_max_;
    const auto velocity_changed = std::abs(handy_desired_velocity_ - handy_active_velocity_) >= 3;
    const auto elapsed = handy_clock_.elapsed() - handy_last_motion_request_ms_;
    if (handy_started_ && !range_changed && (!velocity_changed || elapsed < kHandyRequestIntervalMs)) return;
    if (handy_started_ && elapsed < kHandyRequestIntervalMs) return;
    send_handy_request(HandyRequest::SetSlide, QStringLiteral("/slide"), QJsonObject{
        {"min", handy_desired_min_}, {"max", handy_desired_max_}
    });
}

void DeviceRouter::request_handy_stop() {
    if (handy_reply_ != nullptr) {
        handy_stop_requested_ = true;
        return;
    }
    if (!handy_started_ || handy_active_connection_key_.isEmpty()) return;
    handy_stop_requested_ = false;
    send_handy_request(HandyRequest::StopHamp, QStringLiteral("/hamp/stop"));
}

void DeviceRouter::reset_handy_motion_tracking() {
    handy_state_ = HandyState::Idle;
    handy_started_ = false;
    handy_stop_requested_ = false;
    handy_waiting_for_motion_ = false;
    handy_has_position_ = false;
    handy_previous_position_ = 0.0;
    handy_observed_min_ = 1.0;
    handy_observed_max_ = 0.0;
    handy_desired_min_ = 0;
    handy_desired_max_ = 100;
    handy_desired_velocity_ = 5;
    handy_active_min_ = -1;
    handy_active_max_ = -1;
    handy_active_velocity_ = -1;
    handy_last_motion_request_ms_ = handy_clock_.elapsed() - kHandyRequestIntervalMs;
}

void DeviceRouter::on_intiface_message(const QString& message) {
    const auto document = QJsonDocument::fromJson(message.toUtf8());
    const auto messages = document.isArray() ? document.array() : QJsonArray{document.object()};
    for (const auto& value : messages) {
        const auto object = value.toObject();
        if (object.contains("ServerInfo")) {
            const auto start_scanning = QJsonObject{{"StartScanning", QJsonObject{{"Id", intiface_request_id_++}}}};
            const auto request_device_list = QJsonObject{{"RequestDeviceList", QJsonObject{{"Id", intiface_request_id_++}}}};
            intiface_->sendTextMessage(QString::fromUtf8(QJsonDocument(QJsonArray{start_scanning, request_device_list}).toJson(QJsonDocument::Compact)));
        }
        if (object.contains("DeviceList")) {
            intiface_device_index_ = -1;
            intiface_feature_index_ = -1;
            const auto devices = object.value("DeviceList").toObject().value("Devices").toObject();
            for (auto it = devices.begin(); it != devices.end(); ++it) {
                select_intiface_device(it.value().toObject());
                if (intiface_device_index_ >= 0) break;
            }
        }
        if (object.contains("DeviceAdded")) select_intiface_device(object.value("DeviceAdded").toObject());
    }
}

void DeviceRouter::select_intiface_device(const QJsonObject& device) {
    const auto features = device.value("DeviceFeatures").toObject();
    for (auto it = features.begin(); it != features.end(); ++it) {
        const auto feature = it.value().toObject();
        const auto position = feature.value("Output").toObject().value("Position").toObject();
        const auto range = position.value("Value").toArray();
        if (position.isEmpty() || range.size() != 2) continue;

        const auto minimum = range.at(0).toInt();
        const auto maximum = range.at(1).toInt();
        const auto device_index = device.value("DeviceIndex").toInt(-1);
        const auto feature_index = feature.value("FeatureIndex").toInt(it.key().toInt());
        if (maximum < minimum || device_index < 0 || feature_index < 0) continue;

        intiface_device_index_ = device_index;
        intiface_feature_index_ = feature_index;
        intiface_position_min_ = minimum;
        intiface_position_max_ = maximum;
        reset_output_tracking();
        emit status_changed(tr("Intiface armed: %1 (L0 → first Position feature)").arg(device.value("DeviceName").toString()), true);
        return;
    }
    emit status_changed(tr("Intiface device has no Position feature; output remains disarmed"), false);
}

void DeviceRouter::reset_output_tracking() { last_sent_valid_.fill(false); }

void DeviceRouter::on_intiface_error(const QAbstractSocket::SocketError) {
    emit status_changed(intiface_->errorString(), false);
}
