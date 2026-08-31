#pragma once

#include "motion_bridge/types.hpp"

#include <QJsonObject>
#include <QObject>
#include <QSerialPort>
#include <QUdpSocket>
#include <QWebSocket>

#include <array>
#include <chrono>

class HandyPositionOutput;

class DeviceRouter final : public QObject {
    Q_OBJECT

public:
    enum class Mode { None, Usb, Wifi, Intiface, Handy };
    Q_ENUM(Mode)

    explicit DeviceRouter(QObject* parent = nullptr);
    void set_mode(Mode mode);
    void set_usb_port(const QString& port);
    void set_wifi_endpoint(const QString& host, quint16 port);
    void set_intiface_url(const QUrl& url);
    void set_handy_connection_key(const QString& key);
    void set_armed(bool armed);
    [[nodiscard]] Mode mode() const noexcept;
    [[nodiscard]] bool armed() const noexcept;
    void send(const motion_bridge::Axes& axes, std::chrono::milliseconds interval);
    void emergency_stop();

signals:
    void status_changed(const QString& text, bool connected);

private slots:
    void on_intiface_message(const QString& message);
    void on_intiface_error(QAbstractSocket::SocketError error);

private:
    void ensure_transport();
    void send_output(const motion_bridge::Axes& axes, std::chrono::milliseconds interval, bool force_full = false);
    void send_intiface_zero();
    void reset_output_tracking();
    void select_intiface_device(const QJsonObject& device);

    Mode mode_{Mode::None};
    bool armed_{};
    QString usb_port_;
    QString wifi_host_{"tcode.local"};
    quint16 wifi_port_{8000};
    QUrl intiface_url_{"ws://127.0.0.1:12345"};
    QSerialPort* serial_{};
    QUdpSocket* udp_{};
    QWebSocket* intiface_{};
    HandyPositionOutput* handy_{};
    int intiface_request_id_{1};
    int intiface_device_index_{-1};
    int intiface_feature_index_{-1};
    int intiface_position_min_{};
    int intiface_position_max_{100};
    motion_bridge::Axes last_sent_axes_;
    std::array<bool, 6> last_sent_valid_{};
};
