#pragma once

#include "motion_bridge/types.hpp"

#include <QJsonObject>
#include <QElapsedTimer>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QObject>
#include <QSerialPort>
#include <QUdpSocket>
#include <QWebSocket>

#include <array>
#include <chrono>

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
    enum class HandyState { Idle, CheckingConnection, Preparing, Ready, Failed };
    enum class HandyRequest { None, CheckConnection, SetHampMode, SetSlide, StartHamp, SetVelocity, StopHamp };

    void ensure_transport();
    void send_output(const motion_bridge::Axes& axes, std::chrono::milliseconds interval, bool force_full = false);
    void send_intiface_zero();
    void send_handy_output(const motion_bridge::Axes& axes, std::chrono::milliseconds interval);
    void request_handy_connection_check();
    void send_handy_request(HandyRequest operation, const QString& path, const QJsonObject& body = {});
    void on_handy_reply(HandyRequest operation, int http_status, QNetworkReply::NetworkError error,
                        const QString& error_text, const QByteArray& response);
    void request_handy_motion();
    void request_handy_stop();
    void reset_handy_motion_tracking();
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
    QNetworkAccessManager* handy_api_{};
    int intiface_request_id_{1};
    int intiface_device_index_{-1};
    int intiface_feature_index_{-1};
    int intiface_position_min_{};
    int intiface_position_max_{100};
    QString handy_connection_key_;
    QString handy_active_connection_key_;
    QNetworkReply* handy_reply_{};
    HandyState handy_state_{HandyState::Idle};
    bool handy_started_{};
    bool handy_stop_requested_{};
    bool handy_waiting_for_motion_{};
    bool handy_has_position_{};
    double handy_previous_position_{};
    double handy_observed_min_{1.0};
    double handy_observed_max_{};
    int handy_desired_min_{};
    int handy_desired_max_{100};
    int handy_desired_velocity_{5};
    int handy_active_min_{-1};
    int handy_active_max_{-1};
    int handy_active_velocity_{-1};
    qint64 handy_last_motion_request_ms_{-1000};
    QElapsedTimer handy_clock_;
    motion_bridge::Axes last_sent_axes_;
    std::array<bool, 6> last_sent_valid_{};
};
