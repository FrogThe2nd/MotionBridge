#pragma once

#include <QElapsedTimer>
#include <QJsonObject>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QObject>

#include <chrono>

// Handy's cloud API is too slow for the 50 Hz TCode stream. This output owns
// its own latest-value scheduler and sends HDSP target positions instead.
class HandyPositionOutput final : public QObject {
    Q_OBJECT

public:
    explicit HandyPositionOutput(QObject* parent = nullptr);

    void set_connection_key(const QString& key);
    void set_armed(bool armed);
    void send_position(double normalized_position, std::chrono::milliseconds interval);
    void emergency_stop();

    [[nodiscard]] bool armed() const noexcept;

signals:
    void status_changed(const QString& text, bool connected);

private:
    enum class State { Idle, CheckingConnection, Preparing, Ready, Failed };
    enum class Request { CheckConnection, SetHdspMode, SetPosition, StopPosition };

    void request_connection_check();
    void send_request(Request request, const QString& path, const QJsonObject& body = {});
    void on_reply(Request request, int http_status, QNetworkReply::NetworkError error,
                  const QString& error_text, const QByteArray& response);
    void request_latest_position();
    void reset_tracking();

    QString connection_key_;
    QString active_connection_key_;
    QNetworkAccessManager* api_{};
    QNetworkReply* reply_{};
    State state_{State::Idle};
    bool armed_{};
    bool stop_requested_{};
    bool has_latest_position_{};
    int latest_position_{};
    int latest_velocity_{};
    int in_flight_position_{};
    int in_flight_velocity_{};
    int last_sent_position_{};
    int last_sent_velocity_{};
    qint64 last_request_ms_{-1000};
    QElapsedTimer clock_;
};
