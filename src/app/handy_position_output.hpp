#pragma once

#include <QElapsedTimer>
#include <QJsonObject>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QObject>
#include <QTimer>
#include <QUrl>

#include <chrono>

// Handy's cloud API is too slow for the 50 Hz TCode stream. This output owns
// its own latest-value scheduler and sends HDSP target positions instead.
class HandyPositionOutput final : public QObject {
    Q_OBJECT

public:
    explicit HandyPositionOutput(QObject* parent = nullptr, QUrl api_base_url = {});

    void set_connection_key(const QString& key);
    void set_armed(bool armed);
    void send_position(double normalized_position, std::chrono::milliseconds interval);
    void emergency_stop();

    [[nodiscard]] bool armed() const noexcept;

signals:
    void status_changed(const QString& text, bool connected);

private:
    enum class State { Idle, CheckingConnection, Preparing, Ready, Failed };
    enum class Request { CheckConnection, CheckInfo, SetHdspMode, SetPosition };

    void request_connection_check();
    void send_request(Request request, const QString& path, const QJsonObject& body = {});
    void on_reply(Request request, int http_status, QNetworkReply::NetworkError error,
                  const QString& error_text, const QByteArray& response);
    void request_latest_position();
    void reset_tracking();

    QString connection_key_;
    QString active_connection_key_;
    QUrl api_base_url_;
    QNetworkAccessManager* api_{};
    QNetworkReply* reply_{};
    QTimer send_timer_;
    State state_{State::Idle};
    bool armed_{};
    bool has_latest_position_{};
    bool has_sent_position_{};
    double latest_position_{};
    int latest_interval_ms_{20};
    double in_flight_position_{};
    int in_flight_duration_ms_{};
    double last_sent_position_{};
    qint64 last_request_ms_{-1000};
    quint64 request_generation_{};
    QElapsedTimer clock_;
};
