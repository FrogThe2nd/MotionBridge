#include "handy_position_output.hpp"

#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkRequest>

#include <algorithm>
#include <cmath>

namespace {

constexpr auto kHandyBaseUrl = "https://www.handyfeeling.com/api/handy/v2";
constexpr qint64 kRequestIntervalMs = 120;
constexpr int kPositionThresholdPercent = 1;

QUrl handy_url(const QString& path) {
    return QUrl(QString::fromLatin1(kHandyBaseUrl) + path);
}

} // namespace

HandyPositionOutput::HandyPositionOutput(QObject* parent) : QObject(parent) {
    api_ = new QNetworkAccessManager(this);
    clock_.start();
}

void HandyPositionOutput::set_connection_key(const QString& key) {
    const auto trimmed = key.trimmed();
    if (connection_key_ == trimmed) return;
    emergency_stop();
    connection_key_ = trimmed;
}

void HandyPositionOutput::set_armed(const bool armed) {
    if (!armed) {
        emergency_stop();
        return;
    }
    if (armed_) return;
    if (connection_key_.isEmpty()) {
        armed_ = false;
        emit status_changed(tr("Enter a Handy connection key"), false);
        return;
    }
    armed_ = true;
    active_connection_key_ = connection_key_;
    reset_tracking();
    request_connection_check();
}

bool HandyPositionOutput::armed() const noexcept { return armed_; }

void HandyPositionOutput::send_position(const double normalized_position, const std::chrono::milliseconds interval) {
    if (!armed_ || state_ != State::Ready) return;
    const auto position = std::clamp(static_cast<int>(std::lround(
        std::clamp(normalized_position, 0.0, 1.0) * 100.0)), 0, 100);
    const auto interval_ms = std::max<int64_t>(1, interval.count());
    const auto previous = has_latest_position_ ? latest_position_ : last_sent_position_;
    const auto direction = position > previous ? 1 : position < previous ? -1 : 0;
    const auto speed = std::abs(position - previous) * 1000.0 / static_cast<double>(interval_ms);
    latest_position_ = position;
    latest_velocity_ = direction == 0 ? 0 : direction * std::clamp(static_cast<int>(std::lround(speed)), 5, 100);
    has_latest_position_ = true;
    request_latest_position();
}

void HandyPositionOutput::emergency_stop() {
    armed_ = false;
    if (reply_ != nullptr) {
        stop_requested_ = true;
        return;
    }
    if (active_connection_key_.isEmpty()) return;
    const auto position = has_latest_position_ ? latest_position_ : last_sent_position_;
    send_request(Request::StopPosition, QStringLiteral("/hdsp/xpvp"), QJsonObject{
        {"position", position}, {"velocity", 0}, {"stopOnTarget", true}
    });
}

void HandyPositionOutput::request_connection_check() {
    if (!armed_ || reply_ != nullptr) return;
    state_ = State::CheckingConnection;
    emit status_changed(tr("Checking Handy connection"), false);
    send_request(Request::CheckConnection, QStringLiteral("/connected"));
}

void HandyPositionOutput::send_request(const Request request, const QString& path, const QJsonObject& body) {
    if (reply_ != nullptr || active_connection_key_.isEmpty()) return;
    QNetworkRequest request_data(handy_url(path));
    request_data.setRawHeader("X-Connection-Key", active_connection_key_.toUtf8());
    request_data.setTransferTimeout(8000);
    if (request == Request::CheckConnection) {
        reply_ = api_->get(request_data);
    } else {
        request_data.setHeader(QNetworkRequest::ContentTypeHeader, QStringLiteral("application/json"));
        reply_ = api_->put(request_data, QJsonDocument(body).toJson(QJsonDocument::Compact));
    }
    auto* completed_reply = reply_;
    connect(completed_reply, &QNetworkReply::finished, this, [this, completed_reply, request] {
        const auto status = completed_reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        const auto error = completed_reply->error();
        const auto error_text = completed_reply->errorString();
        const auto response = completed_reply->readAll();
        if (reply_ == completed_reply) reply_ = nullptr;
        completed_reply->deleteLater();
        on_reply(request, status, error, error_text, response);
    });
}

void HandyPositionOutput::on_reply(const Request request, const int http_status,
                                   const QNetworkReply::NetworkError error, const QString& error_text,
                                   const QByteArray& response) {
    const auto document = QJsonDocument::fromJson(response);
    const auto object = document.object();
    const auto response_error = object.value("error");
    const auto api_failed = object.value("result").toInt() < 0 ||
                            (!response_error.isUndefined() && !response_error.isNull());
    const auto api_error_text = response_error.isObject()
        ? response_error.toObject().value("message").toString()
        : response_error.toString();
    if (error != QNetworkReply::NoError || http_status < 200 || http_status >= 300 || api_failed) {
        if (request == Request::StopPosition && !armed_) return;
        state_ = State::Failed;
        armed_ = false;
        emit status_changed(tr("Handy request failed: %1").arg(api_error_text.isEmpty() ? error_text : api_error_text), false);
        return;
    }
    if (stop_requested_) {
        stop_requested_ = false;
        emergency_stop();
        return;
    }
    if (request == Request::StopPosition) {
        active_connection_key_.clear();
        state_ = State::Idle;
        return;
    }
    if (!armed_) return;

    switch (request) {
    case Request::CheckConnection:
        if (!document.isObject() || !object.value("connected").toBool()) {
            state_ = State::Failed;
            armed_ = false;
            emit status_changed(tr("Handy is not connected in Handyverse"), false);
            return;
        }
        state_ = State::Preparing;
        send_request(Request::SetHdspMode, QStringLiteral("/mode"), QJsonObject{{"mode", 2}});
        break;
    case Request::SetHdspMode:
        state_ = State::Ready;
        emit status_changed(tr("Handy armed: direct position control"), true);
        break;
    case Request::SetPosition:
        last_sent_position_ = in_flight_position_;
        last_sent_velocity_ = in_flight_velocity_;
        last_request_ms_ = clock_.elapsed();
        emit status_changed(tr("Handy following L0: direct position control"), true);
        break;
    case Request::StopPosition:
        break;
    }
}

void HandyPositionOutput::request_latest_position() {
    if (!armed_ || state_ != State::Ready || reply_ != nullptr || !has_latest_position_) return;
    const auto delta = std::abs(latest_position_ - last_sent_position_);
    const auto direction_changed = latest_velocity_ != 0 && last_sent_velocity_ != 0 &&
                                   (latest_velocity_ > 0) != (last_sent_velocity_ > 0);
    if (delta < kPositionThresholdPercent && !direction_changed) return;
    if (!direction_changed && clock_.elapsed() - last_request_ms_ < kRequestIntervalMs) return;
    in_flight_position_ = latest_position_;
    in_flight_velocity_ = latest_velocity_;
    send_request(Request::SetPosition, QStringLiteral("/hdsp/xpvp"), QJsonObject{
        {"position", in_flight_position_}, {"velocity", in_flight_velocity_}, {"stopOnTarget", false}
    });
}

void HandyPositionOutput::reset_tracking() {
    state_ = State::Idle;
    stop_requested_ = false;
    has_latest_position_ = false;
    latest_position_ = 0;
    latest_velocity_ = 0;
    in_flight_position_ = 0;
    in_flight_velocity_ = 0;
    last_sent_position_ = 0;
    last_sent_velocity_ = 0;
    last_request_ms_ = clock_.elapsed() - kRequestIntervalMs;
}
