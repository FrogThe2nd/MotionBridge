#include "handy_position_output.hpp"

#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkRequest>

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

constexpr auto kHandyBaseUrl = "https://www.handyfeeling.com/api/handy/v2";
constexpr qint64 kRequestIntervalMs = 120;
constexpr int kMinimumDurationMs = 60;
constexpr int kMaximumDurationMs = 1000;
constexpr double kPositionThresholdPercent = 0.1;

} // namespace

HandyPositionOutput::HandyPositionOutput(QObject* parent, QUrl api_base_url)
    : QObject(parent), api_base_url_(std::move(api_base_url)) {
    if (api_base_url_.isEmpty()) api_base_url_ = QUrl(QString::fromLatin1(kHandyBaseUrl));
    auto base_url = api_base_url_.toString();
    while (base_url.endsWith(u'/')) base_url.chop(1);
    api_base_url_ = QUrl(base_url);
    api_ = new QNetworkAccessManager(this);
    send_timer_.setSingleShot(true);
    connect(&send_timer_, &QTimer::timeout, this, &HandyPositionOutput::request_latest_position);
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
    if (!armed_) return;
    latest_position_ = std::clamp(normalized_position, 0.0, 1.0) * 100.0;
    latest_interval_ms_ = std::clamp(static_cast<int>(std::max<int64_t>(1, interval.count())),
                                     kMinimumDurationMs, kMaximumDurationMs);
    has_latest_position_ = true;
    request_latest_position();
}

void HandyPositionOutput::emergency_stop() {
    armed_ = false;
    state_ = State::Idle;
    send_timer_.stop();
    ++request_generation_;
    if (reply_ != nullptr) {
        auto* stale_reply = reply_;
        reply_ = nullptr;
        stale_reply->abort();
    }
    active_connection_key_.clear();
    has_latest_position_ = false;
}

void HandyPositionOutput::request_connection_check() {
    if (!armed_ || reply_ != nullptr) return;
    state_ = State::CheckingConnection;
    emit status_changed(tr("Checking Handy connection"), false);
    send_request(Request::CheckConnection, QStringLiteral("/connected"));
}

void HandyPositionOutput::send_request(const Request request, const QString& path, const QJsonObject& body) {
    if (reply_ != nullptr || active_connection_key_.isEmpty()) return;
    QNetworkRequest request_data(QUrl(api_base_url_.toString() + path));
    request_data.setRawHeader("X-Connection-Key", active_connection_key_.toUtf8());
    request_data.setRawHeader("Accept", "application/json");
    request_data.setTransferTimeout(8000);
    if (request == Request::CheckConnection || request == Request::CheckInfo) {
        reply_ = api_->get(request_data);
    } else {
        request_data.setHeader(QNetworkRequest::ContentTypeHeader, QStringLiteral("application/json"));
        reply_ = api_->put(request_data, QJsonDocument(body).toJson(QJsonDocument::Compact));
    }
    auto* completed_reply = reply_;
    const auto generation = request_generation_;
    connect(completed_reply, &QNetworkReply::finished, this, [this, completed_reply, request, generation] {
        const auto status = completed_reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        const auto error = completed_reply->error();
        const auto error_text = completed_reply->errorString();
        const auto response = completed_reply->readAll();
        if (reply_ == completed_reply) reply_ = nullptr;
        completed_reply->deleteLater();
        if (generation != request_generation_) return;
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
        state_ = State::Failed;
        armed_ = false;
        emit status_changed(tr("Handy request failed: %1").arg(api_error_text.isEmpty() ? error_text : api_error_text), false);
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
        emit status_changed(tr("Checking Handy firmware"), false);
        send_request(Request::CheckInfo, QStringLiteral("/info"));
        break;
    case Request::CheckInfo:
        if (!document.isObject() || !object.value("fwStatus").isDouble() ||
            object.value("fwStatus").toInt() == 1) {
            state_ = State::Failed;
            armed_ = false;
            emit status_changed(tr("Handy firmware needs an update"), false);
            return;
        }
        send_request(Request::SetHdspMode, QStringLiteral("/mode"), QJsonObject{{"mode", 2}});
        break;
    case Request::SetHdspMode:
        state_ = State::Ready;
        emit status_changed(tr("Handy armed: timed L0 position control"), true);
        request_latest_position();
        break;
    case Request::SetPosition:
        last_sent_position_ = in_flight_position_;
        has_sent_position_ = true;
        emit status_changed(tr("Handy following L0: timed position control"), true);
        request_latest_position();
        break;
    }
}

void HandyPositionOutput::request_latest_position() {
    if (!armed_ || state_ != State::Ready || reply_ != nullptr || !has_latest_position_) return;
    const auto delta = std::abs(latest_position_ - last_sent_position_);
    if (has_sent_position_ && delta < kPositionThresholdPercent) return;

    const auto now = clock_.elapsed();
    const auto elapsed_since_request = now - last_request_ms_;
    if (has_sent_position_ && elapsed_since_request < kRequestIntervalMs) {
        send_timer_.start(static_cast<int>(kRequestIntervalMs - elapsed_since_request));
        return;
    }

    in_flight_position_ = latest_position_;
    const auto duration_basis = has_sent_position_ ? elapsed_since_request
                                                   : std::max<qint64>(latest_interval_ms_, kRequestIntervalMs);
    in_flight_duration_ms_ = std::clamp(static_cast<int>(duration_basis),
                                        kMinimumDurationMs, kMaximumDurationMs);
    last_request_ms_ = now;
    send_request(Request::SetPosition, QStringLiteral("/hdsp/xpt"), QJsonObject{
        {"immediateResponse", true}, {"stopOnTarget", true},
        {"duration", in_flight_duration_ms_}, {"position", in_flight_position_}
    });
}

void HandyPositionOutput::reset_tracking() {
    state_ = State::Idle;
    send_timer_.stop();
    has_latest_position_ = false;
    has_sent_position_ = false;
    latest_position_ = 0;
    latest_interval_ms_ = 20;
    in_flight_position_ = 0;
    in_flight_duration_ms_ = 0;
    last_sent_position_ = 0;
    last_request_ms_ = clock_.elapsed() - kRequestIntervalMs;
}
