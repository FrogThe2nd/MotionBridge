#include "handy_position_output.hpp"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QHash>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPointer>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThread>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>

namespace {

struct RecordedRequest {
    QByteArray method;
    QByteArray path;
    QByteArray connection_key;
    QJsonObject body;
};

class MockHandyServer final : public QObject {
public:
    bool start() {
        connect(&server_, &QTcpServer::newConnection, this, [this] {
            while (server_.hasPendingConnections()) {
                auto* socket = server_.nextPendingConnection();
                buffers_.insert(socket, {});
                connect(socket, &QTcpSocket::readyRead, this, [this, socket] { read_requests(socket); });
            }
        });
        return server_.listen(QHostAddress::LocalHost, 0);
    }

    [[nodiscard]] QUrl base_url() const {
        return QUrl(QStringLiteral("http://127.0.0.1:%1/api/handy/v2").arg(server_.serverPort()));
    }

    [[nodiscard]] const QList<RecordedRequest>& requests() const { return requests_; }

private:
    static int content_length(const QByteArray& header) {
        for (const auto& line : header.split('\n')) {
            const auto trimmed = line.trimmed();
            static const QByteArray prefix("Content-Length:");
            if (trimmed.left(prefix.size()).compare(prefix, Qt::CaseInsensitive) != 0) continue;
            return trimmed.mid(prefix.size()).trimmed().toInt();
        }
        return 0;
    }

    static QByteArray header_value(const QByteArray& header, const QByteArray& name) {
        for (const auto& line : header.split('\n')) {
            const auto trimmed = line.trimmed();
            const auto prefix = name + ':';
            if (trimmed.left(prefix.size()).compare(prefix, Qt::CaseInsensitive) != 0) continue;
            return trimmed.mid(name.size() + 1).trimmed();
        }
        return {};
    }

    void read_requests(QTcpSocket* socket) {
        auto& buffer = buffers_[socket];
        buffer += socket->readAll();
        while (true) {
            const auto header_end = buffer.indexOf("\r\n\r\n");
            if (header_end < 0) return;
            const auto header = buffer.left(header_end);
            const auto body_length = content_length(header);
            const auto request_length = header_end + 4 + body_length;
            if (buffer.size() < request_length) return;

            const auto body = buffer.mid(header_end + 4, body_length);
            const auto request_line = header.left(header.indexOf("\r\n")).split(' ');
            RecordedRequest request;
            if (request_line.size() >= 2) {
                request.method = request_line.at(0);
                request.path = request_line.at(1);
            }
            request.connection_key = header_value(header, "X-Connection-Key");
            request.body = QJsonDocument::fromJson(body).object();
            requests_.append(request);
            buffer.remove(0, request_length);

            QByteArray response_body = R"({"result":0})";
            if (request.path.endsWith("/connected")) response_body = R"({"connected":true})";
            if (request.path.endsWith("/info")) response_body = R"({"fwStatus":0})";
            const auto delay_ms = request.path.endsWith("/hdsp/xpt") && xpt_count_++ == 0 ? 180 : 0;
            QPointer<QTcpSocket> guarded_socket(socket);
            QTimer::singleShot(delay_ms, this, [guarded_socket, response_body] {
                if (!guarded_socket) return;
                const auto response = QByteArray("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: ") +
                                      QByteArray::number(response_body.size()) +
                                      "\r\nConnection: keep-alive\r\n\r\n" + response_body;
                guarded_socket->write(response);
                guarded_socket->flush();
            });
        }
    }

    QTcpServer server_;
    QHash<QTcpSocket*, QByteArray> buffers_;
    QList<RecordedRequest> requests_;
    int xpt_count_{};
};

bool wait_until(const std::function<bool()>& predicate, const int timeout_ms) {
    QElapsedTimer timeout;
    timeout.start();
    while (!predicate() && timeout.elapsed() < timeout_ms) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        QThread::msleep(2);
    }
    return predicate();
}

int fail(const char* message) {
    std::cerr << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char** argv) {
    QCoreApplication application(argc, argv);
    MockHandyServer server;
    if (!server.start()) return fail("mock Handy server failed to start");

    HandyPositionOutput output(nullptr, server.base_url());
    bool ready = false;
    QObject::connect(&output, &HandyPositionOutput::status_changed,
                     [&ready](const QString&, const bool connected) { ready = ready || connected; });
    output.set_connection_key(QStringLiteral("test-key"));
    output.set_armed(true);
    output.send_position(0.10125, std::chrono::milliseconds{20});

    if (!wait_until([&ready] { return ready; }, 2000)) return fail("Handy handshake did not complete");
    if (!wait_until([&server] {
            return std::count_if(server.requests().cbegin(), server.requests().cend(),
                                 [](const auto& request) { return request.path.endsWith("/hdsp/xpt"); }) >= 1;
        }, 1000)) return fail("first timed position request was not sent");

    output.send_position(0.35, std::chrono::milliseconds{20});
    output.send_position(0.8037, std::chrono::milliseconds{20});
    if (!wait_until([&server] {
            return std::count_if(server.requests().cbegin(), server.requests().cend(),
                                 [](const auto& request) { return request.path.endsWith("/hdsp/xpt"); }) >= 2;
        }, 2000)) return fail("latest L0 target was not sent after the in-flight request");

    QList<RecordedRequest> timed_positions;
    for (const auto& request : server.requests()) {
        if (request.path.contains("xpvp")) return fail("legacy velocity endpoint was used");
        if (request.connection_key != "test-key") return fail("connection key header was missing");
        if (request.path.endsWith("/hdsp/xpt")) timed_positions.append(request);
    }
    const QList<QByteArray> expected_paths{
        "/api/handy/v2/connected", "/api/handy/v2/info", "/api/handy/v2/mode"
    };
    for (const auto& path : expected_paths) {
        if (std::none_of(server.requests().cbegin(), server.requests().cend(),
                         [&path](const auto& request) { return request.path == path; })) {
            return fail("Handy handshake request was missing");
        }
    }
    if (timed_positions.size() < 2 ||
        std::abs(timed_positions.at(0).body.value("position").toDouble() - 10.125) > 0.0001 ||
        std::abs(timed_positions.at(1).body.value("position").toDouble() - 80.37) > 0.0001) {
        return fail("in-flight updates were not coalesced to the latest L0 position");
    }
    for (const auto& request : timed_positions) {
        const auto duration = request.body.value("duration").toInt();
        if (!request.body.value("immediateResponse").toBool() ||
            !request.body.value("stopOnTarget").toBool() || duration < 60 || duration > 1000 ||
            request.body.contains("velocity")) {
            return fail("timed position payload did not match the MultiFunPlayer-compatible shape");
        }
    }

    output.emergency_stop();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    return 0;
}
