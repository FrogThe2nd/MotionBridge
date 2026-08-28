#include "motion_bridge_controller.hpp"
#include "language_controller.hpp"
#include "obj_geometry.hpp"

#include <QGuiApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QIcon>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <qqml.h>

#ifdef Q_OS_WIN
#include <windows.h>
#include <dbghelp.h>
#endif

namespace {
void startup_message_handler(QtMsgType, const QMessageLogContext&, const QString& message) {
    QFile log(QDir(QDir::tempPath()).filePath("MotionBridge-startup.log"));
    if (!log.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) return;
    log.write(QDateTime::currentDateTime().toString(Qt::ISODate).toUtf8());
    log.write(" ");
    log.write(message.toUtf8());
    log.write("\n");
}

#ifdef Q_OS_WIN
LONG WINAPI write_crash_dump(EXCEPTION_POINTERS* exception) {
    wchar_t temporary_path[MAX_PATH]{};
    const auto length = GetTempPathW(MAX_PATH, temporary_path);
    if (length == 0 || length >= MAX_PATH) return EXCEPTION_EXECUTE_HANDLER;

    const auto directory = std::wstring(temporary_path) + L"MotionBridge-crashes";
    CreateDirectoryW(directory.c_str(), nullptr);

    SYSTEMTIME time{};
    GetLocalTime(&time);
    wchar_t filename[MAX_PATH]{};
    swprintf_s(filename, L"%s\\MotionBridge-%04u%02u%02u-%02u%02u%02u-%lu.dmp",
               directory.c_str(), time.wYear, time.wMonth, time.wDay,
               time.wHour, time.wMinute, time.wSecond, GetCurrentProcessId());
    const HANDLE file = CreateFileW(filename, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
                                    FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file == INVALID_HANDLE_VALUE) return EXCEPTION_EXECUTE_HANDLER;

    MINIDUMP_EXCEPTION_INFORMATION exception_information{};
    exception_information.ThreadId = GetCurrentThreadId();
    exception_information.ExceptionPointers = exception;
    exception_information.ClientPointers = FALSE;
    MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), file,
                      MiniDumpNormal, &exception_information, nullptr, nullptr);
    CloseHandle(file);
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif
}

int main(int argc, char* argv[]) {
#ifdef Q_OS_WIN
    SetUnhandledExceptionFilter(write_crash_dump);
#endif
    qInstallMessageHandler(startup_message_handler);
    qputenv("QT_QUICK_CONTROLS_STYLE", "Basic");
    QGuiApplication app(argc, argv);
    QCoreApplication::setOrganizationName("Motion Bridge");
    QCoreApplication::setApplicationName("Motion Bridge");
    QCoreApplication::setOrganizationDomain("motionbridge.local");
    app.setWindowIcon(QIcon(QStringLiteral(
        ":/qt/qml/MotionBridge/App/assets/icons/motion-bridge.svg")));

    LanguageController language_controller(&app);
    MotionBridgeController controller;
    qmlRegisterType<ObjGeometry>("MotionBridge.Native", 1, 0, "ObjGeometry");
    QQmlApplicationEngine engine;
    language_controller.set_engine(&engine);
    engine.rootContext()->setContextProperty("companion", &controller);
    engine.rootContext()->setContextProperty("languageController", &language_controller);
    const QUrl url(QStringLiteral("qrc:/qt/qml/MotionBridge/App/qml/Main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed, &app, [] { QCoreApplication::exit(-1); }, Qt::QueuedConnection);
    engine.load(url);
    return app.exec();
}
