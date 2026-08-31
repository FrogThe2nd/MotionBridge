#include "motion_bridge_settings.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>

QSettings motion_bridge_settings(const QString& application_directory) {
    const auto local_data = QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation);
    const auto legacy_directory = QDir(local_data).filePath("FallenDollTCode");
    const auto legacy_path = QDir(legacy_directory).filePath("companion.ini");
    const auto settings_directory = QDir(local_data).filePath("MotionBridge");
    const auto settings_path = QDir(settings_directory).filePath("motion-bridge.ini");
    const auto resolved_application_directory = application_directory.isEmpty()
        ? QCoreApplication::applicationDirPath()
        : application_directory;
    const auto portable = QFileInfo::exists(QDir(resolved_application_directory).filePath("portable.mode"));
    if (!portable) {
        QDir().mkpath(settings_directory);
        if (!QFileInfo::exists(settings_path) && QFileInfo::exists(legacy_path)) QFile::copy(legacy_path, settings_path);
        return QSettings(settings_path, QSettings::IniFormat);
    }

    const auto directory = QDir(resolved_application_directory).filePath("config");
    const auto path = QDir(directory).filePath("motion-bridge.ini");
    const auto legacy_portable_path = QDir(directory).filePath("companion.ini");
    QDir().mkpath(directory);
    if (!QFileInfo::exists(path)) {
        if (QFileInfo::exists(legacy_portable_path)) QFile::copy(legacy_portable_path, path);
        else if (QFileInfo::exists(legacy_path)) QFile::copy(legacy_path, path);
    }
    return QSettings(path, QSettings::IniFormat);
}

int normalize_ui_scale_percent(const int value) noexcept {
    switch (value) {
    case 75:
    case 90:
    case 100:
    case 110:
    case 125:
        return value;
    default:
        return 0;
    }
}
