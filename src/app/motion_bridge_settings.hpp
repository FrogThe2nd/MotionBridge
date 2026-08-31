#pragma once

#include <QSettings>
#include <QString>

QSettings motion_bridge_settings(const QString& application_directory = {});
int normalize_ui_scale_percent(int value) noexcept;
