#include "core/base/scannerbase.h"

ScannerBase::ScannerBase(QObject* parent, QSettings* parent_settings)
    : QObject(parent)
    , configs("configs.ini", QSettings::IniFormat)
{
    settings = parent_settings;
}

void ScannerBase::setSettings(QSettings* settings_in)
{
    settings = settings_in;
}
