#include "scannerbase.h"

ScannerBase::ScannerBase(QObject* parent, QSettings* parent_settings)
	: QObject(parent)
{
	settings = parent_settings;
}

void ScannerBase::setSettings(QSettings* settings_in)
{
	settings = settings_in;
}
