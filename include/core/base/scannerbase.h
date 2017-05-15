#ifndef BASESCANNER_H
#define BASESCANNER_H

#include <QObject>
#include <QSettings>

class ScannerBase : public QObject {
    Q_OBJECT

public:
    ScannerBase(QObject* parent, QSettings* parent_settings);
    void setSettings(QSettings* settings_in);

protected:
    QSettings* settings;
    QSettings configs;
};

#endif // BASESCANNER_H
