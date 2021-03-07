#ifndef SCANNERWIDGET_H
#define SCANNERWIDGET_H

#include <QCheckBox>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSettings>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QWidget>
#include <QtWidgets/QMainWindow>

#include "core/reconstruction/reconstructioninterface.h"
#include "io/openniinterface.h"
#include "utility/tools.h"

class ScannerWidget : public QMainWindow {
    Q_OBJECT

public:
    ScannerWidget(QWidget* parent = 0);

private:
    QSettings* settings;
    QString settingsPath;

    QWidget* centralWidget;
    QStatusBar* statusBar;
    QVBoxLayout* vBoxLayout;

    QPushButton* makeProjectButton;
    QPushButton* openProjectButton;

    QPushButton* initButton;
    QCheckBox* recCheck;
    QCheckBox* streamFromCheck;
    QCheckBox* recToPclDataCheck;
    QCheckBox* undistCheck;
    QCheckBox* bilateralCheck;

    QPushButton* takeImagesButton;
    QPushButton* takeOpImagesButton;
    QPushButton* takeOneOpImageButton;
    QPushButton* saveDataButton;

    QPushButton* drawScene3dModelButton;
    QCheckBox* reconstructCheck;
    QCheckBox* undistrtionCheck;
    QCheckBox* bilateralFilterCheck;
    QCheckBox* statFilterCheck;
    QCheckBox* mlsFilterCheck;

    OpenNiInterface* openniInterface;
    ReconstructionInterface* reconstructionInterface;

    void reloadSettings();
    void initializeSettings();
    void initializeReconstruction();
    void initializeMainInterfaceSettings();

    void initializeMainInterface();
    void initializeOpenDialogInterface();

    void initializeDebugInterface();
    void initializeReleaseInterface();

    void makeProject();

signals:
    void signal_write_com_port(char);

public slots:
    void slot_make_project();
    void slot_open_project();

    void slot_start_stream();
    void slot_start_rotation_stream();
    void slot_take_long_images();
    void slot_take_one_long_image();
    void slot_save_long_image_data();
    void slot_perform_reconstruction();

    void slot_record_stream(int);
    void slot_replay_recording(int);
    void slot_record_pcd(int);
    void slot_use_undistortion(int);
    void slot_use_bilateral(int);
};

#endif // SCANNERWIDGET_H
