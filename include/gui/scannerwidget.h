#ifndef SCANNERWIDGET_H
#define SCANNERWIDGET_H

#include <QtWidgets/QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSlider>
#include <QCheckBox>
#include <QStatusBar>

#include <QString>

#include <QSettings>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QDebug>

#include <core/reconstruction/reconstructioninterface.h>
#include <io/openniinterface.h>
#include <utility/tools.h>

class ScannerWidget : public QMainWindow
{
	Q_OBJECT

public:
	ScannerWidget(QWidget *parent = 0);

private:
	QSettings*   settings;
	QString		 settingsPath;

	QWidget*	 centralWidget;
	QStatusBar*  statusBar;
	QVBoxLayout* vBoxLayout;

	QPushButton* makeProjectButton;
	QPushButton* openProjectButton;

	QPushButton* initButton;
	QCheckBox*	 recCheck;
	QCheckBox*	 streamFromCheck;
	QCheckBox*	 recToPclDataCheck;
	QCheckBox*	 undistCheck;
	QCheckBox*	 bilateralCheck;

	QPushButton* takeImagesButton;
	QPushButton* takeOpImagesButton;
	QPushButton* takeOneOpImageButton;
	QPushButton* saveDataButton;

	QPushButton* drawScene3dModelButton;
	QCheckBox*	 reconstructCheck;
	QCheckBox*   undistrtionCheck;
	QCheckBox*	 bilateralFilterCheck;
	QCheckBox*	 statFilterCheck;
	QCheckBox*	 mlsFilterCheck;
	
	QSlider* slider;

	OpenNiInterface* openniInterface;
	ReconstructionInterface* reconstructionInterface;

	void initializeSettings();
	void initializeReconstruction();

	void initializeMainInterface();
	void initializeOpenDialogInterface();
	void initializeDebugInterface();
	void initializeReleaseInterface();

	void makeProject();

public slots:
	void slot_make_project();
	void slot_open_project();

	void slot_initialize();

	void slot_take_images();
	void slot_take_op_images();
	void slot_take_one_op_image();
	void slot_save_data();

	void slot_draw_scene3d_model();	
	void slot_change_pair(int);

signals:
	void signal_take_one_image();
	void signal_write_com_port(char);

};

#endif // SCANNERWIDGET_H
