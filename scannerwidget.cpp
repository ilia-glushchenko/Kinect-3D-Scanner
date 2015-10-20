#include "scannerwidget.h"

ScannerWidget::ScannerWidget(QWidget *parent)
	:QMainWindow(parent)
{	
	initializeOpenDialogInterface();
}

//###############################################################

void ScannerWidget::initializeSettings()
{
	settings = new QSettings(settingsPath, QSettings::IniFormat, this);
}

void ScannerWidget::initializeReconstruction()
{
	openniInterface = new OpenNiInterface(this, settings);
	reconstructionInterface = new ReconstructionInterface(this, settings);
}

void ScannerWidget::initializeOpenDialogInterface()
{
	setWindowTitle("No project opened...");

	makeProjectButton = new QPushButton("Make Project");
	connect(makeProjectButton, SIGNAL(clicked()), this, SLOT(slot_make_project()));
	openProjectButton = new QPushButton("Open Project");
	connect(openProjectButton, SIGNAL(clicked()), this, SLOT(slot_open_project()));

	QWidget* centralWidget = new QWidget(this);
	QVBoxLayout* vBoxLayout = new QVBoxLayout(this);
	vBoxLayout->addWidget(makeProjectButton);
	vBoxLayout->addWidget(openProjectButton);
	centralWidget->setLayout(vBoxLayout);
	setCentralWidget(centralWidget);
}

void ScannerWidget::initializeDebugInterface()
{
	QWidget* centralWidget = new QWidget(this);

	statusBar = new QStatusBar();
	statusBar->showMessage("Started");
	setStatusBar(statusBar);

	makeProjectButton = new QPushButton("Make Project");
	connect(makeProjectButton, SIGNAL(clicked()), this, SLOT(slot_make_project()));
	openProjectButton = new QPushButton("Open Project");
	connect(openProjectButton, SIGNAL(clicked()), this, SLOT(slot_open_project()));

	initButton = new QPushButton("Start Stream");
	connect(initButton, SIGNAL(clicked()),
		this, SLOT(slot_initialize()));

	recCheck = new QCheckBox("Recording streams");
	streamFromCheck = new QCheckBox("Stream from recording");
	recToPclDataCheck = new QCheckBox("Save recording as PCL data");
	undistCheck = new QCheckBox("Undistortion");
	bilateralCheck = new QCheckBox("Use Bilateral filter");
	connect(recCheck, SIGNAL(stateChanged(int)),
		openniInterface, SLOT(slot_set_record_stream(int)));
	connect(streamFromCheck, SIGNAL(stateChanged(int)),
		openniInterface, SLOT(slot_set_stream_from_record(int)));
	connect(recToPclDataCheck, SIGNAL(stateChanged(int)),
		openniInterface, SLOT(slot_set_record_to_pcd(int)));
	connect(undistCheck, SIGNAL(stateChanged(int)),
		openniInterface, SLOT(slot_set_stream_undistortion(int)));
	connect(bilateralCheck, SIGNAL(stateChanged(int)),
		openniInterface, SLOT(slot_set_stream_bilateral(int)));
	streamFromCheck->setChecked(true);

	takeImagesButton = new QPushButton("Take Images");
	takeOpImagesButton = new QPushButton("Take OP Images");
	takeOneOpImageButton = new QPushButton("Take One OP Image");
	saveDataButton = new QPushButton("Save Data");
	connect(takeImagesButton, SIGNAL(clicked()),
		this, SLOT(slot_take_images()));
	connect(takeOpImagesButton, SIGNAL(clicked()),
		this, SLOT(slot_take_op_images()));
	connect(takeOneOpImageButton, SIGNAL(clicked()),
		this, SLOT(slot_take_one_op_image()));
	connect(saveDataButton, SIGNAL(clicked()),
		this, SLOT(slot_save_data()));

	drawScene3dModelButton = new QPushButton("Draw 3D model");
	connect(drawScene3dModelButton, SIGNAL(clicked()),
		this, SLOT(slot_draw_scene3d_model()));

	reconstructCheck = new QCheckBox("Reconstruct");
	undistrtionCheck = new QCheckBox("Undistortion");
	bilateralFilterCheck = new QCheckBox("Bilateral filter");
	statFilterCheck = new QCheckBox("Statistic filter");
	mlsFilterCheck = new QCheckBox("Smooth filter");
	connect(reconstructCheck, SIGNAL(stateChanged(int)),
		reconstructionInterface, SLOT(slot_set_use_reconstruction(int)));
	connect(undistrtionCheck, SIGNAL(stateChanged(int)),
		reconstructionInterface, SLOT(slot_set_use_undistortion(int)));
	connect(bilateralFilterCheck, SIGNAL(stateChanged(int)),
		reconstructionInterface, SLOT(slot_set_use_bilateral_filter(int)));
	connect(statFilterCheck, SIGNAL(stateChanged(int)),
		reconstructionInterface, SLOT(slot_set_use_statistical_outlier_removal_filter(int)));
	connect(mlsFilterCheck, SIGNAL(stateChanged(int)),
		reconstructionInterface, SLOT(slot_set_use_moving_least_squares_filter(int)));
	reconstructCheck->setChecked(true);
	undistrtionCheck->setChecked(true);
	bilateralFilterCheck->setChecked(true);

	//------------------------------------------------
	vBoxLayout = new QVBoxLayout();

	vBoxLayout->addWidget(makeProjectButton);
	vBoxLayout->addWidget(openProjectButton);
	vBoxLayout->addWidget(initButton);
	vBoxLayout->addWidget(recCheck);
	vBoxLayout->addWidget(streamFromCheck);
	vBoxLayout->addWidget(recToPclDataCheck);
	vBoxLayout->addWidget(undistCheck);
	vBoxLayout->addWidget(bilateralCheck);

	vBoxLayout->addWidget(takeImagesButton);
	vBoxLayout->addWidget(takeOpImagesButton);
	vBoxLayout->addWidget(takeOneOpImageButton);
	vBoxLayout->addWidget(saveDataButton);

	vBoxLayout->addWidget(drawScene3dModelButton);
	vBoxLayout->addWidget(reconstructCheck);
	vBoxLayout->addWidget(undistrtionCheck);
	vBoxLayout->addWidget(bilateralFilterCheck);
	vBoxLayout->addWidget(statFilterCheck);
	vBoxLayout->addWidget(mlsFilterCheck);
	
	centralWidget->setLayout(vBoxLayout);
	setCentralWidget(centralWidget);

	slider = new QSlider(Qt::Horizontal);
	slider->setTickInterval(1);
	slider->setSingleStep(1);
	slider->setFixedWidth(600);
	connect(slider, SIGNAL(valueChanged(int)),
		this, SLOT(slot_change_pair(int)));
}

void ScannerWidget::initializeReleaseInterface()
{
	centralWidget = new QWidget(this);

	makeProjectButton = new QPushButton("Make Project");
	connect(makeProjectButton, SIGNAL(clicked()), this, SLOT(slot_make_project()));
	openProjectButton = new QPushButton("Open Project");
	connect(openProjectButton, SIGNAL(clicked()), this, SLOT(slot_open_project()));
	initButton = new QPushButton("Perform scanning");
	saveDataButton = new QPushButton("Save scanning");
	drawScene3dModelButton = new QPushButton("Reconstruct scanning");

	vBoxLayout = new QVBoxLayout(this);
	vBoxLayout->addWidget(makeProjectButton);
	vBoxLayout->addWidget(openProjectButton);
	vBoxLayout->addWidget(initButton);
	vBoxLayout->addWidget(saveDataButton);
	vBoxLayout->addWidget(drawScene3dModelButton);
	
	centralWidget->setLayout(vBoxLayout);
	setCentralWidget(centralWidget);
}

//###############################################################
void ScannerWidget::slot_make_project()
{
	QString makeProjectPath = QFileDialog::getExistingDirectory(this, "Select new project directory...", "./");
	if (makeProjectPath.isEmpty() == false)
	{
		QDir dir(makeProjectPath);
		if (!dir.exists()) {
			dir.mkdir(".");
		}
		
		//Create calibartion folder
		tools::copyRecursively("./default_project/calibration", QString("%1/calibration").arg(makeProjectPath));

		//Create stream folder
		dir = QDir(QString("%1/stream").arg(makeProjectPath));
		if (!dir.exists()) {
			dir.mkdir(".");
		}

		//Create pcd folder
		dir = QDir(QString("%1/pcd").arg(makeProjectPath));
		if (!dir.exists()) {
			dir.mkdir(".");
		}

		QFile projectFile("./default_project/project.ini");
		if (projectFile.exists()) { 
			projectFile.copy(makeProjectPath + "/project.ini");
		}
		else {
			qDebug() << "Error: Default project does not exists!";
		}
		
	}
}

void ScannerWidget::slot_open_project()
{
	QString tempSettingsPath = QFileDialog::getOpenFileName(this, "Select project ini file...", "./", "*.ini");
	if (tempSettingsPath.isEmpty() == false)
	{
		settingsPath = tempSettingsPath;
		initializeSettings();
		initializeReconstruction();

		setWindowTitle(QString("Project: %1").arg(settings->value("PROJECT_SETTINGS").toString()));
		if (settings->value("PROJECT_SETTINGS/DEBUG_INTERFACE").toBool()) {
			initializeDebugInterface();
		}
		else {
			initializeReleaseInterface();
		}
	}
}


void ScannerWidget::slot_initialize()
{
	if (!openniInterface->isInit())
	{
		openniInterface->initialize_interface();

		if (openniInterface->isInit())
		{
			recCheck->setDisabled(true);
			streamFromCheck->setDisabled(true);
			recToPclDataCheck->setDisabled(true);

			initButton->setText("Stop Stream");
			openniInterface->start_stream();
		}
	}
	else
	{
		openniInterface->shutdown_interface();

		recCheck->setDisabled(false);
		streamFromCheck->setDisabled(false);
		recToPclDataCheck->setDisabled(false);

		initButton->setText("Start Stream");
	}
}

void ScannerWidget::slot_take_images()
{
	if (!openniInterface->isInit())
	{
		openniInterface->initialize_interface();

		if (openniInterface->isInit())
		{
			recCheck->setDisabled(true);
			streamFromCheck->setDisabled(true);
			recToPclDataCheck->setDisabled(true);

			initButton->setText("Stop");
			openniInterface->start_rotation_stream();
		}
	}
	else
	{
		openniInterface->shutdown_interface();

		recCheck->setDisabled(false);
		streamFromCheck->setDisabled(false);
		recToPclDataCheck->setDisabled(false);

		initButton->setText("Take Images");
	}
}

void ScannerWidget::slot_take_op_images()
{
	if (!openniInterface->isInit())
	{
		openniInterface->initialize_interface();

		if (openniInterface->isInit())
		{
			recCheck->setDisabled(true);
			streamFromCheck->setDisabled(true);
			recToPclDataCheck->setDisabled(true);

			initButton->setText("Stop");
			openniInterface->start_iterative_rotation_stream();
		}
	}
	else
	{
		openniInterface->shutdown_interface();

		recCheck->setDisabled(false);
		streamFromCheck->setDisabled(false);
		recToPclDataCheck->setDisabled(false);

		initButton->setText("Start stream");
	}
}

void ScannerWidget::slot_take_one_op_image()
{
	openniInterface->initialize_interface();
	if (openniInterface->isInit())
	{
		recCheck->setDisabled(true);
		streamFromCheck->setDisabled(true);
		recToPclDataCheck->setDisabled(true);

		openniInterface->take_one_optimized_image();

		recCheck->setDisabled(false);
		streamFromCheck->setDisabled(false);
		recToPclDataCheck->setDisabled(false);

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	openniInterface->shutdown_interface();
}

void ScannerWidget::slot_save_data()
{
	openniInterface->save_optimized_images();
}

void ScannerWidget::slot_draw_scene3d_model()
{
	reconstructionInterface->slot_perform_reconstruction();

	if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool() == false)
		if (slider->isHidden())
			slider->show();
}

void ScannerWidget::slot_change_pair(int index)
{
	reconstructionInterface->slot_change_pair(index);
}