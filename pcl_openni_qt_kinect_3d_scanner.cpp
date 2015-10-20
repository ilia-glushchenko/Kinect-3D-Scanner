#include "pcl_openni_qt_kinect_3d_scanner.h"

#define WIDTH  640 
#define HEIGHT 480 

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

PCL_OpenNI_Qt_Kinect_3d_Scanner::PCL_OpenNI_Qt_Kinect_3d_Scanner(
	QWidget *parent
	)
{
	setParent(parent);

	settings = new QSettings("scaner.ini", QSettings::IniFormat, this);

	openniInterface		 = new OpenNiInterface(this);
	volumeReconstruction = new VolumeReconstruction(this);

	//------------------------------------------------

	statusBar = new QStatusBar();
	statusBar->showMessage("Started");
	setWindowTitle("Main window");
	setStatusBar(statusBar);	

	initButton = new QPushButton("Start Stream");
	connect(initButton, SIGNAL(clicked()),
		this, SLOT(slot_initialize()));

	recCheck = new QCheckBox("Recording streams");
	connect(recCheck,		 SIGNAL(stateChanged(int)),
			openniInterface, SLOT(slot_set_record_stream(int)));
	streamFromCheck = new QCheckBox("Stream from recording");
	connect(streamFromCheck, SIGNAL(stateChanged(int)),
			openniInterface, SLOT  (slot_set_stream_from_record(int)));
	streamFromCheck->setChecked(true);
	recToPclDataCheck = new QCheckBox("Save recording as PCL data");
	connect(recToPclDataCheck, SIGNAL(stateChanged(int)),
			openniInterface, SLOT(slot_set_record_to_pcd(int)));
	undistCheck	= new QCheckBox("Undistortion");
	connect(undistCheck,     SIGNAL(stateChanged(int)),
			openniInterface, SLOT(slot_set_stream_undistortion(int)));
	bilateralCheck = new QCheckBox("Use Bilateral filter");
	connect(bilateralCheck,  SIGNAL(stateChanged(int)),
			openniInterface, SLOT(slot_set_stream_bilateral(int)));
	
	takeImagesButton = new QPushButton("Take Images");
	connect(takeImagesButton, SIGNAL(clicked()),
		this, SLOT(slot_take_images()));
	takeOpImagesButton = new QPushButton("Take OP Images");
	connect(takeOpImagesButton, SIGNAL(clicked()),
		this, SLOT(slot_take_op_images()));
	takeOneOpImageButton = new QPushButton("Take One OP Image");
	connect(takeOneOpImageButton, SIGNAL(clicked()),
		this, SLOT(slot_take_one_op_image()));

	drawScene3dModelButton = new QPushButton("Draw 3D model");
	connect(drawScene3dModelButton, SIGNAL(clicked()),
		this, SLOT(slot_draw_scene3d_model()));

	icpCheck			 = new QCheckBox("Reconstruct");
	undistrtionCheck	 = new QCheckBox("Undistortion");
	bilateralFilterCheck = new QCheckBox("Bilateral filter");
	removeNanCheck		 = new QCheckBox("Remove NaN");
	statFilterCheck		 = new QCheckBox("Statistic filter");
	approxFilterCheck	 = new QCheckBox("Smooth filter");
	icpCheck			->setChecked(true);
	undistrtionCheck	->setChecked(true);
	bilateralFilterCheck->setChecked(true);
	removeNanCheck		->setChecked(true);
	statFilterCheck		->setChecked(true);
	

	saveDataButton = new QPushButton("Save Data");
	connect(saveDataButton, SIGNAL(clicked()),
		this, SLOT(slot_save_data()));

	readDataButton = new QPushButton("Read Data");
	connect(readDataButton, SIGNAL(clicked()),
		this, SLOT(slot_read_data()));

	//------------------------------------------------
	vBoxLayout = new QVBoxLayout();
	vBoxLayout->addWidget(initButton);
	vBoxLayout->addWidget(recCheck);
	vBoxLayout->addWidget(streamFromCheck);
	vBoxLayout->addWidget(recToPclDataCheck);
	vBoxLayout->addWidget(undistCheck);
	vBoxLayout->addWidget(bilateralCheck);

	vBoxLayout->addWidget(takeImagesButton);
	vBoxLayout->addWidget(takeOpImagesButton);
	vBoxLayout->addWidget(takeOneOpImageButton);
	vBoxLayout->addWidget(drawScene3dModelButton);
	vBoxLayout->addWidget(icpCheck);
	vBoxLayout->addWidget(undistrtionCheck);
	vBoxLayout->addWidget(bilateralFilterCheck);
	vBoxLayout->addWidget(removeNanCheck);
	vBoxLayout->addWidget(statFilterCheck);
	vBoxLayout->addWidget(approxFilterCheck);
	
	vBoxLayout->addWidget(saveDataButton);
	vBoxLayout->addWidget(readDataButton);

	widget = new QWidget();
	widget->setLayout(vBoxLayout);

	setCentralWidget(widget);

	//--------------------PCL GUI-------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> tmp_viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer = tmp_viewer;
	viewer.get()->setBackgroundColor(
		settings->value("VISUALIZATION/BG_R").toDouble() / 255.0f,
		settings->value("VISUALIZATION/BG_G").toDouble() / 255.0f,
		settings->value("VISUALIZATION/BG_B").toDouble() / 255.0f
		);

	
	add_viewer_debug_text();

	if (settings->value("VISUALIZATION/DRAW_AXIS").toBool())
		viewer.get()->addCoordinateSystem(settings->value("VISUALIZATION/AXIS_SIZE").toFloat());
	viewer.get()->initCameraParameters();
	viewer.get()->spinOnce(100);

	//--------------------------------------------------
	slider = new QSlider(Qt::Horizontal);
	slider->setTickInterval(1);
	slider->setSingleStep(1);
	slider->setFixedWidth(600);
	connect(slider, SIGNAL(valueChanged(int)),
			this,   SLOT(slot_change_pair(int)));
}

//#######################################################
//#---------------------SAVE READ------------------------

void PCL_OpenNI_Qt_Kinect_3d_Scanner::save_data()
{
	openniInterface->save_optimized_images();
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::read_data()
{
	prepare_read();

	int from = settings->value("READING_SETTING/FROM").toInt();
	int to   = settings->value("READING_SETTING/TO").toInt();
	int step = settings->value("READING_SETTING/STEP").toInt();

	qDebug() << "Start reading data...";
	//Read point clouds
	for (int i = from; i <= to; i += step)
	{
		PcdPtr point_cloud_ptr(new Pcd);
				
		qDebug() << "Reading"
				 << settings->value("READING_PATTERNS/POINT_CLOUD_NAME").toString().arg(i);
		PclIO::load_one_point_cloud(
			settings->value(
				"READING_PATTERNS/POINT_CLOUD_NAME"
			).toString().arg(i),
			point_cloud_ptr
		);

		if (point_cloud_ptr.get()->points.empty())
			continue;
		
		point_cloud_vector.push_back(point_cloud_ptr);

		QString filename_pattern =
			settings->value("READING_PATTERNS/POINT_CLOUD_IMAGE_NAME").toString();
		cv::Mat img = cv::imread(filename_pattern.arg(i).toStdString());
		qDebug() << "Reading" << filename_pattern.arg(i);
		colorImagesMatVector.push_back(img);
	}

	//Init final trans matrix
	for (int i = 0; i < point_cloud_vector.size(); i++)
	{
		Eigen::Matrix4f empty_matrix = Eigen::Matrix4f::Identity();
		final_translation_matrix_vector.push_back(empty_matrix);
	}

	qDebug() << "Done!";
	qDebug() << "Total:" << point_cloud_vector.size();
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::prepare_read()
{
	colorImagesMatVector.clear();
	point_cloud_vector.clear();
	cloudIndexesVector.clear();

	keypoint_point_cloud_vector.clear();
	correspondences_vector.clear();

	icp_matrix_vector.clear();

	sac_translation_matrix_vector.clear();
	icp_translation_matrix_vector.clear();
	final_translation_matrix_vector.clear();

	afterThreshNanMatchesImagesVector.clear();

	openniInterface->deleteLater();
	volumeReconstruction->deleteLater();

	openniInterface		 = new OpenNiInterface(this);
	volumeReconstruction = new VolumeReconstruction(this);
}

//#######################################################
//#------------------RECONSTRUCTION----------------------

//----------------------Filters--------------------------
void PCL_OpenNI_Qt_Kinect_3d_Scanner::apply_bilateral_filter(
	PcdPtr src_point_cloud_ptr
	)
{
	cv::Mat img, img_res;
	img_res.create(HEIGHT, WIDTH, CV_32FC1);
	img.create(HEIGHT, WIDTH, CV_32FC1);

	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
			if (pcl_isnan(src_point_cloud_ptr.get()->points[x + y*WIDTH].z))
				img.at<float>(y, x) = 0.0f;
			else
				img.at<float>(y, x) = static_cast<float>(src_point_cloud_ptr.get()->points[x + y*WIDTH].z);

	int d =
		settings->value(
		"OPENCV_BILATERAL_FILTER_SETTINGS/D"
		).toInt();
	double sigma_color =
		settings->value(
		"OPENCV_BILATERAL_FILTER_SETTINGS/SIGMA_COLOR"
		).toDouble();
	double sigma_space =
		settings->value(
		"OPENCV_BILATERAL_FILTER_SETTINGS/SIGMA_SPACE"
		).toDouble();

	cv::bilateralFilter(img, img_res, d, sigma_color, sigma_space);

	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
			if (img_res.at<float>(y, x) == 0)
				src_point_cloud_ptr.get()->points[x + y*WIDTH].z = NAN;
			else
				src_point_cloud_ptr.get()->points[x + y*WIDTH].z = img_res.at<float>(y, x);
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::apply_statistical_outlier_removal_filter(
	PcdPtr in_point_cloud_ptr,
	PcdPtr out_point_cloud_ptr
	)
{
	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setInputCloud(in_point_cloud_ptr);
	sor.setKeepOrganized(true);
	sor.setMeanK(
		settings->value(
			"STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/MEAN_K"
		).toInt()
	);
	sor.setStddevMulThresh(
		settings->value(
			"STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/MUL_THRESH"
		).toFloat()
	);
	sor.filter(*out_point_cloud_ptr.get());

	if (settings->value(
			"STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/ENABLE_LOG"
		).toBool())
	{
		qDebug() << "Statistical removal: from "
				 << out_point_cloud_ptr.get()->points.size()
				 << "to"
				 << in_point_cloud_ptr.get()->points.size();
	}

}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::apply_moving_least_squares_filter(
	PcdPtr point_cloud_ptr	
	)
{
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
	pcl::PointCloud<pcl::PointNormal> mls_points;
	pcl::MovingLeastSquares<PointType, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(point_cloud_ptr);
	mls.setPolynomialFit(true);
	mls.setSqrGaussParam(
		settings->value(
			"MOVING_LEAST_SQUARES_FILTER_SETTINGS/SQR_GAUSS_PARAM"
		).toFloat()
	);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(
		settings->value(
			"MOVING_LEAST_SQUARES_FILTER_SETTINGS/SEARCH_RADIUS"
		).toFloat()
	);
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::apply_voxel_grid_reduction(
	PcdPtr in_point_cloud_ptr,
	PcdPtr out_point_cloud_ptr,
	float x_k, float y_k, float z_k
	)
{
	float leaf_x = settings->value("VOXEL_GRID_REDUCTION_SETTINGS/LEAF_X").toFloat();
	float leaf_y = settings->value("VOXEL_GRID_REDUCTION_SETTINGS/LEAF_Y").toFloat();
	float leaf_z = settings->value("VOXEL_GRID_REDUCTION_SETTINGS/LEAF_Z").toFloat();

	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud(in_point_cloud_ptr);
	sor.setLeafSize(leaf_x * x_k, leaf_y * y_k, leaf_z * z_k);
	sor.filter(*out_point_cloud_ptr.get());
	qDebug() << "Reduced from" 
			 << in_point_cloud_ptr.get()->points.size() 
			 << "to" 
			 << out_point_cloud_ptr.get()->points.size();
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::filter_point_cloud(
	PcdPtr src_point_cloud_ptr,
	PcdPtr dest_point_cloud_ptr
	)
{
	//Bilateral
	if (bilateralFilterCheck->isChecked())	
		apply_bilateral_filter(src_point_cloud_ptr);
	
	//Remove Nan
	if (removeNanCheck->isChecked()) {			
		vector<int> indexesVector;
		pcl::removeNaNFromPointCloud(*src_point_cloud_ptr.get(), *dest_point_cloud_ptr.get(), indexesVector);
		cloudIndexesVector.push_back(indexesVector);
	}
	
	//Statistic reduction
	if (statFilterCheck->isChecked()) {	
		apply_statistical_outlier_removal_filter(dest_point_cloud_ptr, src_point_cloud_ptr);
		pcl::copyPointCloud(*src_point_cloud_ptr.get(), *dest_point_cloud_ptr.get());
	}

	//Smooth
	if (approxFilterCheck->isChecked())
		apply_moving_least_squares_filter(dest_point_cloud_ptr);
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::merging_all_point_clouds(
	PcdPtrVector point_cloud_vector,
	PcdPtr in_point_cloud_ptr
	)
{
	qDebug() << "Merging point clouds...";
	for (int i = point_cloud_vector.size() - 1; i >= 0; i--)
	{
		*in_point_cloud_ptr.get() += *point_cloud_vector.at(i).get();
		point_cloud_vector.pop_back();
	}

	point_cloud_vector.clear();
	qDebug() << "Done!";
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::remove_nan_from_all_clouds()
{
	cloudIndexesVector.clear();

	qDebug() << "Removing NaN's from point clouds...";
	for (int i = 0; i < point_cloud_vector.size(); i++)
	{
		PcdPtr		tmp(new Pcd);
		vector<int> indexesVector;
		point_cloud_vector[i].get()->is_dense = false;
		pcl::removeNaNFromPointCloud(*point_cloud_vector[i].get(), *tmp.get(), indexesVector);
		pcl::copyPointCloud(*tmp.get(), *point_cloud_vector[i].get());
		cloudIndexesVector.push_back(indexesVector);
	}
	qDebug() << "Done!";
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::filter_all_point_clouds()
{
	qDebug() << "Applying filters for point clouds...";
	qDebug() << "Applying undistortion";
	if (undistrtionCheck->isChecked())
	{
		CalibrationInterface calibrationInterface(this);
		calibrationInterface.loadCalibrationData();
		calibrationInterface.calibrate();
		calibrationInterface.undistort(point_cloud_vector);
	}
	for (int i = 0; i < point_cloud_vector.size(); i++)
	{
		qDebug() << QString("Applying filters %1/%2").arg(i + 1).arg(point_cloud_vector.size());
		PcdPtr filtered_point_cloud_ptr(new Pcd);
		filter_point_cloud(point_cloud_vector[i], filtered_point_cloud_ptr);
		pcl::copyPointCloud(*filtered_point_cloud_ptr.get(), *point_cloud_vector[i].get());
	}
	qDebug() << "Done!";
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::apply_voxel_grid_reduction_to_all_point_clouds()
{
	qDebug() << "Applying reduction to all point clouds...";
	for (int i = 0; i < point_cloud_vector.size(); i++)
	{
		qDebug() << QString("Reduction %1/%2").arg(i + 1).arg(point_cloud_vector.size());
		PcdPtr tmp(new Pcd);
		apply_voxel_grid_reduction(point_cloud_vector[i], tmp);
		point_cloud_vector[i] = tmp;
	}
	qDebug() << "Done!";
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::reorganize_all_point_clouds()
{
	qDebug() << "Reorganization all point clouds";
	for (int i = 0; i < cloudIndexesVector.size(); i++)
	{
		PcdPtr tmp_pdc_ptr(new Pcd);
		tmp_pdc_ptr.get()->width  = WIDTH;
		tmp_pdc_ptr.get()->height = HEIGHT;
		tmp_pdc_ptr.get()->resize(HEIGHT*WIDTH);

		for (int y = 0; y < HEIGHT; y++)
			for (int x = 0; x < WIDTH; x++)
			{
				tmp_pdc_ptr.get()->at(x, y).x = 0;
				tmp_pdc_ptr.get()->at(x, y).y = 0;
				tmp_pdc_ptr.get()->at(x, y).z = NAN;
				tmp_pdc_ptr.get()->at(x, y).r = 0;
				tmp_pdc_ptr.get()->at(x, y).g = 0;
				tmp_pdc_ptr.get()->at(x, y).b = 0;
			}

		for (int j = 0; j < point_cloud_vector[i].get()->size(); j++)
			tmp_pdc_ptr.get()->points[cloudIndexesVector[i][j]] = point_cloud_vector[i].get()->points[j];

		pcl::copyPointCloud(*tmp_pdc_ptr.get(), *point_cloud_vector[i].get());
		point_cloud_vector[i].get()->is_dense = false;
	}
	qDebug() << "Done!";
}

//----------------------Vizualizer-------------------------

void PCL_OpenNI_Qt_Kinect_3d_Scanner::add_viewer_debug_text()
{
	if (settings->value("FINAL_SETTINGS/DRAW_ALL_CAMERA_POSES").toBool())
		visualize_all_camera_poses();

	if (settings->value("VISUALIZATION/DEBUG_INI_ENABLE").toBool())
	{
		QString allGroupsString;
		foreach(const QString &group, settings->childGroups()) {
			if (settings->value(
				QString("%1/ENABLE_IN_VISUALIZATION").arg(group)
				).toBool()
				)
			{
				QString groupString = QString("%1 \n").arg(group);
				settings->beginGroup(group);

				foreach(const QString &key, settings->childKeys()) {
					if (key != "ENABLE_IN_VISUALIZATION")
						groupString.append(QString("%1: %2; ").arg(key, settings->value(key).toString()));
				}

				settings->endGroup();
				groupString.append("\n\n");

				allGroupsString.append(groupString);
			}
		}
		viewer.get()->addText(
			allGroupsString.toStdString(),
			settings->value("VISUALIZATION/DEBUG_INI_X").toInt(),
			settings->value("VISUALIZATION/DEBUG_INI_Y").toInt(),
			settings->value("VISUALIZATION/DEBUG_INI_FONTSIZE").toInt(),
			settings->value("VISUALIZATION/DEBUG_INI_R").toDouble() / 255.0f,
			settings->value("VISUALIZATION/DEBUG_INI_G").toDouble() / 255.0f,
			settings->value("VISUALIZATION/DEBUG_INI_B").toDouble() / 255.0f
		);
	}

}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::set_viewer_pose(
	pcl::visualization::PCLVisualizer& viewer,
	const Eigen::Affine3f& viewer_pose
	)
{
	Eigen::Vector3f pos_vector = viewer_pose            * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(
		pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]
		);
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::visualize_point_clouds(
	PcdPtrVector point_cloud_vector
	)
{
	for (int i = 0; i < point_cloud_vector.size(); i++)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(point_cloud_vector[i]);
		viewer.get()->addPointCloud<PointType>(point_cloud_vector[i], rgb, QString("Cloud #%1").arg(i).toStdString());
	}

}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::visualize_all_keypoint_clouds()
{
	if (keypoint_point_cloud_vector.empty() == false)
	{
		for (int i = 0; i < keypoint_point_cloud_vector.size(); i++)
		{
			viewer.get()->addCorrespondences<PointType>(
				keypoint_point_cloud_vector[i].first,
				keypoint_point_cloud_vector[i].second,
				correspondences_vector[i],
				QString("Cor %1").arg(i).toStdString()
			);
		}

		if (settings->value("FINAL_SETTINGS/DRAW_FIRST_LAST_KP").toBool())
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color2(keypoint_point_cloud_vector[0].first, 0, 0, 0);
			viewer.get()->addPointCloud<PointType>(keypoint_point_cloud_vector[0].first, single_color2, QString("Cloud P1 %1").arg(0).toStdString());
			viewer.get()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, QString("Cloud P1 %1").arg(0).toStdString());

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color3(keypoint_point_cloud_vector[keypoint_point_cloud_vector.size() - 1].second, 0, 255, 0);
			viewer.get()->addPointCloud<PointType>(keypoint_point_cloud_vector[keypoint_point_cloud_vector.size() - 1].second,
				single_color3, QString("Cloud P2 %1").arg(keypoint_point_cloud_vector.size() - 1).toStdString()
				);
			viewer.get()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, QString("Cloud P2 %1").arg(keypoint_point_cloud_vector.size() - 1).toStdString());
		}

		for (int i = 1; i < keypoint_point_cloud_vector.size(); i++)
		{
			int r1, g1, b1;
			if (i % 2 == 0)
			{
				r1 = 255.0f * ((float)(i + 1) / (float)keypoint_point_cloud_vector.size());
				g1 = 0;
				b1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypoint_point_cloud_vector.size());
			}
			else
			{
				r1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypoint_point_cloud_vector.size());
				g1 = 0;
				b1 = 255.0f * ((float)(i + 1) / (float)keypoint_point_cloud_vector.size()); 
			}

			if (settings->value("FINAL_SETTINGS/DRAW_SECOND").toBool())
			{
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color1(keypoint_point_cloud_vector[i - 1].second, r1, g1, b1);
				viewer.get()->addPointCloud<PointType>(keypoint_point_cloud_vector[i - 1].second, single_color1, QString("Cloud P2 %1").arg(i - 1).toStdString());
				viewer.get()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, QString("Cloud P2 %1").arg(i - 1).toStdString());
			}

			if (settings->value("FINAL_SETTINGS/DRAW_FIRST").toBool())
			{
				if (settings->value("FINAL_SETTINGS/CHANGE_PARE_COLOR").toBool())
				{
					if (i % 2 == 0)
					{
						r1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypoint_point_cloud_vector.size());
						b1 = 255.0f * ((float)(i + 1) / (float)keypoint_point_cloud_vector.size());
					}
					else
					{
						r1 = 255.0f * ((float)(i + 1) / (float)keypoint_point_cloud_vector.size());
						b1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypoint_point_cloud_vector.size()); 
					}
				}

				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(keypoint_point_cloud_vector[i].first, r1, g1, b1);
				viewer.get()->addPointCloud<PointType>(keypoint_point_cloud_vector[i].first, single_color, QString("Cloud P1 %1").arg(i).toStdString());
				viewer.get()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, QString("Cloud P1 %1").arg(i).toStdString());
			}
		}
	}
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::visualize_all_camera_poses()
{
	float radius = settings->value("FINAL_SETTINGS/CAMERA_SPHERE_RADIUS").toFloat();
	for (int i = 0; i < final_translation_matrix_vector.size(); i++)
	{
		PointType pC;
		pC.x = final_translation_matrix_vector[i](0, 3);
		pC.y = final_translation_matrix_vector[i](1, 3);
		pC.z = final_translation_matrix_vector[i](2, 3);

		viewer.get()->addSphere(pC, radius, QString("%1").arg(i).toStdString());
	}
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::visualize_mesh()
{
	//Run CPU_TSDF and reconstruc volume
	volumeReconstruction->calculate(
		point_cloud_vector,
		&final_translation_matrix_vector
	);

	//Cleare viewer
	viewer.get()->removeAllShapes();
	viewer.get()->removeAllPointClouds();
	add_viewer_debug_text();

	//Make mesh and draw it
	if (settings->value("CPU_TSDF_SETTINGS/DRAW_MESH").toBool())
	{
		pcl::PolygonMesh mesh;
		volumeReconstruction->getPoligonMesh(mesh);
		viewer.get()->addPolygonMesh(mesh);

		//Draw volume cude
		if (settings->value("CPU_TSDF_SETTINGS/DRAW_VOLUME_CUBE").toBool())
		{
			double x_vol = settings->value("CPU_TSDF_SETTINGS/X_VOL").toDouble();
			double y_vol = settings->value("CPU_TSDF_SETTINGS/Y_VOL").toDouble();
			double z_vol = settings->value("CPU_TSDF_SETTINGS/Z_VOL").toDouble();
			double x_shift = settings->value("CPU_TSDF_SETTINGS/X_SHIFT").toDouble();
			double y_shift = settings->value("CPU_TSDF_SETTINGS/Y_SHIFT").toDouble();
			double z_shift = settings->value("CPU_TSDF_SETTINGS/Z_SHIFT").toDouble();
			viewer.get()->addCube(
				-(x_vol / 2.0f) + x_shift, (x_vol / 2.0f) + x_shift,
				-(y_vol / 2.0f) + y_shift, (y_vol / 2.0f) + y_shift,
				-(z_vol / 2.0f) + z_shift, (z_vol / 2.0f) + z_shift
			);
		}
	}
}

//----------------------KeyPoints--------------------------

void PCL_OpenNI_Qt_Kinect_3d_Scanner::calculate_keypoint_clouds(
	PcdPtr cloud_ptr1,
	PcdPtr cloud_ptr2,
	cv::Mat img1,
	cv::Mat img2,
	PcdPtr keypoint_cloud_ptr1,
	PcdPtr keypoint_cloud_ptr2
	)
{
	bool aruco = settings->value("KEYPOINT_SEARCH_SETTINGS/USE_ARUCO").toBool();
	bool surf  = settings->value("KEYPOINT_SEARCH_SETTINGS/USE_SURF").toBool();

	//Find keypoints
	if (aruco == true)
	{
		ArUcoKeypointDetector aruco(
			this, cloud_ptr1, cloud_ptr2, img1, img2, keypoint_cloud_ptr1, keypoint_cloud_ptr2
		);
		aruco.detect();
	}

	if (surf == true)
	{
		SurfKeypointDetector surf(
			this, cloud_ptr1, cloud_ptr2, img1, img2, keypoint_cloud_ptr1, keypoint_cloud_ptr2
		);
		surf.detect();
		surf.getMatchImagesVector(&afterThreshNanMatchesImagesVector);
	}

	//Fill correspondences
	if (keypoint_cloud_ptr1.get()->width > 0 &&
		keypoint_cloud_ptr2.get()->width > 0)
	{
		pcl::Correspondences correspondences;
		for (int i = 0; i < keypoint_cloud_ptr1.get()->points.size() - 1; i++)
		{
			pcl::Correspondence correspondence;
			correspondence.index_query = i;
			correspondence.index_match = i;
			PointType a = keypoint_cloud_ptr1.get()->points[i];
			PointType b = keypoint_cloud_ptr2.get()->points[i];
			correspondence.distance = sqrtf(powf(a.x-b.x, 2)+powf(a.y-b.y, 2)+powf(a.z-b.z, 2));
			correspondences.push_back(correspondence);
		}
		correspondences_vector.push_back(correspondences);
	}
	else
	{
		qDebug() << "ERROR: 0 Keypoints found!";
	}
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::calculate_all_keypoint_pairs()
{
	qDebug() << "Calculating keypoint pairs...";
	for (int i = 1; i < colorImagesMatVector.size(); i++)
	{
		qDebug() << QString("Calculating pairs %1/%2").arg(i).arg(colorImagesMatVector.size() - 1);

		PcdPtr point_cloud_ptr1(new Pcd);
		PcdPtr point_cloud_ptr2(new Pcd);

		calculate_keypoint_clouds(
			point_cloud_vector  [i - 1], point_cloud_vector  [i],
			colorImagesMatVector[i - 1], colorImagesMatVector[i],
			point_cloud_ptr1,		     point_cloud_ptr2
		);

		keypoint_point_cloud_vector.push_back(std::make_pair(point_cloud_ptr1, point_cloud_ptr2));
	}
	qDebug() << "Done!";

	if (settings->value(
		"OPENCV_KEYPOINT_DETECTION_SETTINGS/DRAW_GOOD_FILTERED_MATCHES"
		).toBool())
	{
		if (!afterThreshNanMatchesImagesVector.empty())
			ImagesViewerWidget* viewer1 = new ImagesViewerWidget(afterThreshNanMatchesImagesVector, "After NaN & Thresh");
	}
}

//------------------------SaC------------------------------

void PCL_OpenNI_Qt_Kinect_3d_Scanner::perform_sac_registration()
{
	SaCRegistration sac(this);
	sac.calculateSaCTransformation(
		keypoint_point_cloud_vector,
		correspondences_vector
	);
	sac.getTransformation(sac_translation_matrix_vector);

	for (int i = 0; i < final_translation_matrix_vector.size(); i++)
		final_translation_matrix_vector[i] *= sac_translation_matrix_vector[i];

	if (!settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool() &&
		 settings->value("SAC_SETTINGS/APPLY_IN_FINAL").toBool())
	{
		sac.applyTransfomation(point_cloud_vector);
	}	
}

//------------------------ICP------------------------------

void PCL_OpenNI_Qt_Kinect_3d_Scanner::calculate_icp(
	PcdPtr input_cloud_ptr,
	PcdPtr target_cloud_ptr
	)
{
	PcdPtr output_cloud_ptr(new Pcd);

	int maxIter 
		= settings->value("ICP_SETTINGS/MAX_ITERATIONS").toInt();
	double trans_epsilon 
		= settings->value("ICP_SETTINGS/TRANSFORMATION_EPSILON").toDouble();
	double euclid_epsilon 
		= settings->value("ICP_SETTINGS/EUCLIDEAN_EPSILON").toDouble();

	pcl::IterativeClosestPointNonLinear<PointType, PointType> icp;
	icp.setInputCloud(input_cloud_ptr);
	icp.setInputTarget(target_cloud_ptr);
	icp.setMaximumIterations(maxIter);
	icp.setTransformationEpsilon(trans_epsilon);
	icp.setEuclideanFitnessEpsilon(euclid_epsilon);
	icp.align(*output_cloud_ptr.get());

	icp_matrix_vector.push_back(icp.getFinalTransformation());

	if (icp.hasConverged())
	{
		pcl::copyPointCloud(*output_cloud_ptr.get(), *input_cloud_ptr.get());

		qDebug() << "ICP converged.";
		qDebug() << "The score is " << icp.getFitnessScore();
	}
	else
		qDebug() << "ICP did not converge.";	
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::calculate_icp_for_all_keypoint_pairs()
{
	for (int i = 0; i < keypoint_point_cloud_vector.size(); i++)
	{
		qDebug() << QString("Calculating ICP %1/%2").arg(i + 1).arg(keypoint_point_cloud_vector.size());
		
		calculate_icp(
			keypoint_point_cloud_vector[i].second,
			keypoint_point_cloud_vector[i].first
		);
	}
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::calculate_recursive_icp()
{
	qDebug() << "Calculating recursive ICP...";
	for (int i = 0; i < point_cloud_vector.size(); i++)
	{
		Eigen::Matrix4f empty_matrix = Eigen::Matrix4f::Identity();
		icp_translation_matrix_vector.push_back(empty_matrix);
	}

	for (int i = icp_matrix_vector.size() - 1; i >= 0; i--)
	{
		qDebug() << QString("Calculating ICP %1/%2").arg(icp_matrix_vector.size() - i).arg(icp_matrix_vector.size());
		for (int j = point_cloud_vector.size() - 1; j > i; j--)
			icp_translation_matrix_vector[j] = icp_translation_matrix_vector[j] * icp_matrix_vector[i];
	}

	for (int i = 0; i < icp_translation_matrix_vector.size(); i++)
		final_translation_matrix_vector[i] *= icp_translation_matrix_vector[i];
	qDebug() << "Done!";

	//Apply to keypoint clouds
	for (int i = icp_matrix_vector.size() - 2; i >= 0; i--)
	{
		for (int j = keypoint_point_cloud_vector.size() - 1; j > i; j--)
		{
			pcl::transformPointCloud(
				*keypoint_point_cloud_vector[j].first.get(),
				*keypoint_point_cloud_vector[j].first.get(),
				icp_matrix_vector[i]
			);
			pcl::transformPointCloud(
				*keypoint_point_cloud_vector[j].second.get(),
				*keypoint_point_cloud_vector[j].second.get(),
				icp_matrix_vector[i]
			);
		}
	}
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::apply_icp_vector()
{
	for (int i = 0; i < point_cloud_vector.size(); i++)
	{
		pcl::transformPointCloud(
			*point_cloud_vector[i].get(),
			*point_cloud_vector[i].get(),
			icp_translation_matrix_vector[i]
		);
	}
}

//-----------------------MAIN-----------------------------

void PCL_OpenNI_Qt_Kinect_3d_Scanner::pcl_main()
{
	if (icpCheck->isChecked() && keypoint_point_cloud_vector.empty())
	{
		filter_all_point_clouds();
		reorganize_all_point_clouds();
		calculate_all_keypoint_pairs();
		remove_nan_from_all_clouds();

		if (settings->value("SAC_SETTINGS/CALCULATE_IN_FINAL").toBool())
			perform_sac_registration();
			
		if (settings->value("ICP_SETTINGS/CALCULATE_IN_FINAL").toBool())
		{
			calculate_icp_for_all_keypoint_pairs();
			calculate_recursive_icp();
		}		

		if (settings->value("VOXEL_GRID_REDUCTION_SETTINGS/ENABLE_IN_FINAL").toBool())
			apply_voxel_grid_reduction_to_all_point_clouds();

		if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
		{
			reorganize_all_point_clouds();
			visualize_mesh();
		}
		else
		{
			if (settings->value("ICP_SETTINGS/APPLY_IN_FINAL").toBool())
				apply_icp_vector();

			//Debug slider
			slider->setRange(1, point_cloud_vector.size() - 1);
			if (slider->isHidden())
				slider->show();			
			pair_viewer_index = -1;

			if (settings->value("FINAL_SETTINGS/MERGING_ALL_CLOUDS").toBool())
			{
				PcdPtr main_point_cloud_ptr(new Pcd);
				PcdPtr filtered_main_point_cloud_ptr(new Pcd);
				merging_all_point_clouds(point_cloud_vector, main_point_cloud_ptr);
				filtered_main_point_cloud_ptr = main_point_cloud_ptr;

				if (settings->value("VOXEL_GRID_REDUCTION_SETTINGS/ENABLE_IN_FINAL").toBool())
				{
					qDebug() << "Applying voxel grid reduction to final cloud...";
					apply_voxel_grid_reduction(main_point_cloud_ptr, filtered_main_point_cloud_ptr);
					qDebug() << "Done!";
				}

				if (settings->value("MOVING_LEAST_SQUARES_FILTER_SETTINGS/ENABLE_IN_FINAL").toBool())
				{
					qDebug() << "Applying smoothing to final cloud...";
					apply_moving_least_squares_filter(filtered_main_point_cloud_ptr);
					qDebug() << "Done!";
				}

				if (settings->value("SAVING_FINAL_POINT_CLOUD_SETTINGS/SAVE_PCD").toBool())
				{
					qDebug() << "Saving final cloud as PCD file...";
					PclIO::save_one_point_cloud(
						settings->value("SAVING_FINAL_POINT_CLOUD_SETTINGS/FINAL_PCD_FILENAME").toString(),
						filtered_main_point_cloud_ptr
						);
					qDebug() << "Done!";
				}

				if (settings->value("SAVING_FINAL_POINT_CLOUD_SETTINGS/SAVE_PLY").toBool())
				{
					qDebug() << "Saving final cloud as PLY file...";
					PclIO::save_one_point_cloud(
						settings->value("SAVING_FINAL_POINT_CLOUD_SETTINGS/FINAL_PLY_FILENAME").toString(),
						filtered_main_point_cloud_ptr
						);
					qDebug() << "Done!";
				}

				point_cloud_vector.clear();
				point_cloud_vector.push_back(filtered_main_point_cloud_ptr);
			}
		}
	}	

	if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool() == false)
	{
		viewer.get()->removeAllShapes();
		viewer.get()->removeAllPointClouds();
		add_viewer_debug_text();

		if (settings->value("FINAL_SETTINGS/DRAW_ALL_CLOUDS").toBool())
			visualize_point_clouds(point_cloud_vector);
		if (settings->value("FINAL_SETTINGS/DRAW_ALL_KEYPOINT_CLOUDS").toBool())
			visualize_all_keypoint_clouds();
	}				
}

//#######################################################
//#-----------------------SLOTS--------------------------

void PCL_OpenNI_Qt_Kinect_3d_Scanner::slot_initialize()
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

		recCheck		 ->setDisabled(false);
		streamFromCheck  ->setDisabled(false);
		recToPclDataCheck->setDisabled(false);

		initButton->setText("Start Stream");
	}
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::slot_take_images()
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

void PCL_OpenNI_Qt_Kinect_3d_Scanner::slot_take_op_images()
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

void PCL_OpenNI_Qt_Kinect_3d_Scanner::slot_take_one_op_image()
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

void PCL_OpenNI_Qt_Kinect_3d_Scanner::slot_draw_scene3d_model()
{
	pcl_main();
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::slot_save_data()
{
	save_data();
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::slot_read_data()
{
	statFilterCheck->setDisabled(true);
	approxFilterCheck->setDisabled(true);

	slider->hide();
	read_data();

	statFilterCheck->setDisabled(false);
	approxFilterCheck->setDisabled(false);
}

void PCL_OpenNI_Qt_Kinect_3d_Scanner::slot_change_pair(int index)
{
	viewer.get()->removeAllShapes();
	viewer.get()->removeAllPointClouds();
	add_viewer_debug_text();

	viewer.get()->addText(
		QString("CLOUD #%1 & #%2").arg(index - 1).arg(index).toStdString(),
		settings->value("VISUALIZATION/CLOUD_TEXT_X").toInt(),
		settings->value("VISUALIZATION/CLOUD_TEXT_Y").toInt(),
		settings->value("VISUALIZATION/CLOUD_TEXT_FONT_SIZE").toInt(),
		0,0,0,
		"clouds_text"
	);

	if (settings->value("FINAL_SETTINGS/DRAW_ALL_CLOUDS").toBool())
	{
		//Main clouds
		pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb1(point_cloud_vector[index]);
		viewer.get()->addPointCloud<PointType>(point_cloud_vector[index], rgb1, QString("Cloud #%1").arg(index).toStdString());
		pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb2(point_cloud_vector[index - 1]);
		viewer.get()->addPointCloud<PointType>(point_cloud_vector[index - 1], rgb2, QString("Cloud #%1").arg(index - 1).toStdString());
	}
	
	if (settings->value("FINAL_SETTINGS/DRAW_ALL_KEYPOINT_CLOUDS").toBool())
	{
		pair_viewer_index = index - 1;
		index = index - 1;
		//Correspondences
		viewer.get()->addCorrespondences<PointType>(
			keypoint_point_cloud_vector[index].first,
			keypoint_point_cloud_vector[index].second,
			correspondences_vector[index],
			QString("Cor %1").arg(index).toStdString()
		);

		//First keypoint point cloud 
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(
			keypoint_point_cloud_vector[index].first,
			255, 0, 0
		);
		viewer.get()->addPointCloud<PointType>(
			keypoint_point_cloud_vector[index].first, single_color,
			QString("Cloud P1 %1").arg(index).toStdString()
		);
		viewer.get()->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,
			QString("Cloud P1 %1").arg(index).toStdString()
		);

		//Second keypoint point cloud 
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color1(
			keypoint_point_cloud_vector[index].second,
			0, 0, 255
		);
		viewer.get()->addPointCloud<PointType>(
			keypoint_point_cloud_vector[index].second, single_color1,
			QString("Cloud P2 %1").arg(index).toStdString()
		);
		viewer.get()->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,
			QString("Cloud P2 %1").arg(index).toStdString()
		);
	}
}