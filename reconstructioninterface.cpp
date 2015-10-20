#include "reconstructioninterface.h"

//#######################################################

ReconstructionInterface::ReconstructionInterface(QObject *parent, QSettings* parent_settings) 
	: ScannerBase(parent, parent_settings)
{
	volumeReconstruction = new VolumeReconstruction(this, settings);
	pcdVizualizer = new PcdVizualizer(this, settings);

	use_reconstruction   = false;
	use_undistortion	 = false;
	use_bilateral_filter = false;
	use_statistical_outlier_removal_filter = false;
	use_moving_least_squares_filter		   = false;
}

//#######################################################

void ReconstructionInterface::set_use_reconstruction(bool value)
{
	use_reconstruction = value;
}

void ReconstructionInterface::set_use_undistortion(bool value)
{
	use_undistortion = value;
}

void ReconstructionInterface::set_use_bilateral_filter(bool value)
{
	use_bilateral_filter = value;
}

void ReconstructionInterface::set_use_statistical_outlier_removal_filter(bool value)
{
	use_statistical_outlier_removal_filter = value;
}

void ReconstructionInterface::set_use_moving_least_squares_filter(bool value)
{
	use_moving_least_squares_filter = value;
}


//#######################################################
//#---------------------SAVE READ------------------------

void ReconstructionInterface::read_data(
	int from, int to, int step, 
	Frames& frames
	)
{
	qDebug() << "Start reading data...";
	for (int i = from; i <= to; i += step)
	{
		PcdPtr point_cloud_ptr(new Pcd);

		QString pcd_filename_pattern =
			settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "\\" +
			settings->value("READING_PATTERNS/POINT_CLOUD_NAME").toString();
		qDebug() << "Reading" << pcd_filename_pattern.arg(i);
		PclIO::load_one_point_cloud(pcd_filename_pattern.arg(i), point_cloud_ptr);

		if (point_cloud_ptr.get()->points.empty())
			continue;

		QString filename_pattern =
			settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "\\" +
			settings->value("READING_PATTERNS/POINT_CLOUD_IMAGE_NAME").toString();
		qDebug() << "Reading" << filename_pattern.arg(i);
		cv::Mat image = cv::imread(filename_pattern.arg(i).toStdString());
		
		Frame frame;
		frame.pointCloudPtr   = point_cloud_ptr;
		frame.pointCloudImage = image;
		frame.pointCloudNormalPcdPtr = NormalPcdPtr(new NormalPcd);
		frames.push_back(frame);
	}

	qDebug() << "Done!";
	qDebug() << "Total:" << frames.size();
}


//#######################################################
//#------------------RECONSTRUCTION----------------------

//----------------------Filters--------------------------

void ReconstructionInterface::filter_point_cloud(Frame& frame)
{
	PcdPtr dest_point_cloud_ptr(new Pcd);

	//Bilateral
	if (use_bilateral_filter)
	{
		int d = settings->value("OPENCV_BILATERAL_FILTER_SETTINGS/D").toInt();
		double sigma_color = settings->value("OPENCV_BILATERAL_FILTER_SETTINGS/SIGMA_COLOR").toDouble();
		double sigma_space = settings->value("OPENCV_BILATERAL_FILTER_SETTINGS/SIGMA_SPACE").toDouble();
		PcdFilters::apply_bilateral_filter(frame.pointCloudPtr, d, sigma_color, sigma_space);
	}
		
	vector<int> indexes_vector;
	pcl::removeNaNFromPointCloud(*frame.pointCloudPtr.get(), *dest_point_cloud_ptr.get(), indexes_vector);
	frame.pointCloudIndexes = indexes_vector;

	//Statistic reduction
	if (use_statistical_outlier_removal_filter) {
		int   meanK		      = settings->value("STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/MEAN_K").toInt();
		float stddevMulThresh = settings->value("STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/MUL_THRESH").toFloat();
		PcdFilters::apply_statistical_outlier_removal_filter(dest_point_cloud_ptr, frame.pointCloudPtr, meanK, stddevMulThresh);
		
		if (settings->value("STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/ENABLE_LOG").toBool()) {
			qDebug() << "Statistical removal: from "
					 << dest_point_cloud_ptr->size()
					 << "to"
					 << frame.pointCloudPtr->size();
		}

		pcl::copyPointCloud(*frame.pointCloudPtr, *dest_point_cloud_ptr);
	}

	//Smooth
	if (use_moving_least_squares_filter) {
		double sqrGaussParam = settings->value("MOVING_LEAST_SQUARES_FILTER_SETTINGS/SQR_GAUSS_PARAM").toDouble();
		double searchRadius = settings->value("MOVING_LEAST_SQUARES_FILTER_SETTINGS/SEARCH_RADIUS").toDouble();
		pcl::PointCloud<pcl::PointNormal>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointNormal>);
		PcdFilters::apply_moving_least_squares_filter(dest_point_cloud_ptr, smoothed_cloud, sqrGaussParam, searchRadius);
		pcl::copyPointCloud(*smoothed_cloud, *dest_point_cloud_ptr);
	}
		
	pcl::copyPointCloud(*dest_point_cloud_ptr.get(), *frame.pointCloudPtr.get());
}

void ReconstructionInterface::filter_all_point_clouds(Frames& frames)
{
	if (use_undistortion)
	{
		CalibrationInterface calibrationInterface(this, settings);
		calibrationInterface.loadCalibrationData();
		calibrationInterface.calibrate();
		calibrationInterface.undistort(frames);
	}

	qDebug() << "Applying filters for point clouds...";
	for (int i = 0; i < frames.size(); i++)
	{
		qDebug() << QString("Applying filters %1 / %2").arg(i + 1).arg(frames.size()).toStdString().c_str();
		filter_point_cloud(frames[i]);
	}
	qDebug() << "Done!";
}

void ReconstructionInterface::remove_nan_from_all_clouds(Frames& frames)
{
	qDebug() << "Removing NaN's from point clouds...";
	for (int i = 0; i < frames.size(); i++)
	{
		frames[i].pointCloudPtr->is_dense = false;

		PcdPtr tmp_point_cloud_ptr(new Pcd);
		vector<int> indexes_vector;
		pcl::removeNaNFromPointCloud(*frames[i].pointCloudPtr, *tmp_point_cloud_ptr, indexes_vector);
		pcl::copyPointCloud(*tmp_point_cloud_ptr, *frames[i].pointCloudPtr);

		frames[i].pointCloudIndexes = indexes_vector;
	}
	qDebug() << "Done!";
}

void ReconstructionInterface::reorganize_all_point_clouds(Frames& frames)
{
	qDebug() << "Reorganization all point clouds";
	for (int i = 0; i < frames.size(); i++)
	{
		PcdPtr tmp_pcd_ptr(new Pcd);
		tmp_pcd_ptr->resize(HEIGHT*WIDTH);

		for (int y = 0; y < HEIGHT; y++) {
			for (int x = 0; x < WIDTH; x++) {
				tmp_pcd_ptr->at(x, y).z = NAN;
			}
		}

		for (int j = 0; j < frames[i].pointCloudPtr->size(); j++) {
			tmp_pcd_ptr->points[frames[i].pointCloudIndexes[j]] = frames[i].pointCloudPtr->points[j];
		}

		pcl::copyPointCloud(*tmp_pcd_ptr, *frames[i].pointCloudPtr);
		frames[i].pointCloudPtr->is_dense = false;
	}
	qDebug() << "Done!";
}

//----------------------KeyPoints------------------------

void ReconstructionInterface::calculate_keypoint_clouds(
	Frame& frame1, Frame& frame2,
	KeypointsFrame& keypoints_frame,
	std::vector<cv::Mat>& matches_images_vector
	)
{
	bool aruco = settings->value("KEYPOINT_SEARCH_SETTINGS/USE_ARUCO").toBool();
	bool surf  = settings->value("KEYPOINT_SEARCH_SETTINGS/USE_SURF").toBool();

	PcdPtr keypoint_cloud_ptr1(new Pcd);
	PcdPtr keypoint_cloud_ptr2(new Pcd);

	//Find keypoints
	if (aruco == true)
	{
		ArUcoKeypointDetector aruco(
			this, 
			frame1.pointCloudPtr,	frame2.pointCloudPtr, 
			frame1.pointCloudImage, frame2.pointCloudImage, 
			keypoint_cloud_ptr1,    keypoint_cloud_ptr2
		);
		aruco.detect();
	}

	if (surf == true)
	{
		SurfKeypointDetector surf(
			this, 
			frame1.pointCloudPtr,   frame2.pointCloudPtr,
			frame1.pointCloudImage, frame2.pointCloudImage,
			keypoint_cloud_ptr1,    keypoint_cloud_ptr2
		);
		surf.detect();

		if (settings->value("OPENCV_KEYPOINT_DETECTION_SETTINGS/DRAW_GOOD_FILTERED_MATCHES").toBool())
		{
			 surf.getMatchImagesVector(&matches_images_vector);
		}
	}

	//Fill keypoints_frame
	pcl::Correspondences correspondences;
	for (int i = 0; i < keypoint_cloud_ptr1.get()->points.size() - 1; i++)
	{
		pcl::Correspondence correspondence;
		correspondence.index_query = i;
		correspondence.index_match = i;
		PointType a = keypoint_cloud_ptr1.get()->points[i];
		PointType b = keypoint_cloud_ptr2.get()->points[i];
		correspondence.distance = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2) + powf(a.z - b.z, 2));
		correspondences.push_back(correspondence);
	}

	keypoints_frame.keypointsPcdCorrespondences = correspondences;
	keypoints_frame.keypointsPcdPair = std::make_pair(keypoint_cloud_ptr1, keypoint_cloud_ptr2);

	if (correspondences.empty())
		qDebug() << "ERROR: 0 Keypoints found!";
}

void ReconstructionInterface::calculate_all_keypoint_pairs(
	Frames& frames,
	KeypointsFrames& keypointsFrames
	)
{
	std::vector<cv::Mat> matches_images_vector;

	qDebug() << "Calculating keypoint pairs...";
	for (int i = 1; i < frames.size(); i++)
	{
		qDebug() << QString("Calculating pairs %1 / %2").arg(i).arg(frames.size() - 1).toStdString().c_str();

		KeypointsFrame keypointsFrame;
		keypointsFrame.keypointsNormalPcdPair = std::make_pair(NormalPcdPtr(new NormalPcd), NormalPcdPtr(new NormalPcd));
		calculate_keypoint_clouds(frames[i - 1], frames[i], keypointsFrame, matches_images_vector);
		keypointsFrames.push_back(keypointsFrame);
	}
	qDebug() << "Done!";

	if (settings->value("OPENCV_KEYPOINT_DETECTION_SETTINGS/DRAW_GOOD_FILTERED_MATCHES").toBool() &&
		!matches_images_vector.empty())
	{
		ImagesViewerWidget* viewer1 = new ImagesViewerWidget(matches_images_vector, "After NaN & Thresh");
	}
}

void ReconstructionInterface::calculate_middle_based_all_keypoint_pairs(
	Frames& frames,
	KeypointsFrames& keypointsFrames,
	int middle_index
	)
{
	std::vector<cv::Mat> matches_images_vector;

	qDebug() << "Calculating keypoint pairs...";
	for (int i = 0; i < frames.size(); i++)
	{
		qDebug() << QString("Calculating pairs %1 / %2").arg(i + 1).arg(frames.size()).toStdString().c_str();

		if (i != middle_index)
		{
			KeypointsFrame keypointsFrame;
			keypointsFrame.keypointsNormalPcdPair = std::make_pair(NormalPcdPtr(new NormalPcd), NormalPcdPtr(new NormalPcd));
			calculate_keypoint_clouds(frames[middle_index], frames[i], keypointsFrame, matches_images_vector);
			keypointsFrames.push_back(keypointsFrame);
		}
	}
	qDebug() << "Done!";

	if (settings->value("OPENCV_KEYPOINT_DETECTION_SETTINGS/DRAW_GOOD_FILTERED_MATCHES").toBool() &&
		!matches_images_vector.empty())
	{
		ImagesViewerWidget* viewer1 = new ImagesViewerWidget(matches_images_vector, "After NaN & Thresh");
	}		
}

//---------------------Registration----------------------

void ReconstructionInterface::perform_sac_registration(
	Frames&	frames,
	KeypointsFrames& keypointsFrames,
	Eigen::Matrix4f& intial_transformation_matrix,
	Matrix4fVector& sac_translation_matrix_vector,
	int middle_index
	)
{
	SaCRegistration sac(this);
	sac.calculateSaCTransformation(keypointsFrames, intial_transformation_matrix, middle_index);
	sac.getTransformation(sac_translation_matrix_vector);

	if (!settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool() &&
		!settings->value("FINAL_SETTINGS/MIDDLE_BASED_RECONSTRUCTION").toBool() &&
		!settings->value("FINAL_SETTINGS/LUM_BASED_RECONSTRUCTION_ENABLE").toBool() &&
		 settings->value("SAC_SETTINGS/APPLY_IN_FINAL").toBool() &&
		 middle_index == -1)
	{
		qDebug() << "Applying SaC...";
		sac.applyTransfomation(frames);
		qDebug() << "Done!";
	}
}

void ReconstructionInterface::perform_icp_registration(
	Frames&	frames,
	KeypointsFrames& keypointsFrames,
	Eigen::Matrix4f& intial_transformation_matrix,
	Matrix4fVector&  icp_translation_matrix_vector,
	int middle_index
	)
{
	ICPRegistration icp(this);
	icp.calculateICPTransformation(keypointsFrames, intial_transformation_matrix, middle_index);
	icp.getTransformation(icp_translation_matrix_vector);

	if (!settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool() &&
		!settings->value("FINAL_SETTINGS/MIDDLE_BASED_RECONSTRUCTION").toBool() &&
		!settings->value("FINAL_SETTINGS/LUM_BASED_RECONSTRUCTION_ENABLE").toBool() &&
		 settings->value("ICP_SETTINGS/APPLY_IN_FINAL").toBool() &&
		 middle_index == -1)
	{
		qDebug() << "Applying ICP...";
		icp.applyTransfomation(frames);
		qDebug() << "Done!";
	}
}

void ReconstructionInterface::calculate_final_transformation_matrix_vector(
	Matrix4fVector& sac_translation_matrix_vector,
	Matrix4fVector& icp_translation_matrix_vector,
	Matrix4fVector& final_translation_matrix_vector,
	int final_translation_matrix_vector_size
	)
{
	for (int i = 0; i < final_translation_matrix_vector_size; i++)
	{
		Eigen::Matrix4f fin;
		if (!sac_translation_matrix_vector.empty() &&
			!icp_translation_matrix_vector.empty())
		{
			Eigen::Matrix4f sac = sac_translation_matrix_vector[i];
			Eigen::Matrix4f icp = icp_translation_matrix_vector[i];
			fin = icp * sac;
		}
		else if (!sac_translation_matrix_vector.empty())
		{
			Eigen::Matrix4f sac = sac_translation_matrix_vector[i];
			fin = sac;
		}
		else if(!icp_translation_matrix_vector.empty())
		{
			Eigen::Matrix4f icp = icp_translation_matrix_vector[i];
			fin = icp;
		}
		else
		{
			fin = Eigen::Matrix4f::Identity();
		}

		final_translation_matrix_vector.push_back(fin);
	}
}

void ReconstructionInterface::perform_tsdf_integration(
	Frames& frames,
	Matrix4fVector& final_translation_matrix_vector
	)
{
	PcdPtrVector point_cloud_vector;
	for (int i = 0; i < frames.size(); i++)
		point_cloud_vector.push_back(frames[i].pointCloudPtr);

	volumeReconstruction->addPointCloudVector(
		point_cloud_vector,
		final_translation_matrix_vector
	);
}

void ReconstructionInterface::perform_tsdf_meshing(
	Matrix4fVector& final_translation_matrix_vector
	)
{
	volumeReconstruction->prepareVolume();
	pcdVizualizer->redraw();

	if (settings->value("FINAL_SETTINGS/DRAW_ALL_CAMERA_POSES").toBool())
		pcdVizualizer->visualizeCameraPoses(final_translation_matrix_vector);

	if (settings->value("CPU_TSDF_SETTINGS/DRAW_MESH").toBool())
	{
		pcl::PolygonMesh mesh;
		volumeReconstruction->calculateMesh();
		volumeReconstruction->getPoligonMesh(mesh);
		pcdVizualizer->visualizeMesh(mesh);
	}
}

//-------------------------MAIN--------------------------

void ReconstructionInterface::perform_reconstruction()
{
	int from = settings->value("READING_SETTING/FROM").toInt();
	int to   = settings->value("READING_SETTING/TO").toInt();
	int step = settings->value("READING_SETTING/STEP").toInt();

	Frames frames;
	read_data(from, to, step, frames);

	KeypointsFrames keypointsFrames;	
	Matrix4fVector  final_translation_matrix_vector;

	if (use_reconstruction)
	{
		filter_all_point_clouds(frames);
		reorganize_all_point_clouds(frames);
		calculate_all_keypoint_pairs(frames, keypointsFrames);
		remove_nan_from_all_clouds(frames);
		
		Matrix4fVector sac_translation_matrix_vector;
		Matrix4fVector icp_translation_matrix_vector;
		
		Eigen::Matrix4f intial_transformation_matrix = Eigen::Matrix4f::Identity();
		if (settings->value("SAC_SETTINGS/CALCULATE_IN_FINAL").toBool())
			perform_sac_registration(frames, keypointsFrames, intial_transformation_matrix, sac_translation_matrix_vector);

		if (settings->value("ICP_SETTINGS/CALCULATE_IN_FINAL").toBool())
			perform_icp_registration(frames, keypointsFrames, intial_transformation_matrix, icp_translation_matrix_vector);

		calculate_final_transformation_matrix_vector(
			sac_translation_matrix_vector,
			icp_translation_matrix_vector,
			final_translation_matrix_vector,
			frames.size()
		);

		if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
		{
			reorganize_all_point_clouds(frames);
			perform_tsdf_integration(frames, final_translation_matrix_vector);
			perform_tsdf_meshing(final_translation_matrix_vector);
		}		
	}

	if (!settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
	{
		pcdVizualizer->redraw();
		
		if (settings->value("FINAL_SETTINGS/DRAW_ALL_CAMERA_POSES").toBool())
			pcdVizualizer->visualizeCameraPoses(final_translation_matrix_vector);
		if (settings->value("FINAL_SETTINGS/DRAW_ALL_CLOUDS").toBool())
			pcdVizualizer->visualizePointClouds(frames);
		if (settings->value("FINAL_SETTINGS/DRAW_ALL_KEYPOINT_CLOUDS").toBool())
			pcdVizualizer->visualizeKeypointClouds(keypointsFrames);
	}
}

void ReconstructionInterface::perform_iterative_reconstruction()
{
	int from = settings->value("READING_SETTING/FROM").toInt();
	int to   = settings->value("READING_SETTING/TO").toInt();
	int step = settings->value("READING_SETTING/STEP").toInt();

	int length = (to - from) / step;
	int main_step = settings->value("FINAL_SETTINGS/ITERATIVE_RECONSTRUCTION_STEP").toInt();

	if (length < main_step)
	{
		perform_reconstruction();
		return;
	}
		
	Eigen::Matrix4f intial_transformation_matrix = Eigen::Matrix4f::Identity();
	Matrix4fVector camera_pose_matrix_vector;
	for (int i = 0; i < length / main_step; i++)
	{
		qDebug() << "###########################################";
		qDebug() << "START ITERATION:" << i + 1 << "/" << length / main_step
				 << "FROM:" << from + (i * main_step * step)
				 << "TO:"   << from + (i * main_step * step) + (main_step * step)
				 << "STEP:" << step;
		qDebug() << "###########################################";

		Frames frames;
		read_data(
			from + (i * main_step * step), 
			from + (i * main_step * step) + (main_step * step), 
			step, 
			frames
		);

		KeypointsFrames keypointsFrames;
		Matrix4fVector  final_translation_matrix_vector;

		if (use_reconstruction)
		{
			filter_all_point_clouds(frames);
			reorganize_all_point_clouds(frames);
			calculate_all_keypoint_pairs(frames, keypointsFrames);
			remove_nan_from_all_clouds(frames);

			Matrix4fVector sac_translation_matrix_vector;
			Matrix4fVector icp_translation_matrix_vector;
			
			if (settings->value("SAC_SETTINGS/CALCULATE_IN_FINAL").toBool())
				perform_sac_registration(frames, keypointsFrames, intial_transformation_matrix, sac_translation_matrix_vector);

			Eigen::Matrix4f empty_mat = Eigen::Matrix4f::Identity();
			if (settings->value("ICP_SETTINGS/CALCULATE_IN_FINAL").toBool())
				perform_icp_registration(frames, keypointsFrames, empty_mat, icp_translation_matrix_vector);

			calculate_final_transformation_matrix_vector(
				sac_translation_matrix_vector,
				icp_translation_matrix_vector,
				final_translation_matrix_vector,
				frames.size()
			);

			if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
			{
				reorganize_all_point_clouds(frames);
				perform_tsdf_integration(frames, final_translation_matrix_vector);
			}

			intial_transformation_matrix = final_translation_matrix_vector[final_translation_matrix_vector.size() - 1];

			for (int j = 0; j < final_translation_matrix_vector.size(); j++)
				camera_pose_matrix_vector.push_back(final_translation_matrix_vector[j]);

		}

		qDebug() << "###########################################";
		qDebug() << "DONE ITERATION:" << i + 1 << "/" << length / main_step << "PROCESSED TOTAL:" << camera_pose_matrix_vector.size();
		qDebug() << "###########################################\n";
	}

	perform_tsdf_meshing(camera_pose_matrix_vector);
}

void ReconstructionInterface::perform_partition_recursive_reconstruction()
{
	Matrix4fVector  base_translation_matrix_vector;

	int from = settings->value("READING_SETTING/FROM").toInt();
	int to   = settings->value("READING_SETTING/TO").toInt();
	int step = settings->value("READING_SETTING/STEP").toInt();

	int main_step = settings->value("FINAL_SETTINGS/MIDDLE_BASED_RECONSTRUCTION_STEP").toInt();
	int middle_index = (int)round((float)main_step / 2.0f);

	qDebug() << "Central clouds indexes:";
	for (int i = from + middle_index; i <= to - middle_index; i += main_step)
		qDebug() << "  Index:" << i;

	{
		Frames frames;
		KeypointsFrames keypointsFrames;

		read_data(from + middle_index, to - middle_index, main_step, frames);
		filter_all_point_clouds(frames);
		reorganize_all_point_clouds(frames);
		calculate_all_keypoint_pairs(frames, keypointsFrames);
		remove_nan_from_all_clouds(frames);

		Matrix4fVector sac_translation_matrix_vector;
		Matrix4fVector icp_translation_matrix_vector;

		Eigen::Matrix4f intial_transformation_matrix = Eigen::Matrix4f::Identity();
		perform_sac_registration(frames, keypointsFrames, intial_transformation_matrix, sac_translation_matrix_vector);
		perform_icp_registration(frames, keypointsFrames, intial_transformation_matrix, icp_translation_matrix_vector);

		calculate_final_transformation_matrix_vector(
			sac_translation_matrix_vector,
			icp_translation_matrix_vector,
			base_translation_matrix_vector,
			frames.size()
		);
		
		for (int i = 0; i < frames.size(); i++)
			pcl::transformPointCloud(
				*frames[i].pointCloudPtr.get(),
				*frames[i].pointCloudPtr.get(),
				base_translation_matrix_vector[i]
			);

		if (settings->value("FINAL_SETTINGS/DRAW_ALL_CAMERA_POSES").toBool())
			pcdVizualizer->visualizeCameraPoses(base_translation_matrix_vector);
		if (settings->value("FINAL_SETTINGS/DRAW_ALL_CLOUDS").toBool())
			pcdVizualizer->visualizePointClouds(frames);
		if (settings->value("FINAL_SETTINGS/DRAW_ALL_KEYPOINT_CLOUDS").toBool())
			pcdVizualizer->visualizeKeypointClouds(keypointsFrames);
	}

	Matrix4fVector camera_pose_matrix_vector;
	for (int index = 0; index < base_translation_matrix_vector.size(); index++)
	{
		int _from = from + (index * main_step);
		int _to   = from + (index * main_step) + main_step;
		int _step = step;

		Frames			frames;
		KeypointsFrames keypointsFrames;
		Matrix4fVector  final_translation_matrix_vector;
		
		read_data(_from, _to, _step, frames);

		qDebug() << "###########################################";
		qDebug() << "START ITERATION:" << index + 1 << "/" << base_translation_matrix_vector.size()
				 << "FROM:" << _from << "TO:" << _to << "STEP:" << _step
				 << "LENGTH:" << frames.size() << "MIDDLE INDEX:" << _from + middle_index;
		qDebug() << "###########################################";

		filter_all_point_clouds(frames);
		reorganize_all_point_clouds(frames);
		calculate_middle_based_all_keypoint_pairs(frames, keypointsFrames, middle_index);
		remove_nan_from_all_clouds(frames);

		Matrix4fVector sac_translation_matrix_vector;
		perform_sac_registration(frames, keypointsFrames, base_translation_matrix_vector[index], sac_translation_matrix_vector, middle_index);

		Matrix4fVector icp_translation_matrix_vector;
		Eigen::Matrix4f intial_transformation_matrix = Eigen::Matrix4f::Identity();
		perform_icp_registration(frames, keypointsFrames, intial_transformation_matrix, icp_translation_matrix_vector, middle_index);

		int counter = 0;
		for (int i = 0; i < frames.size(); i++)
		{
			Eigen::Matrix4f fin;
			if (i != middle_index)
			{
				fin = icp_translation_matrix_vector[counter] * sac_translation_matrix_vector[counter];
				counter++;
			}
			else
			{
				fin = base_translation_matrix_vector[index];
			}
			final_translation_matrix_vector.push_back(fin);
		}

		if (!settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
		{
			for (int i = 0; i < frames.size(); i++)
			{
				pcl::transformPointCloud(
					*frames[i].pointCloudPtr.get(),
					*frames[i].pointCloudPtr.get(),
					final_translation_matrix_vector[i]
				);
			}
		}

		if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
		{
			for (int i = 0; i < final_translation_matrix_vector.size(); i++)
				camera_pose_matrix_vector.push_back(final_translation_matrix_vector[i]);
		}

		if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
		{
			reorganize_all_point_clouds(frames);
			perform_tsdf_integration(frames, final_translation_matrix_vector);
		}
		else
		{
			if (settings->value("FINAL_SETTINGS/DRAW_ALL_CAMERA_POSES").toBool())
				pcdVizualizer->visualizeCameraPoses(final_translation_matrix_vector);
			if (settings->value("FINAL_SETTINGS/DRAW_ALL_CLOUDS").toBool())
				pcdVizualizer->visualizePointClouds(frames);
			if (settings->value("FINAL_SETTINGS/DRAW_ALL_KEYPOINT_CLOUDS").toBool())
				pcdVizualizer->visualizeKeypointClouds(keypointsFrames);
		}

		qDebug() << "###########################################";
		qDebug() << "DONE ITERATION:"  << index + 1 << "/" << base_translation_matrix_vector.size()
				 << "PROCESSED TOTAL:" << camera_pose_matrix_vector.size();
		qDebug() << "###########################################\n";
	}

	if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
		perform_tsdf_meshing(camera_pose_matrix_vector);
}

void ReconstructionInterface::perform_lum_reconstruction()
{
	//#########################################################################################
	//# Step 1:
	//#########################################################################################
	//#	1) Read initial amount of clouds with some step.	
	//#	2) Do registration while distance between the last cloud camera position	
	//#	   and the first cloud camera position less than threshold.	
	//#	3) When distance threshold passed check the first cloud 
	//#	   and the last clouds as start and end of a loop.	
	//#	4) Fill loop pair vector with cloud indexes, cloud ptr, 
	//#	   cloud translation matrix. 		
	//#	5) Repeat from the beginning while needed.
	//#########################################################################################

	int from = settings->value("READING_SETTING/FROM").toInt();
	int to   = settings->value("READING_SETTING/TO").toInt();
	int loop_step = settings->value("FINAL_SETTINGS/LUM_BASED_RECONSTRUCTION_STEP").toInt();

	float camera_distance_threshold = 
		settings->value("FINAL_SETTINGS/LUM_BASED_RECONSTRUCTION_THRESHOLD").toFloat();
	
	std::vector<std::pair<int, int>> loop_ends_indexes_vector;
	std::vector<KeypointsFrames>	 loop_ends_keypoint_frames_vector;
	std::vector<Matrix4fVector>		 loop_ends_transformation_vector;

	if (settings->value("FINAL_SETTINGS/LUM_BASED_RECONSTRUCTION_DINAMIC_STEP").toBool())
	{
		while (true)
		{
			if (loop_ends_indexes_vector.empty() == false)
			{
				from = loop_ends_indexes_vector[loop_ends_indexes_vector.size() - 1].second;

				if (loop_ends_indexes_vector[loop_ends_indexes_vector.size() - 1].first ==
					loop_ends_indexes_vector[loop_ends_indexes_vector.size() - 1].second)
				{
					loop_ends_indexes_vector.pop_back();
					camera_distance_threshold += 0.002;
				}
				else
				{
					camera_distance_threshold =
						settings->value("FINAL_SETTINGS/LUM_BASED_RECONSTRUCTION_THRESHOLD").toFloat();
				}

				if (loop_ends_indexes_vector.empty() == false &&
					loop_ends_indexes_vector[loop_ends_indexes_vector.size() - 1].second == to)
				{
					break;
				}

			}

			std::vector<KeypointsFrames> inliersKeypointsFramesVector;
			for (int cloud_index = from + loop_step; cloud_index <= to; cloud_index += loop_step)
			{
				//Registration
				Frames frames;
				KeypointsFrames keypointsFrames;
				Matrix4fVector sac_translation_matrix_vector;
				Matrix4fVector icp_translation_matrix_vector;
				Matrix4fVector final_translation_matrix_vector;
				Eigen::Matrix4f empty_mat = Eigen::Matrix4f::Identity();

				read_data(from, cloud_index, cloud_index - from, frames);
				filter_all_point_clouds(frames);
				reorganize_all_point_clouds(frames);
				calculate_all_keypoint_pairs(frames, keypointsFrames);
				remove_nan_from_all_clouds(frames);

				//Saving untransformed filtered clouds
				KeypointsFrames inliersKeypointsFrames;
				KeypointsRejection rejection(this);
				rejection.rejection(keypointsFrames, inliersKeypointsFrames);
				inliersKeypointsFramesVector.push_back(inliersKeypointsFrames);

				if (settings->value("SAC_SETTINGS/CALCULATE_IN_FINAL").toBool())
					perform_sac_registration(frames, keypointsFrames, empty_mat, sac_translation_matrix_vector);
				if (settings->value("ICP_SETTINGS/CALCULATE_IN_FINAL").toBool())
					perform_icp_registration(frames, keypointsFrames, empty_mat, icp_translation_matrix_vector);

				calculate_final_transformation_matrix_vector(
					sac_translation_matrix_vector,
					icp_translation_matrix_vector,
					final_translation_matrix_vector,
					frames.size()
				);

				//Distance check
				PointType a;
				a.x = final_translation_matrix_vector[0](0, 3);
				a.y = final_translation_matrix_vector[0](1, 3);
				a.z = final_translation_matrix_vector[0](2, 3);
				PointType b;
				b.x = final_translation_matrix_vector[1](0, 3);
				b.y = final_translation_matrix_vector[1](1, 3);
				b.z = final_translation_matrix_vector[1](2, 3);

				float distance = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2) + powf(a.z - b.z, 2));
				if (distance > camera_distance_threshold)
				{
					loop_ends_indexes_vector.push_back(std::make_pair(from, cloud_index - loop_step));
					if (inliersKeypointsFramesVector.size() - 1 > 0)
						loop_ends_keypoint_frames_vector.push_back(
						inliersKeypointsFramesVector[inliersKeypointsFramesVector.size() - 2]
					);
					break;
				}
				else if (cloud_index == to)
				{
					loop_ends_indexes_vector.push_back(std::make_pair(from, cloud_index));
					if (inliersKeypointsFramesVector.size() - 1 > 0)
						loop_ends_keypoint_frames_vector.push_back(
						inliersKeypointsFramesVector[inliersKeypointsFramesVector.size() - 2]
					);
				}
			}
		}
	}
	else
	{
		int din_step = settings->value("FINAL_SETTINGS/LUM_BASED_RECONSTRUCTION_FIXED_STEP").toInt();
		for (int i = from; i < to; i += din_step)
		{		
			Frames frames;
			KeypointsFrames keypointsFrames;
			Matrix4fVector sac_translation_matrix_vector;
			Matrix4fVector icp_translation_matrix_vector;
			Matrix4fVector final_translation_matrix_vector;
			Eigen::Matrix4f empty_mat = Eigen::Matrix4f::Identity();

			read_data(i, i + din_step, din_step, frames);
			filter_all_point_clouds(frames);
			reorganize_all_point_clouds(frames);
			calculate_all_keypoint_pairs(frames, keypointsFrames);
			remove_nan_from_all_clouds(frames);

			//Saving untransformed filtered clouds
			KeypointsFrames inliersKeypointsFrames;
			KeypointsRejection rejection(this);
			rejection.rejection(keypointsFrames, inliersKeypointsFrames);
			
			if (settings->value("SAC_SETTINGS/CALCULATE_IN_FINAL").toBool())
				perform_sac_registration(frames, keypointsFrames, empty_mat, sac_translation_matrix_vector);
			if (settings->value("ICP_SETTINGS/CALCULATE_IN_FINAL").toBool())
				perform_icp_registration(frames, keypointsFrames, empty_mat, icp_translation_matrix_vector);

			calculate_final_transformation_matrix_vector(
				sac_translation_matrix_vector,
				icp_translation_matrix_vector,
				final_translation_matrix_vector,
				frames.size()
			);

			loop_ends_keypoint_frames_vector.push_back(inliersKeypointsFrames);
			loop_ends_indexes_vector.push_back(std::make_pair(i, i + din_step));
			loop_ends_transformation_vector.push_back(final_translation_matrix_vector);
		}
	}


	//#########################################################################################
	//# Step 2:
	//#########################################################################################
	//# 1) For every loop
	//#	2) For every inner point cloud calculate translation matrix
	//#	3) Transform every inner point cloud
	//#	4) Calculate loop end ans start connection
	//#	5) Insert every transformed inner point cloud into LUM and calculate LUM
	//#	6) Get every inner point cloud LUM transformation matrix and add it to initial trans. matrix
	//#	7) Go to step 1 if needed
	//#########################################################################################

	Eigen::Matrix4f intial_transformation_matrix = Eigen::Matrix4f::Identity();
	Matrix4fVector camera_pose_matrix_vector;
	int step = settings->value("READING_SETTING/STEP").toInt();

	for (int index = 0; index < loop_ends_indexes_vector.size(); index++)
	{
		Frames frames;	
		KeypointsFrames keypointsFrames;
		Matrix4fVector  final_translation_matrix_vector;
		Matrix4fVector  elch_final_translation_matrix_vector;
		Matrix4fVector  lum_final_translation_matrix_vector;
		
		read_data(loop_ends_indexes_vector[index].first, loop_ends_indexes_vector[index].second, step, frames);
		filter_all_point_clouds(frames);
		reorganize_all_point_clouds(frames);
		calculate_all_keypoint_pairs(frames, keypointsFrames);
		remove_nan_from_all_clouds(frames);
	
		Matrix4fVector sac_translation_matrix_vector;
		Matrix4fVector icp_translation_matrix_vector;		
		if (settings->value("SAC_SETTINGS/CALCULATE_IN_FINAL").toBool())
			perform_sac_registration(frames, keypointsFrames, intial_transformation_matrix, sac_translation_matrix_vector);	
		Eigen::Matrix4f empty_mat = Eigen::Matrix4f::Identity();
		if (settings->value("ICP_SETTINGS/CALCULATE_IN_FINAL").toBool())
			perform_icp_registration(frames, keypointsFrames, empty_mat, icp_translation_matrix_vector);

		calculate_final_transformation_matrix_vector(
			sac_translation_matrix_vector,
			icp_translation_matrix_vector,
			final_translation_matrix_vector,
			frames.size()
		);
		
		//Mergin pairs of keypoint clouds
		std::vector<pcl::Correspondences> correspondences_vector;
		PcdPtrVector   mergedKeypointVector;
		for (int j = 0; j <= keypointsFrames.size(); j++)
		{
			PcdPtr new_keypoint_cloud(new Pcd);

			if (j == 0)
			{						
				pcl::transformPointCloud(
					*loop_ends_keypoint_frames_vector[index][0].keypointsPcdPair.first.get(),
					*loop_ends_keypoint_frames_vector[index][0].keypointsPcdPair.first.get(),
					final_translation_matrix_vector[0]
				);
				for (int i = 0; i < loop_ends_keypoint_frames_vector[index][0].keypointsPcdPair.first.get()->points.size(); i++)
					new_keypoint_cloud.get()->points.push_back(loop_ends_keypoint_frames_vector[index][0].keypointsPcdPair.first.get()->points[i]);

				pcl::Correspondences correspondences;
				for (int i = 0; i < keypointsFrames[0].keypointsPcdPair.first.get()->points.size(); i++)
				{
					new_keypoint_cloud.get()->points.push_back(keypointsFrames[0].keypointsPcdPair.first.get()->points[i]);

					pcl::Correspondence correspondence;
					correspondence = keypointsFrames[0].keypointsPcdCorrespondences[i];
					correspondence.index_query = new_keypoint_cloud.get()->points.size() - 1;
					correspondences.push_back(correspondence);
				}
				correspondences_vector.push_back(correspondences);
			}		
			else if (j == keypointsFrames.size())
			{
				for (int i = 0; i < keypointsFrames[j - 1].keypointsPcdPair.second.get()->points.size(); i++)
					new_keypoint_cloud.get()->points.push_back(keypointsFrames[j - 1].keypointsPcdPair.second.get()->points[i]);
				
				pcl::transformPointCloud(
					*loop_ends_keypoint_frames_vector[index][0].keypointsPcdPair.second.get(),
					*loop_ends_keypoint_frames_vector[index][0].keypointsPcdPair.second.get(),
					final_translation_matrix_vector[final_translation_matrix_vector.size() - 1]
				);
				pcl::Correspondences correspondences;
				for (int i = 0; i < loop_ends_keypoint_frames_vector[index][0].keypointsPcdPair.second.get()->points.size(); i++)
				{
					new_keypoint_cloud.get()->points.push_back(loop_ends_keypoint_frames_vector[index][0].keypointsPcdPair.second.get()->points[i]);

					PointType a, b;
					a = new_keypoint_cloud.get()->points[new_keypoint_cloud.get()->points.size() - 1];
					b = mergedKeypointVector[0].get()->points[loop_ends_keypoint_frames_vector[index][0].keypointsPcdCorrespondences[i].index_query];

					pcl::Correspondence correspondence;
					correspondence.index_query = new_keypoint_cloud.get()->points.size() - 1;
					correspondence.index_match = loop_ends_keypoint_frames_vector[index][0].keypointsPcdCorrespondences[i].index_query; 
					correspondence.distance = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2) + powf(a.z - b.z, 2));
					correspondences.push_back(correspondence);
				}
				correspondences_vector.push_back(correspondences);
			}
			else
			{				
				for (int i = 0; i < keypointsFrames[j - 1].keypointsPcdPair.second.get()->points.size(); i++)
					new_keypoint_cloud.get()->points.push_back(keypointsFrames[j - 1].keypointsPcdPair.second.get()->points[i]);
				
				pcl::Correspondences correspondences;
				for (int i = 0; i < keypointsFrames[j].keypointsPcdPair.first.get()->points.size(); i++)
				{
					new_keypoint_cloud.get()->points.push_back(keypointsFrames[j].keypointsPcdPair.first.get()->points[i]);

					pcl::Correspondence correspondence;
					correspondence = keypointsFrames[j].keypointsPcdCorrespondences[i];
					correspondence.index_query = new_keypoint_cloud.get()->points.size() - 1;
					correspondences.push_back(correspondence);
				}
				correspondences_vector.push_back(correspondences);
			}
				
			new_keypoint_cloud.get()->width  = new_keypoint_cloud.get()->points.size();
			new_keypoint_cloud.get()->height = 1;
			mergedKeypointVector.push_back(new_keypoint_cloud);
		}

		//ELCH correction
		if (settings->value("FINAL_SETTINGS/LUM_BASED_RECONSTRUCTION_PRE_ELCH").toBool())
		{	
			qDebug() << "Calculating ELCH";

			pcl::registration::ELCH<pcl::PointXYZRGB> elch;
			for (int i = 0; i < mergedKeypointVector.size(); i++)
				elch.addPointCloud(mergedKeypointVector[i]);
									
			pcl::registration::ELCH<pcl::PointXYZRGB>::RegistrationPtr reg = elch.getReg();		
			double inlier_threshold = settings->value("SAC_SETTINGS/INLIER_THRESHOLD").toDouble();
			int max_iter = settings->value("SAC_SETTINGS/MAX_ITERATIONS").toInt();
			double trans_epsilon = settings->value("ICP_SETTINGS/TRANSFORMATION_EPSILON").toDouble();
			double euclid_epsilon = settings->value("ICP_SETTINGS/EUCLIDEAN_EPSILON").toDouble();
			reg.get()->setMaxCorrespondenceDistance(inlier_threshold);
			reg.get()->setMaximumIterations(max_iter);
			reg.get()->setEuclideanFitnessEpsilon(euclid_epsilon);
			reg.get()->setTransformationEpsilon(trans_epsilon);
			
			elch.setLoopStart(0);
			elch.setLoopEnd(mergedKeypointVector.size() - 1);

			/*
			std::vector<pcl::registration::CorrespondenceRejector::Ptr> rej_vec;
			rej_vec = reg.get()->getCorrespondenceRejectors();
			for (int i = 0; i < rej_vec.size(); i++)
			{
				qDebug() << "Set Cor:" << i + 1 << "/" << rej_vec.size();
				boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
				for (int k = 0; k < correspondences_vector[i].size(); k++)
					correspondences.get()->push_back(correspondences_vector[i][k]);
				rej_vec[i].get()->setInputCorrespondences(correspondences);
			}
			*/

			elch.compute();

			pcl::registration::ELCH<pcl::PointXYZRGB>::LoopGraphPtr loopGraphPtr;
			loopGraphPtr = elch.getLoopGraph();

			for (int i = 0; i < mergedKeypointVector.size(); i++)
			{
				Eigen::Matrix4f elch_trans = Eigen::Matrix4f::Identity();
				elch_trans = (*loopGraphPtr)[i].transform.matrix();
				final_translation_matrix_vector[i] = elch_trans * final_translation_matrix_vector[i];

				elch_final_translation_matrix_vector.push_back(elch_trans);
			}

			qDebug() << "Done!";
		}
		
		//LUM correction
		if (settings->value("FINAL_SETTINGS/LUM_BASED_RECONSTRUCTION_PRE_LUM").toBool())
		{
			qDebug() << "Calculating LUM";

			//Converting PCD XYZRGB to XYZ
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcd_xyz_vector;
			for (int i = 0; i < mergedKeypointVector.size(); i++)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_xyz(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*mergedKeypointVector[i].get(), *pcd_xyz.get());
				pcd_xyz_vector.push_back(pcd_xyz);
			}

			pcl::registration::LUM<pcl::PointXYZ> lum;
			for (int i = 0; i < pcd_xyz_vector.size(); i++) //LUM add PCD's
				lum.addPointCloud(pcd_xyz_vector[i]);
			for (int i = 0; i < pcd_xyz_vector.size(); i++) //LUM add Correspondeces
			{
				boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
				for (int j = 0; j < correspondences_vector[i].size(); j++)
					correspondences.get()->push_back(correspondences_vector[i][j]);
				if (i < pcd_xyz_vector.size() - 1)
					lum.setCorrespondences(i, i + 1, correspondences);
				else
					lum.setCorrespondences(i, 0, correspondences);
			}
			lum.setMaxIterations(50);
			lum.setConvergenceThreshold(0.0);
			lum.compute();

			for (int i = 0; i < lum.getNumVertices(); i++)
			{
				Eigen::Matrix4f lum_trans = Eigen::Matrix4f::Identity();
				lum_trans = lum.getTransformation(i).matrix();
				final_translation_matrix_vector[i] = lum_trans * final_translation_matrix_vector[i];

				lum_final_translation_matrix_vector.push_back(lum_trans);
			}

			for (int i = 0; i < mergedKeypointVector.size(); i++)
				pcl::transformPointCloud(
					*mergedKeypointVector[i].get(),
					*mergedKeypointVector[i].get(),
					lum_final_translation_matrix_vector[i]
				);

			qDebug() << "Done!";
		}
	
		intial_transformation_matrix = final_translation_matrix_vector[final_translation_matrix_vector.size() - 1];

		/*
		Step 3:
		1) Perform TSDF meshing if needed
		2) Perform point cloud visualization if needed
		*/
		KeypointsFrames mergedKeypointFrames;
		for (int i = 1; i < mergedKeypointVector.size(); i++)
		{
			KeypointsFrame keypointsFrame;
			keypointsFrame.keypointsPcdPair = std::make_pair(mergedKeypointVector[i - 1], mergedKeypointVector[i]);
			keypointsFrame.keypointsPcdCorrespondences = correspondences_vector[i - 1];
			mergedKeypointFrames.push_back(keypointsFrame);
		}

		KeypointsFrame keypointsFrame;
		keypointsFrame.keypointsPcdPair = std::make_pair(mergedKeypointVector[mergedKeypointVector.size() - 1], mergedKeypointVector[0]);
		keypointsFrame.keypointsPcdCorrespondences = correspondences_vector[correspondences_vector.size() - 1];
		mergedKeypointFrames.push_back(keypointsFrame);

		if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
		{
			for (int i = 0; i < final_translation_matrix_vector.size(); i++)
				camera_pose_matrix_vector.push_back(final_translation_matrix_vector[i]);

			reorganize_all_point_clouds(frames);
			perform_tsdf_integration(frames, final_translation_matrix_vector);
		}
		else
		{
			for (int i = 0; i < frames.size(); i++)
				pcl::transformPointCloud(
					*frames[i].pointCloudPtr.get(),
					*frames[i].pointCloudPtr.get(),
					final_translation_matrix_vector[i]
				);

			if (settings->value("FINAL_SETTINGS/DRAW_ALL_CAMERA_POSES").toBool())
				pcdVizualizer->visualizeCameraPoses(final_translation_matrix_vector);
			if (settings->value("FINAL_SETTINGS/DRAW_ALL_CLOUDS").toBool())
				pcdVizualizer->visualizePointClouds(frames);
			if (settings->value("FINAL_SETTINGS/DRAW_ALL_KEYPOINT_CLOUDS").toBool())
				pcdVizualizer->visualizeKeypointClouds(mergedKeypointFrames);
		}
	}

	if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
		perform_tsdf_meshing(camera_pose_matrix_vector);
	
}

//#######################################################
//#-----------------------SLOTS--------------------------

void ReconstructionInterface::slot_perform_reconstruction()
{
	if (settings->value("FINAL_SETTINGS/DEFAUL_RECONSTRUCTION").toBool())
		perform_reconstruction();
	else if (settings->value("FINAL_SETTINGS/ITERATIVE_RECONSTRUCTION").toBool())
		perform_iterative_reconstruction();
	else if (settings->value("FINAL_SETTINGS/MIDDLE_BASED_RECONSTRUCTION").toBool())
		perform_partition_recursive_reconstruction();
	else if (settings->value("FINAL_SETTINGS/LUM_BASED_RECONSTRUCTION_ENABLE").toBool())
		perform_lum_reconstruction();
}

void ReconstructionInterface::slot_change_pair(int index)
{
	/*
	if (point_cloud_vector.empty() ||
		correspondences_vector.empty() ||
		keypoint_point_cloud_vector.empty())
	{
		return;
	}

	pcdVizualizer->redraw();

	pcdVizualizer->viewer.get()->addText(
		QString("CLOUD #%1 & #%2").arg(index - 1).arg(index).toStdString(),
		settings->value("VISUALIZATION/CLOUD_TEXT_X").toInt(),
		settings->value("VISUALIZATION/CLOUD_TEXT_Y").toInt(),
		settings->value("VISUALIZATION/CLOUD_TEXT_FONT_SIZE").toInt(),
		0, 0, 0,
		"clouds_text"
		);

	if (settings->value("FINAL_SETTINGS/DRAW_ALL_CLOUDS").toBool())
	{
		//Main clouds
		pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb1(point_cloud_vector[index]);
		pcdVizualizer->viewer.get()->addPointCloud<PointType>(point_cloud_vector[index], rgb1, QString("Cloud #%1").arg(index).toStdString());
		pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb2(point_cloud_vector[index - 1]);
		pcdVizualizer->viewer.get()->addPointCloud<PointType>(point_cloud_vector[index - 1], rgb2, QString("Cloud #%1").arg(index - 1).toStdString());
	}

	if (settings->value("FINAL_SETTINGS/DRAW_ALL_KEYPOINT_CLOUDS").toBool())
	{
		index = index - 1;
		//Correspondences
		pcdVizualizer->viewer.get()->addCorrespondences<PointType>(
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
		pcdVizualizer->viewer.get()->addPointCloud<PointType>(
			keypoint_point_cloud_vector[index].first, single_color,
			QString("Cloud P1 %1").arg(index).toStdString()
			);
		pcdVizualizer->viewer.get()->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,
			QString("Cloud P1 %1").arg(index).toStdString()
			);

		//Second keypoint point cloud 
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color1(
			keypoint_point_cloud_vector[index].second,
			0, 0, 255
			);
		pcdVizualizer->viewer.get()->addPointCloud<PointType>(
			keypoint_point_cloud_vector[index].second, single_color1,
			QString("Cloud P2 %1").arg(index).toStdString()
			);
		pcdVizualizer->viewer.get()->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,
			QString("Cloud P2 %1").arg(index).toStdString()
			);
	}
	*/
}

void ReconstructionInterface::slot_set_use_reconstruction(int value)
{
	if (value == 0)
		set_use_reconstruction(false);
	else
		set_use_reconstruction(true);
}

void ReconstructionInterface::slot_set_use_undistortion(int value)
{
	if (value == 0)
		set_use_undistortion(false);
	else
		set_use_undistortion(true);
}

void ReconstructionInterface::slot_set_use_bilateral_filter(int value)
{
	if (value == 0)
		set_use_bilateral_filter(false);
	else
		set_use_bilateral_filter(true);
}

void ReconstructionInterface::slot_set_use_statistical_outlier_removal_filter(int value)
{
	if (value == 0)
		set_use_statistical_outlier_removal_filter(false);
	else
		set_use_statistical_outlier_removal_filter(true);
}

void ReconstructionInterface::slot_set_use_moving_least_squares_filter(int value)
{
	if (value == 0)
		set_use_moving_least_squares_filter(false);
	else
		set_use_moving_least_squares_filter(true);
}
