#include "core/reconstruction/reconstructioninterface.h"

#include <pcl/common/distances.h>

#include "core/registration/icpregistration.h"
#include "core/registration/sacregistration.h"
#include "core/registration/linearregistration.hpp"
#include "core/registration/parallelregistration.hpp"
#include "core/registration/middlebasedregistration.hpp"
#include "core/registration/edgebasedregistration.hpp"
#include "core/registration/linearbasedregistration.hpp"
#include "io/pcdinputiterator.hpp"

//#######################################################


ReconstructionInterface::ReconstructionInterface(QObject *parent, QSettings* parent_settings) :
	ScannerBase(parent, parent_settings),
	volumeReconstruction(new VolumeReconstruction(this, settings)),
	pcdVizualizer(new PcdVizualizer(this, settings)),
	use_reconstruction(false),
	use_undistortion(false),
	use_bilateral_filter(false),
	use_statistical_outlier_removal_filter(false),
	use_moving_least_squares_filter(false)
{
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
//#------------------RECONSTRUCTION----------------------

void ReconstructionInterface::perform_tsdf_integration(
		Frames & frames,
		Matrix4fVector & final_translation_matrix_vector
	)
{
	PcdPtrVector point_cloud_vector;
	std::transform(frames.begin(), frames.end(), std::back_inserter(point_cloud_vector),
		[](const Frame & frame){ return frame.pointCloudPtr; });

	volumeReconstruction->addPointCloudVector(
		point_cloud_vector, final_translation_matrix_vector
		);
}


void ReconstructionInterface::perform_tsdf_meshing(
		Matrix4fVector& final_translation_matrix_vector
	)
{
	volumeReconstruction->prepareVolume();
	pcdVizualizer->redraw();

	if (settings->value("FINAL_SETTINGS/DRAW_ALL_CAMERA_POSES").toBool()) {
		pcdVizualizer->visualizeCameraPoses(final_translation_matrix_vector);
	}

	if (settings->value("CPU_TSDF_SETTINGS/DRAW_MESH").toBool())
	{
		pcl::PolygonMesh mesh;
		volumeReconstruction->calculateMesh();
		volumeReconstruction->getPoligonMesh(mesh);
		pcdVizualizer->visualizeMesh(mesh);
	}
}


void ReconstructionInterface::perform_reconstruction()
{
	const int from = settings->value("READING_SETTING/FROM").toInt();
	const int to   = settings->value("READING_SETTING/TO").toInt();
	const int step = settings->value("READING_SETTING/STEP").toInt();
	Matrix4fVector final_transformations;
	std::vector<double> fit;
	std::vector<double> mean;

	PcdInputIterator it(settings, from, to, step);
	PcdInputIterator it2(settings, from + 1, to, step);
	for (; it != PcdInputIterator() && it2 != PcdInputIterator(); ++it, ++it2)
	{
		Frames frames, transformed_frames;

		frames.push_back(*it);
		frames.push_back(*it2);

		PcdFilters filters(this, settings);
		filters.setInput(frames);
		filters.filter(frames);;

		LinearRegistration<SaCRegistration> linear_sac(this, settings);
		linear_sac.setInput(frames, Eigen::Matrix4f(Eigen::Matrix4f::Identity()));
		Matrix4fVector sac_transformations = linear_sac.align(transformed_frames);

		LinearRegistration<ICPRegistration> linear_icp(this, settings);
		linear_icp.setInput(transformed_frames, Eigen::Matrix4f(Eigen::Matrix4f::Identity()));
		linear_icp.setKeypoints(linear_sac.getTransformedKeypoints());
		Matrix4fVector icp_transformations = linear_icp.align(transformed_frames);

		for (uint i = 1; i < icp_transformations.size(); ++i)
		{
			final_transformations.push_back(icp_transformations[i] * sac_transformations[i]);
		}

		const KeypointsFrames & kp = linear_icp.getTransformedKeypoints();
		for (uint i = 0; i < kp.size(); ++i)
		{
			std::vector<double> dis;
			for (uint j = 0; j < kp[i].keypointsPcdPair.first->size(); ++j)
			{
				const PointType & a = (*kp[i].keypointsPcdPair.first)[j]; 
				const PointType & b = (*kp[i].keypointsPcdPair.second)[j];
				dis.push_back(pcl::euclideanDistance(a, b));
			}

			mean.push_back(
				std::accumulate(dis.begin(), dis.end(), 0.0f,
				[](const double & sum, const double & n){ return sum + n; }) / dis.size()
			);
		}

		const auto f = linear_icp.getFitnessScores();
		std::copy(f.begin(), f.end(), std::back_inserter(fit));
	}
		
	pcdVizualizer->visualizeCameraPoses(final_transformations);
	pcdVizualizer->plotCameraDistances(fit, false, "FS", "Score");
	pcdVizualizer->plotCameraDistances(mean, false, "Mean", "M Dist");
	/*
	if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool()) 
	{
		perform_tsdf_integration(frames, final_transformations);
		perform_tsdf_meshing(final_transformations);
	}
	else
	{
		pcdVizualizer->redraw();
		if (settings->value("FINAL_SETTINGS/DRAW_ALL_CAMERA_POSES").toBool())
			pcdVizualizer->visualizeCameraPoses(final_transformations);
		if (settings->value("FINAL_SETTINGS/DRAW_ALL_CLOUDS").toBool())
			pcdVizualizer->visualizePointClouds(transformed_frames);
		if (settings->value("FINAL_SETTINGS/DRAW_ALL_KEYPOINT_CLOUDS").toBool())
			pcdVizualizer->visualizeKeypointClouds(linear_icp.getTransformedKeypoints());
	}
	*/
}


void ReconstructionInterface::perform_iterative_reconstruction()
{
	LinearBasedRegistration lbr(this, settings);
	lbr.setVolumeReconstructor(volumeReconstruction);
	lbr.setVisualizer(pcdVizualizer);
	lbr.reconstruct();
}


void ReconstructionInterface::perform_partition_recursive_reconstruction()
{
	MiddleBasedRegistration mbr(this, settings);
	mbr.setVolumeReconstructor(volumeReconstruction);
	mbr.setVisualizer(pcdVizualizer);
	mbr.reconstruct();
}


void ReconstructionInterface::perform_lum_reconstruction()
{
	EdgeBasedRegistration ebr(this, settings);
	ebr.setVolumeReconstructor(volumeReconstruction);
	ebr.setVisualizer(pcdVizualizer);
	ebr.reconstruct();
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
