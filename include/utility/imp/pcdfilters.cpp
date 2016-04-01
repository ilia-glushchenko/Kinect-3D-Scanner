#include "utility/pcdfilters.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>

#include "io/calibrationinterface.h"


PcdFilters::PcdFilters(QObject *parent, QSettings* parent_settings) : 
	ScannerBase(parent, parent_settings),
	undistortion(settings->value("CALIBRATION/ENABLE_IN_FINAL").toBool()),
	bilateral(settings->value("OPENCV_BILATERAL_FILTER_SETTINGS/ENABLE_IN_FINAL").toBool()),
	statistical(settings->value("STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/ENABLE_IN_FINAL").toBool()),
	mls(settings->value("MOVING_LEAST_SQUARES_FILTER_SETTINGS/ENABLE_IN_FINAL").toBool()),
	voxel_grid(settings->value("VOXEL_GRID_REDUCTION_SETTINGS/ENABLE_IN_FINAL").toBool())
{
}

void PcdFilters::setInput(const Frames & input_frames)
{
	frames = input_frames;
}

void PcdFilters::filter(Frames & filtered_frames)
{
	if (undistortion)
	{
		CalibrationInterface calibrationInterface(this, settings);
		calibrationInterface.loadCalibrationData();
		calibrationInterface.calibrate();
		calibrationInterface.undistort(frames);
	}

	filter_all_frames(frames);
	reorganize_all_frames(frames);
		
	filtered_frames = frames;
}

Frames PcdFilters::getFilteredFrames()
{
	return frames;
}


void PcdFilters::filter_all_frames(Frames & frames)
{
	for (uint i = 0; i < frames.size(); ++i)
	{
		filter_one_frame(frames[i]);
	}
}


void PcdFilters::reorganize_all_frames(Frames & frames)
{
	qDebug() << "Reorganization all point clouds";
	for (uint i = 0; i < frames.size(); ++i)
	{
		if (frames[i].pointCloudPtr->height == HEIGHT)
		{
			continue;
		}

		PcdPtr tmp_pcd_ptr(new Pcd);
		tmp_pcd_ptr->width = WIDTH;
		tmp_pcd_ptr->height = HEIGHT;
		tmp_pcd_ptr->resize(HEIGHT*WIDTH);

		for (int y = 0; y < HEIGHT; y++)
		{
			for (int x = 0; x < WIDTH; x++)
			{
				tmp_pcd_ptr->at(x, y).z = NAN;
			}
		}

		for (int j = 0; j < frames[i].pointCloudPtr->size(); j++)
		{
			(*tmp_pcd_ptr)[frames[i].pointCloudIndexes[j]] = (*frames[i].pointCloudPtr)[j];
		}

		pcl::copyPointCloud(*tmp_pcd_ptr, *frames[i].pointCloudPtr);
		frames[i].pointCloudPtr->is_dense = false;
	}
	qDebug() << "Done!";
}


void PcdFilters::filter_one_frame(Frame & frame)
{
	PcdPtr dest_point_cloud_ptr(new Pcd);

	//Bilateral
	if (bilateral)
	{
		const int d = settings->value("OPENCV_BILATERAL_FILTER_SETTINGS/D").toInt();
		const double sigma_color = settings->value("OPENCV_BILATERAL_FILTER_SETTINGS/SIGMA_COLOR").toDouble();
		const double sigma_space = settings->value("OPENCV_BILATERAL_FILTER_SETTINGS/SIGMA_SPACE").toDouble();
		apply_bilateral_filter(frame.pointCloudPtr, d, sigma_color, sigma_space);
	}

	pcl::removeNaNFromPointCloud(*frame.pointCloudPtr, *dest_point_cloud_ptr, frame.pointCloudIndexes);

	//Statistic reduction
	if (statistical) 
	{
		const int meanK = settings->value("STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/MEAN_K").toInt();
		const float stddevMulThresh = settings->value("STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/MUL_THRESH").toFloat();
		apply_statistical_outlier_removal_filter(dest_point_cloud_ptr, frame.pointCloudPtr, meanK, stddevMulThresh);

		if (settings->value("STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/ENABLE_LOG").toBool()) 
		{
			qDebug() 
				<< "Statistical removal: from " 
				<< dest_point_cloud_ptr->size() 
				<< "to" 
				<< frame.pointCloudPtr->size();
		}

		pcl::copyPointCloud(*frame.pointCloudPtr, *dest_point_cloud_ptr);
	}

	//Smooth
	if (mls) 
	{
		const double sqrGaussParam = settings->value("MOVING_LEAST_SQUARES_FILTER_SETTINGS/SQR_GAUSS_PARAM").toDouble();
		const double searchRadius = settings->value("MOVING_LEAST_SQUARES_FILTER_SETTINGS/SEARCH_RADIUS").toDouble();
		NormalPcdPtr smoothed_cloud(new NormalPcd);
		apply_moving_least_squares_filter(dest_point_cloud_ptr, smoothed_cloud, sqrGaussParam, searchRadius);
		pcl::copyPointCloud(*smoothed_cloud, *dest_point_cloud_ptr);
	}

	pcl::copyPointCloud(*dest_point_cloud_ptr, *frame.pointCloudPtr);
}



void PcdFilters::apply_bilateral_filter(
		PcdPtr & src_point_cloud_ptr,
		const int & d,
		const double & sigma_color,
		const double & sigma_space
	)
{
	cv::Mat img, img_res;
	img_res.create(HEIGHT, WIDTH, CV_32FC1);
	img.create(HEIGHT, WIDTH, CV_32FC1);

	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			img.at<float>(y, x) 
				= pcl_isnan((*src_point_cloud_ptr)[x + y*WIDTH].z) ? 0 : (*src_point_cloud_ptr)[x + y*WIDTH].z;
		}
	}

	cv::bilateralFilter(img, img_res, d, sigma_color, sigma_space);

	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			(*src_point_cloud_ptr)[x + y*WIDTH].z = img_res.at<float>(y, x) == 0 ? NAN : img_res.at<float>(y, x);
		}
	}
}

void PcdFilters::apply_statistical_outlier_removal_filter(
		PcdPtr & in_point_cloud_ptr,
		PcdPtr & out_point_cloud_ptr,
		const int & meanK,
		const float & stddevMulThresh
	)
{
	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setInputCloud(in_point_cloud_ptr);
	sor.setKeepOrganized(true);
	sor.setMeanK(meanK);
	sor.setStddevMulThresh(stddevMulThresh);
	sor.filter(*out_point_cloud_ptr);
}

void PcdFilters::apply_moving_least_squares_filter(
	PcdPtr & point_cloud_ptr_in,
	pcl::PointCloud<pcl::PointNormal>::Ptr & point_cloud_ptr_out,
	const double & sqrGaussParam,
	const double & searchRadius
	)
{
	//ToDo: Test. It might not work due to absence of normal in input cloud!
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
	pcl::PointCloud<pcl::PointNormal> mls_points;

	pcl::MovingLeastSquares<PointType, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(point_cloud_ptr_in);
	mls.setPolynomialFit(true);
	mls.setSqrGaussParam(sqrGaussParam);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(searchRadius);
	mls.process(*point_cloud_ptr_out);
}

void PcdFilters::apply_voxel_grid_reduction(
		PcdPtr & in_point_cloud_ptr,
		PcdPtr & out_point_cloud_ptr,
		const float & leaf_x,
		const float & leaf_y,
		const float & leaf_z
	)
{
	pcl::VoxelGrid<PointType> vg;
	vg.setInputCloud(in_point_cloud_ptr);
	vg.setLeafSize(leaf_x, leaf_y, leaf_z);
	vg.filter(*out_point_cloud_ptr);

	qDebug() << "Reduced from"
		<< in_point_cloud_ptr.get()->points.size()
		<< "to"
		<< out_point_cloud_ptr.get()->points.size();
}
