#include "pcdfilters.h"

#define WIDTH  640
#define HEIGHT 480

PcdFilters::PcdFilters(
	QObject *parent
	)
{
	setParent(parent);
}

void PcdFilters::apply_bilateral_filter(
	PcdPtr src_point_cloud_ptr
	)
{
	QSettings settings("scaner.ini", QSettings::IniFormat);

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
		settings.value(
		"OPENCV_BILATERAL_FILTER_SETTINGS/D"
		).toInt();
	double sigma_color =
		settings.value(
		"OPENCV_BILATERAL_FILTER_SETTINGS/SIGMA_COLOR"
		).toDouble();
	double sigma_space =
		settings.value(
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

void PcdFilters::apply_statistical_outlier_removal_filter(
	PcdPtr in_point_cloud_ptr,
	PcdPtr out_point_cloud_ptr
	)
{
	QSettings settings("scaner.ini", QSettings::IniFormat);

	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setInputCloud(in_point_cloud_ptr);
	sor.setKeepOrganized(true);
	sor.setMeanK(
		settings.value(
		"STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/MEAN_K"
		).toInt()
		);
	sor.setStddevMulThresh(
		settings.value(
		"STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/MUL_THRESH"
		).toFloat()
		);
	sor.filter(*out_point_cloud_ptr.get());

	if (settings.value(
		"STATISTICAL_OUTLIER_REMOVAL_FILTER_SETTINGS/ENABLE_LOG"
		).toBool())
	{
		qDebug() << "Statistical removal: from "
			<< out_point_cloud_ptr.get()->points.size()
			<< "to"
			<< in_point_cloud_ptr.get()->points.size();
	}

}

void PcdFilters::apply_moving_least_squares_filter(
	PcdPtr point_cloud_ptr
	)
{
	QSettings settings("scaner.ini", QSettings::IniFormat);

	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
	pcl::PointCloud<pcl::PointNormal> mls_points;
	pcl::MovingLeastSquares<PointType, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(point_cloud_ptr);
	mls.setPolynomialFit(true);
	mls.setSqrGaussParam(
		settings.value(
			"MOVING_LEAST_SQUARES_FILTER_SETTINGS/SQR_GAUSS_PARAM"
		).toFloat()
	);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(
		settings.value(
			"MOVING_LEAST_SQUARES_FILTER_SETTINGS/SEARCH_RADIUS"
		).toFloat()
	);
}

void PcdFilters::apply_voxel_grid_reduction(
	PcdPtr in_point_cloud_ptr,
	PcdPtr out_point_cloud_ptr,
	float x_k, float y_k, float z_k
	)
{
	QSettings settings("scaner.ini", QSettings::IniFormat);

	float leaf_x = settings.value("VOXEL_GRID_REDUCTION_SETTINGS/LEAF_X").toFloat();
	float leaf_y = settings.value("VOXEL_GRID_REDUCTION_SETTINGS/LEAF_Y").toFloat();
	float leaf_z = settings.value("VOXEL_GRID_REDUCTION_SETTINGS/LEAF_Z").toFloat();

	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud(in_point_cloud_ptr);
	sor.setLeafSize(leaf_x * x_k, leaf_y * y_k, leaf_z * z_k);
	sor.filter(*out_point_cloud_ptr.get());
	qDebug() << "Reduced from"
			 << in_point_cloud_ptr.get()->points.size()
			 << "to"
			 << out_point_cloud_ptr.get()->points.size();
}
