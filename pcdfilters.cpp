#include "pcdfilters.h"

#define WIDTH  640
#define HEIGHT 480

PcdFilters::PcdFilters(QObject *parent) : QObject(parent)
{

}

void PcdFilters::apply_bilateral_filter(
	PcdPtr src_point_cloud_ptr, int d, double sigma_color, double sigma_space
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

	cv::bilateralFilter(img, img_res, d, sigma_color, sigma_space);

	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
			if (img_res.at<float>(y, x) == 0)
				src_point_cloud_ptr.get()->points[x + y*WIDTH].z = NAN;
			else
				src_point_cloud_ptr.get()->points[x + y*WIDTH].z = img_res.at<float>(y, x);
}

void PcdFilters::apply_statistical_outlier_removal_filter(
	PcdPtr in_point_cloud_ptr, PcdPtr out_point_cloud_ptr, int meanK, float stddevMulThresh
	)
{
	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setInputCloud(in_point_cloud_ptr);
	sor.setKeepOrganized(true);
	sor.setMeanK(meanK);
	sor.setStddevMulThresh(stddevMulThresh);
	sor.filter(*out_point_cloud_ptr.get());
}

void PcdFilters::apply_moving_least_squares_filter(
	PcdPtr point_cloud_ptr_in, 
	pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_ptr_out, 
	double sqrGaussParam, double searchRadius
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
	PcdPtr in_point_cloud_ptr,
	PcdPtr out_point_cloud_ptr,
	float leaf_x, float leaf_y, float leaf_z
	)
{
	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud(in_point_cloud_ptr);
	sor.setLeafSize(leaf_x, leaf_y, leaf_z);
	sor.filter(*out_point_cloud_ptr.get());
	qDebug() << "Reduced from"
			 << in_point_cloud_ptr.get()->points.size()
			 << "to"
			 << out_point_cloud_ptr.get()->points.size();
}
