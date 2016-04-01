#ifndef PCLIO_H
#define PCLIO_H

#include <QString>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <boost/shared_ptr.hpp>

namespace pclio
{
	template< typename PointT > 
	void save_one_point_cloud(
		const QString & filename,
		const boost::shared_ptr<pcl::PointCloud<PointT>> & point_cloud_ptr
	);

	void save_one_polygon_mesh(
		const QString & filename,
		const pcl::PolygonMesh & mesh
	);

	template< typename PointT > 
	void save_point_cloud_vector(
		const QString & filename_pattern,
		const std::vector<boost::shared_ptr<pcl::PointCloud<PointT>>> & point_cloud_vector
	);

	template< typename PointT >
	void load_one_point_cloud(
		const QString & filename,
		boost::shared_ptr<pcl::PointCloud<PointT>> & point_cloud_ptr
	);

	template< typename PointT >
	void scale_one_point_cloud(
		boost::shared_ptr<pcl::PointCloud<PointT>> & point_cloud_ptr
	);
};

#endif // PCLIO_H
