#include "io/pclio.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

template< typename PointT >
void pclio::save_one_point_cloud(
	const QString & filename,
	const boost::shared_ptr<pcl::PointCloud<PointT>> & point_cloud_ptr
	)
{
	if (!filename.isEmpty())
	{
		if (filename.contains(".pcd"))
		{
			pcl::io::savePCDFileBinary(filename.toStdString(), *point_cloud_ptr);
		}
		else if (filename.contains(".ply"))
		{
			pcl::io::savePLYFileASCII(filename.toStdString(), *point_cloud_ptr);
		}
	}
}

template void pclio::save_one_point_cloud(
	const QString & filename,
	const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & point_cloud_ptr
);

template void pclio::save_one_point_cloud(
	const QString & filename,
	const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & point_cloud_ptr
);


void pclio::save_one_polygon_mesh(
	const QString & filename,
	const pcl::PolygonMesh & mesh
	)
{
	if (!filename.isEmpty() && filename.contains(".ply"))
	{
		pcl::io::savePLYFile(filename.toStdString(), mesh);
	}
}


template< typename PointT >
void pclio::save_point_cloud_vector(
	const QString & filename_pattern,
	const std::vector<boost::shared_ptr<pcl::PointCloud<PointT>>> & point_cloud_vector
	)
{
	if (!point_cloud_vector.empty() && !filename_pattern.isEmpty())
	{
		for (int i = 0; i < point_cloud_vector.size(); i++)
		{
			pclio::save_one_point_cloud((filename_pattern.arg(i)), (point_cloud_vector[i]));
		}
	}
}

template void pclio::save_point_cloud_vector(
	const QString & filename_pattern,
	const std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> & point_cloud_vector
);

template void pclio::save_point_cloud_vector(
	const QString & filename_pattern,
	const std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> & point_cloud_vector
);


template< typename PointT >
void pclio::load_one_point_cloud(
	const QString & filename,
	boost::shared_ptr<pcl::PointCloud<PointT>> & point_cloud_ptr
	)
{
	if (!filename.isEmpty())
	{
		if (filename.contains(".pcd"))
		{
			pcl::io::loadPCDFile(filename.toStdString(), *point_cloud_ptr);
		}
		else if (filename.contains(".ply"))
		{
			pcl::io::loadPLYFile(filename.toStdString(), *point_cloud_ptr);
		}
	}
}

template void pclio::load_one_point_cloud(
	const QString & filename,
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & point_cloud_ptr
);

template void pclio::load_one_point_cloud(
	const QString & filename,
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & point_cloud_ptr
);


template< typename PointT >
void pclio::scale_one_point_cloud(
	boost::shared_ptr<pcl::PointCloud<PointT>> & point_cloud_ptr
	)
{
	for (int j = 0; j < point_cloud_ptr->size(); j++)
	{
		if (!pcl_isnan((*point_cloud_ptr)[j].z))
		{
			(*point_cloud_ptr)[j].z /= 1000.0f;
			(*point_cloud_ptr)[j].y /= 1000.0f;
			(*point_cloud_ptr)[j].x /= 1000.0f;
		}
	}
}

template void pclio::scale_one_point_cloud(
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & point_cloud_ptr
);

template void pclio::scale_one_point_cloud(
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & point_cloud_ptr
);
