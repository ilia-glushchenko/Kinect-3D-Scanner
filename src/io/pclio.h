#pragma once

#include <QString>

#include <boost/shared_ptr.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace pcl_io 
{

inline void save_one_point_cloud(
    const QString& filename,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & point_cloud_ptr)
{
    if (!filename.isEmpty()) {
        if (filename.contains(".pcd")) {
            pcl::io::savePCDFileBinary(filename.toStdString(), *point_cloud_ptr);
        }
        else if (filename.contains(".ply")) {
            pcl::io::savePLYFileASCII(filename.toStdString(), *point_cloud_ptr);
        }
    }
}

inline void save_one_polygon_mesh(
    const QString& filename,
    const pcl::PolygonMesh& mesh)
{
    if (!filename.isEmpty() && filename.contains(".ply")) {
        pcl::io::savePLYFile(filename.toStdString(), mesh);
    }
}

inline void save_point_cloud_vector(
    const QString& filename_pattern,
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& point_cloud_vector)
{
    if (!point_cloud_vector.empty() && !filename_pattern.isEmpty()) {
        for (int i = 0; i < point_cloud_vector.size(); i++) {
            pcl_io::save_one_point_cloud((filename_pattern.arg(i)), (point_cloud_vector[i]));
        }
    }
}

inline void load_one_point_cloud(
    const QString& filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud_ptr)
{
    if (!filename.isEmpty()) {
        if (filename.contains(".pcd")) {
            pcl::io::loadPCDFile(filename.toStdString(), *point_cloud_ptr);
        }
        else if (filename.contains(".ply")) {
            pcl::io::loadPLYFile(filename.toStdString(), *point_cloud_ptr);
        }
    }
}

inline void scale_one_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud_ptr)
{
    for (int j = 0; j < point_cloud_ptr->size(); j++) 
    {
        if (!std::isnan((*point_cloud_ptr)[j].z)) 
        {
            (*point_cloud_ptr)[j].z /= 1000.0f;
            (*point_cloud_ptr)[j].y /= 1000.0f;
            (*point_cloud_ptr)[j].x /= 1000.0f;
        }
    }
}

} // namespace pcl_io
