#ifndef PCDFILTERS_H
#define PCDFILTERS_H

#include <QObject>
#include <QSettings>
#include <QDebug>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>

#include "types.h"

class PcdFilters : public QObject
{
	Q_OBJECT

public:
	PcdFilters(QObject *parent);

	static void apply_bilateral_filter(
		PcdPtr src_point_cloud_ptr
	);

	static void apply_statistical_outlier_removal_filter(
		PcdPtr in_point_cloud_ptr,
		PcdPtr out_point_cloud_ptr
	);
	static void apply_moving_least_squares_filter(
		PcdPtr point_cloud_ptr
	);
	static void apply_voxel_grid_reduction(
		PcdPtr in_point_cloud_ptr,
		PcdPtr out_point_cloud_ptr,
		float x_k = 1.0f, float y_k = 1.0f, float z_k = 1.0f
	);

};

#endif // PCDFILTERS_H
