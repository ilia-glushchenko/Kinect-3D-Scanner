#ifndef PCDFILTERS_H
#define PCDFILTERS_H

#include <QObject>
#include <QDebug>

#include "core/base/types.h"
#include "core/base/scannerbase.h"

class PcdFilters : public ScannerBase
{
	Q_OBJECT

public:
	PcdFilters(QObject *parent, QSettings* parent_settings);

	void setInput(const Frames & input_frames);

	void filter(Frames & filtered_frames);

	static void reorganize_all_frames(Frames & frames);

	Frames getFilteredFrames();
	

private:
	const bool undistortion;
	const bool bilateral;
	const bool statistical;
	const bool mls;
	const bool voxel_grid;
	Frames frames;

	void filter_all_frames(Frames & frames);

	void filter_one_frame(Frame & frame);

	void apply_bilateral_filter(
		PcdPtr & src_point_cloud_ptr,
		const int & d,
		const double & sigma_color,
		const double & sigma_space
	);

	void apply_statistical_outlier_removal_filter(
		PcdPtr & in_point_cloud_ptr,
		PcdPtr & out_point_cloud_ptr,
		const int & meanK,
		const float & stddevMulThresh
	);

	void apply_moving_least_squares_filter(
		PcdPtr & point_cloud_ptr_in,
		pcl::PointCloud<pcl::PointNormal>::Ptr & point_cloud_ptr_out,
		const double & sqrGaussParam,
		const double & searchRadius
	);

	void apply_voxel_grid_reduction(
		PcdPtr & in_point_cloud_ptr,
		PcdPtr & out_point_cloud_ptr,
		const float & leaf_x,
		const float & leaf_y,
		const float & leaf_z
	);
};

#endif // PCDFILTERS_H
