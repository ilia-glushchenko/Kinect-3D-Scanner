#ifndef KEYPOINTSREJECTION_H
#define KEYPOINTSREJECTION_H

#include <QObject>
#include <QSettings>
#include <QDebug>

#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include "types.h"

class KeypointsRejection : public QObject
{
	Q_OBJECT

public:
	KeypointsRejection(QObject *parent);
	void rejection(
		KeypointsFrames& in_keypointsFrames,
		KeypointsFrames& out_keypointsFrames
	);
	void rejection(
		KeypointsFrame& in_keypointsFrame,
		KeypointsFrame& out_keypointsFrame
	);

private:
	QSettings* settings;

	double inlier_threshold;
	int    max_iter;
	int    min_idsac_threshold;

	void perform_rejection(
		KeypointsFrames& in_keypointsFrames,
		KeypointsFrames& out_keypointsFrames
	);

	void perform_rejection(
		KeypointsFrame& in_keypointsFrame,
		KeypointsFrame& out_keypointsFrame
	);
	
	void iterative_decrementive_sample_consensus_rejection(
		KeypointsFrame& keypointsFrame,
		std::vector<int>& exclude_indexes
	);

	void calculate_sac(
		PcdPtr input_point_cloud_ptr,
		PcdPtr target_point_cloud_ptr,
		pcl::Correspondences& ñorrespondences,
		double inlier_threshold,
		int max_iter,
		pcl::Correspondences& inliers,
		Eigen::Matrix4f& out_transformation_matrix
	);

	void update_clouds(
		PcdPtr in_input_point_cloud_ptr1,
		PcdPtr in_traget_point_cloud_ptr2,
		pcl::Correspondences in_correspondences,
		PcdPtr out_input_point_cloud_ptr1,
		PcdPtr out_traget_point_cloud_ptr2,
		pcl::Correspondences& out_correspondences
	);

	void add_camera_pose_points(
		PcdPtr input_point_cloud_ptr,
		PcdPtr target_point_cloud_ptr,
		pcl::Correspondences& correspondences,
		Eigen::Matrix4f& input_transformation_matrix,
		Eigen::Matrix4f& target_transformation_matrix
	);

	void copyKeypointsFrame(
		KeypointsFrame& in_frame,
		KeypointsFrame& out_frame
	);
};

#endif // KEYPOINTSREJECTION_H
