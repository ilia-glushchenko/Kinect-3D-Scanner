#ifndef SACREGISTRATION_H
#define SACREGISTRATION_H

#include <QObject>
#include <QSettings>
#include <QString>
#include <QDebug>

#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include "types.h"

class SaCRegistration : public ScannerBase
{
	Q_OBJECT

public:
	SaCRegistration(QObject *parent, QSettings* parent_settings);
	
	void calculateSaCTransformation(
		KeypointsFrames& keypointsFrames,
		Eigen::Matrix4f& intial_transformation_matrix,
		int middle_index = -1
	);
	void applyTransfomation(
		Frames& frames
	);
	void getTransformation(
		Matrix4fVector& sac_translation_matrix_vector
	);

private:
	double inlier_threshold;
	int    max_iter;

	void prepare();
	Matrix4fVector _sac_translation_matrix_vector;

	void calculate_sac(
		KeypointsFrame& keypointsFrame,
		double inlier_threshold, 
		int max_iter,
		pcl::Correspondences& inliers, 
		Eigen::Matrix4f& out_transformation_matrix
	);
	void calculate_one_keypoint_pair_sac(
		KeypointsFrame& keypointsFrame,
		Eigen::Matrix4f& transformation_matrix
	);
	void calculate_all_keypoint_pair_sac(
		KeypointsFrames& keypointsFrames,
		Eigen::Matrix4f& intial_transformation_matrix
	);
	void calculate_middle_based_all_keypoint_pair_sac(
		KeypointsFrames& keypointsFrames,
		Eigen::Matrix4f& intial_transformation_matrix,
		int middle_index
	);
	void apply_sac_vector(
		Frames& frames
	);	

	void copyKeypointsFrame(
		KeypointsFrame& in_frame,
		KeypointsFrame& out_frame
	);
};

#endif // SACREGISTRATION_H
