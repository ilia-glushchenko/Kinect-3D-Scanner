#ifndef ICPREGISTRATION_H
#define ICPREGISTRATION_H

#include <QObject>
#include <QDebug>
#include <QSettings>

#include <pcl/filters/filter.h>
#include <pcl/registration/icp_nl.h>

#include "core/base/types.h"

class ICPRegistration : public ScannerBase
{
	Q_OBJECT

public:
	ICPRegistration(QObject *parent, QSettings* parent_settings);

	void calculateICPTransformation(
		KeypointsFrames& keypointsFrames,
		Eigen::Matrix4f& intial_transformation_matrix,
		int middle_index =- 1
	);
	void applyTransfomation(
		Frames& frames
	);
	void getTransformation(
		Matrix4fVector& icp_translation_matrix_vector
	);

private:
	void prepare();
	Matrix4fVector _icp_translation_matrix_vector;

	void calculate_one_keypoint_pair_icp(
		KeypointsFrame& keypointsFrame,
		Eigen::Matrix4f& transformation_matrix
	);
	void calculate_all_keypoint_pair_icp(
		KeypointsFrames& keypointsFrames,
		Eigen::Matrix4f& intial_transformation_matrix
	);
	void calculate_middle_based_all_keypoint_pair_icp(
		KeypointsFrames& keypointsFrames,
		Eigen::Matrix4f& intial_transformation_matrix,
		int middle_index
	);
	void apply_icp_vector(
		Frames& frames
	);
};

#endif // ICPREGISTRATION_H
