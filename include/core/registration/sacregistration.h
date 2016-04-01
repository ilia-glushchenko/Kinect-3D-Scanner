#ifndef SACREGISTRATION_H
#define SACREGISTRATION_H

#include "core/base/types.h"


class SaCRegistration : public ScannerBase
{
	Q_OBJECT

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef boost::shared_ptr<pcl::Correspondences> CorrespondencesPtr;

	SaCRegistration(QObject *parent, QSettings* parent_settings);
	
	void setInput(
		const KeypointsFrame & keypoints_frame_, 
		const Eigen::Matrix4f & initial_transformation_
	);

	Eigen::Matrix4f align();

	Eigen::Matrix4f getTransformation() const;

	float getFitnessScore() const;

private:
	const double inlier_threshold;
	const int max_iter;
	
	KeypointsFrame  keypoints_frame;
	Eigen::Matrix4f initial_transformation;
	Eigen::Matrix4f result_t;

	void calculate();

};

#endif // SACREGISTRATION_H
