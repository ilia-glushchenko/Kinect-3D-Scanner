#ifndef LUM_CORRECTION_H
#define LUM_CORRECTION_H

#include "core/base/types.h"
#include "core/base/scannerbase.h"

class LumCorrection : public ScannerBase
{
public:
	typedef boost::shared_ptr<pcl::Correspondences> CorrespondencesPtr;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


	LumCorrection(QObject * parent, QSettings * parent_settings);

	void setInput(
		const PcdPtrVector & merged_keypoints_,
		const CorrespondencesVector & merged_correspondences_
	);


	Matrix4fVector correct();

	Matrix4fVector getTransformations() const;

private:
	CorrespondencesVector merged_correspondences;
	PcdPtrVector merged_keypoints;
	Matrix4fVector result_t;

	void calculate_lum_correction();
	
};

#endif //LUM_CORRECTION_H