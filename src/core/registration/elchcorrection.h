#ifndef ELCH_CORRECTION_HPP
#define ELCH_CORRECTION_HPP

#include "core/base/scannerbase.h"
#include "core/base/scannertypes.h"

class ElchCorrection : public ScannerBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ElchCorrection(QObject* parent, QSettings* parent_settings);

    void setInput(
        const PcdPtrVector& merged_keypoints_,
        const CorrespondencesVector& merged_correspondences_);

    Matrix4fVector correct();

    Matrix4fVector getTransformations();

private:
    CorrespondencesVector merged_correspondences;
    PcdPtrVector merged_keypoints;
    Matrix4fVector result_t;

    void calculate_elch_correction();
};

#endif //ELCH_CORRECTION_HPP