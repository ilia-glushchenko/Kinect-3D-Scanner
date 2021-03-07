#ifndef ICPREGISTRATION_H
#define ICPREGISTRATION_H

#include <QObject>

#include "core/base/scannertypes.h"

class ICPRegistration : public ScannerBase {
    Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ICPRegistration(QObject* parent, QSettings* parent_settings);

    void setInput(
        const KeypointsFrame& keypoints_frame_,
        const Eigen::Matrix4f& initial_transformation_);

    Eigen::Matrix4f align();

    Eigen::Matrix4f getTransformation() const;

    float getFitnessScore() const;

private:
    KeypointsFrame keypoints_frame;
    Eigen::Matrix4f initial_transformation;
    Eigen::Matrix4f result_t;
    float fitness_score;

    void calculate();
};

#endif // ICPREGISTRATION_H
