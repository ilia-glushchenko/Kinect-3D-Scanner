#include "core/registration/sacregistration.h"

#include <QDebug>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

SaCRegistration::SaCRegistration(QObject* parent, QSettings* parent_settings)
    : ScannerBase(parent, parent_settings)
    , inlier_threshold(configs.value("SAC_SETTINGS/INLIER_THRESHOLD").toDouble())
    , max_iter(configs.value("SAC_SETTINGS/MAX_ITERATIONS").toInt())
{
}

void SaCRegistration::setInput(
    const KeypointsFrame& keypoints_frame_,
    const Eigen::Matrix4f& initial_transformation_)
{
    keypoints_frame = keypoints_frame_;
    initial_transformation = initial_transformation_;
}

Eigen::Matrix4f SaCRegistration::align()
{
    result_t = Eigen::Matrix4f::Identity();
    calculate();
    return result_t;
}

Eigen::Matrix4f SaCRegistration::getTransformation() const
{
    return result_t;
}

float SaCRegistration::getFitnessScore() const
{
    return 0;
}

void SaCRegistration::calculate()
{
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> sac;
    sac.setInputSource(keypoints_frame.keypointsPcdPair.second);
    sac.setInputTarget(keypoints_frame.keypointsPcdPair.first);
    sac.setInlierThreshold(DISABLED_INLIER_THRESHOLD);
    sac.setMaximumIterations(max_iter);
    sac.setInputCorrespondences(
        CorrespondencesPtr(new pcl::Correspondences(keypoints_frame.keypointsPcdCorrespondences)));
    pcl::Correspondences tmp_correspondences;
    sac.getCorrespondences(tmp_correspondences);

    result_t = initial_transformation * sac.getBestTransformation();

    //Transform Keypoint clouds
    pcl::transformPointCloud(
        *keypoints_frame.keypointsPcdPair.first, *keypoints_frame.keypointsPcdPair.first, initial_transformation);
    pcl::transformPointCloud(
        *keypoints_frame.keypointsPcdPair.second, *keypoints_frame.keypointsPcdPair.second, result_t);
}
