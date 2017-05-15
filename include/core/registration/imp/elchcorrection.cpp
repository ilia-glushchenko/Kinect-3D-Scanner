#include "core/registration/elchcorrection.h"

#include <QDebug>
#include <pcl/registration/elch.h>

ElchCorrection::ElchCorrection(QObject* parent, QSettings* parent_settings)
    : ScannerBase(parent, parent_settings)
{
}

void ElchCorrection::setInput(
    const PcdPtrVector& merged_keypoints_,
    const CorrespondencesVector& merged_correspondences_)
{
    merged_keypoints = merged_keypoints_;
    merged_correspondences = merged_correspondences_;
}

Matrix4fVector ElchCorrection::correct()
{
    calculate_elch_correction();
    return result_t;
}

Matrix4fVector ElchCorrection::getTransformations()
{
    return result_t;
}

void ElchCorrection::calculate_elch_correction()
{
    qDebug() << "Calculating ELCH";

    pcl::registration::ELCH<pcl::PointXYZRGB> elch;
    for (int i = 0; i < merged_keypoints.size(); i++) {
        elch.addPointCloud(merged_keypoints[i]);
    }

    pcl::registration::ELCH<pcl::PointXYZRGB>::RegistrationPtr reg = elch.getReg();
    reg->setMaxCorrespondenceDistance(configs.value("SAC_SETTINGS/INLIER_THRESHOLD").toDouble());
    reg->setMaximumIterations(configs.value("SAC_SETTINGS/MAX_ITERATIONS").toInt());
    reg->setEuclideanFitnessEpsilon(configs.value("ICP_SETTINGS/EUCLIDEAN_EPSILON").toDouble());
    reg->setTransformationEpsilon(configs.value("ICP_SETTINGS/TRANSFORMATION_EPSILON").toDouble());

    elch.setLoopStart(0);
    elch.setLoopEnd(merged_keypoints.size() - 1);
    elch.compute();

    pcl::registration::ELCH<pcl::PointXYZRGB>::LoopGraphPtr loopGraphPtr;
    loopGraphPtr = elch.getLoopGraph();
    for (int i = 0; i < merged_keypoints.size(); i++) {
        result_t.push_back((*loopGraphPtr)[i].transform.matrix());
    }

    qDebug() << "Done!";
}
