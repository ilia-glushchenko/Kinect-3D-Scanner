#include "core/registration/icpregistration.h"

#include <QDebug>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>

ICPRegistration::ICPRegistration(QObject* parent, QSettings* parent_settings)
    : ScannerBase(parent, parent_settings)
    , initial_transformation(Eigen::Matrix4f::Identity())
    , result_t(Eigen::Matrix4f::Identity())
    , fitness_score(0)
{
}

void ICPRegistration::setInput(
    const KeypointsFrame& keypoints_frame_,
    const Eigen::Matrix4f& initial_transformation_)
{
    keypoints_frame = keypoints_frame_;
    initial_transformation = initial_transformation_;
}

Eigen::Matrix4f ICPRegistration::align()
{
    result_t = initial_transformation;
    calculate();
    return result_t;
}

Eigen::Matrix4f ICPRegistration::getTransformation() const
{
    return result_t;
}

float ICPRegistration::getFitnessScore() const
{
    return fitness_score;
}

//----------------------------------------------------

void ICPRegistration::calculate()
{
    PcdPtr& input_point_cloud_ptr = keypoints_frame.keypointsPcdPair.second;
    PcdPtr& target_point_cloud_ptr = keypoints_frame.keypointsPcdPair.first;
    pcl::Correspondences& correspondeces = keypoints_frame.keypointsPcdCorrespondences;

    const int& maxIter = configs.value("ICP_SETTINGS/MAX_ITERATIONS").toInt();
    const double& trans_epsilon = configs.value("ICP_SETTINGS/TRANSFORMATION_EPSILON").toDouble();
    const double& euclid_epsilon = configs.value("ICP_SETTINGS/EUCLIDEAN_EPSILON").toDouble();

    const int& k_size = input_point_cloud_ptr->size();

    if (configs.value("ICP_SETTINGS/POINT_TO_PLANE").toBool() && k_size > 20) {
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        gicp.setInputSource(input_point_cloud_ptr);
        gicp.setInputTarget(target_point_cloud_ptr);
        gicp.setMaximumIterations(maxIter);
        gicp.setTransformationEpsilon(trans_epsilon);
        gicp.setEuclideanFitnessEpsilon(euclid_epsilon);
        Pcd transformed_cloud;
        gicp.align(transformed_cloud);

        if (gicp.hasConverged()) {
            //ICP aligment
            pcl::transformPointCloud(*target_point_cloud_ptr, *target_point_cloud_ptr, initial_transformation);
            result_t = gicp.getFinalTransformation() * initial_transformation;
            pcl::transformPointCloud(*input_point_cloud_ptr, *input_point_cloud_ptr, result_t);
            fitness_score = gicp.getFitnessScore();
        } else {
            qDebug() << "PCL GICP has not converge.";
        }
    } else {
        pcl::IterativeClosestPointNonLinear<PointType, PointType> icp;
        icp.setInputSource(input_point_cloud_ptr);
        icp.setInputTarget(target_point_cloud_ptr);
        icp.setMaximumIterations(maxIter);
        icp.setTransformationEpsilon(trans_epsilon);
        icp.setEuclideanFitnessEpsilon(euclid_epsilon);
        Pcd transformed_cloud;
        icp.align(transformed_cloud);

        if (icp.hasConverged()) {
            //ICP aligment
            pcl::transformPointCloud(*target_point_cloud_ptr, *target_point_cloud_ptr, initial_transformation);
            result_t = icp.getFinalTransformation() * initial_transformation;
            pcl::transformPointCloud(*input_point_cloud_ptr, *input_point_cloud_ptr, result_t);
            fitness_score = icp.getFitnessScore();
        } else {
            qDebug() << "ICP did not converge.";
        }
    }
}
