#include "core/registration/lumcorrection.h"

#include <QDebug>
#include <pcl/common/distances.h>
#include <pcl/registration/lum.h>

LumCorrection::LumCorrection(QObject* parent, QSettings* parent_settings)
    : ScannerBase(parent, parent_settings)
{
}

void LumCorrection::setInput(
    const PcdPtrVector& merged_keypoints_,
    const CorrespondencesVector& merged_correspondences_)
{
    merged_correspondences = merged_correspondences_;
    merged_keypoints = merged_keypoints_;
}

Matrix4fVector LumCorrection::correct()
{
    calculate_lum_correction();
    return result_t;
}

Matrix4fVector LumCorrection::getTransformations() const
{
    return result_t;
}

void LumCorrection::calculate_lum_correction()
{
    qDebug() << "Calculating LUM";

    //Converting PCD XYZRGB to XYZ
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcd_xyz_vector;
    for (size_t i = 0; i < merged_keypoints.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*merged_keypoints[i], *pcd_xyz);
        pcd_xyz_vector.push_back(pcd_xyz);
    }

    pcl::registration::LUM<pcl::PointXYZ> lum;
    for (uint i = 0; i < pcd_xyz_vector.size(); ++i) {
        lum.addPointCloud(pcd_xyz_vector[i]);
    }
    for (uint i = 0; i < pcd_xyz_vector.size(); ++i) {
        const uint j = i < pcd_xyz_vector.size() - 1 ? i + 1 : 0;
        lum.setCorrespondences(
            i, j, CorrespondencesPtr(new pcl::Correspondences(merged_correspondences[i])));
    }
    lum.setMaxIterations(50);
    lum.setConvergenceThreshold(0.0);
    lum.compute();

    for (int i = 0; i < lum.getNumVertices(); i++) {
        result_t.push_back(lum.getTransformation(i).matrix());
    }

    qDebug() << "Done!";
}
