#include "core/reconstruction/reconstructioninterface.h"

#include <pcl/common/distances.h>

#include "core/registration/edgebasedregistration.hpp"
#include "core/registration/icpregistration.h"
#include "core/registration/linearbasedregistration.hpp"
#include "core/registration/linearregistration.hpp"
#include "core/registration/middlebasedregistration.hpp"
#include "core/registration/parallelregistration.hpp"
#include "core/registration/sacregistration.h"
#include "io/pcdinputiterator.hpp"

//#######################################################

ReconstructionInterface::ReconstructionInterface(QObject* parent, QSettings* parent_settings)
    : ScannerBase(parent, parent_settings)
    , volumeReconstruction(new VolumeReconstruction(this, settings))
    , pcdVizualizer(new PcdVizualizer(this, settings))
{
}

void ReconstructionInterface::reloadSettings()
{
    volumeReconstruction->deleteLater();

    volumeReconstruction = VolumeReconstruction::Ptr(new VolumeReconstruction(this, settings));
    pcdVizualizer->redraw();
}

//#######################################################
//#------------------RECONSTRUCTION----------------------

void ReconstructionInterface::perform_tsdf_integration(
    Frames& frames,
    Matrix4fVector& final_translation_matrix_vector)
{
    PcdPtrVector point_cloud_vector;
    std::transform(frames.begin(), frames.end(), std::back_inserter(point_cloud_vector),
        [](const Frame& frame) { return frame.pointCloudPtr; });

    volumeReconstruction->addPointCloudVector(point_cloud_vector, final_translation_matrix_vector);
}

void ReconstructionInterface::perform_tsdf_meshing(
    Matrix4fVector& final_translation_matrix_vector)
{
    volumeReconstruction->prepareVolume();
    pcdVizualizer->redraw();

    if (settings->value("VISUALIZATION/DRAW_ALL_CAMERA_POSES").toBool()) {
        pcdVizualizer->visualizeCameraPoses(final_translation_matrix_vector);
    }

    if (settings->value("VISUALIZATION/CPU_TSDF_DRAW_MESH").toBool()) {
        pcl::PolygonMesh mesh;
        volumeReconstruction->calculateMesh();
        volumeReconstruction->getPoligonMesh(mesh);
        pcdVizualizer->visualizeMesh(mesh);
    }
}

void ReconstructionInterface::perform_reconstruction()
{
    const bool is_fixed_size = settings->value("ALGORITHM_SETTINGS/LINEAR_RECONSTRUCTION_FIXED_SIZE").toBool();
    const int fixed_size = settings->value("ALGORITHM_SETTINGS/LINEAR_RECONSTRUCTION_SIZE").toInt();

    int from = is_fixed_size ? 0 : settings->value("READING_SETTING/FROM").toInt();
    int to = is_fixed_size ? std::numeric_limits<int>::max() : settings->value("READING_SETTING/TO").toInt();

    PcdInputIterator range_it(settings, from, to, 1);
    from = range_it.getLowerBound();
    to = range_it.getUpperBound();

    const int dinamyc_step = (to - from) / fixed_size == 0 ? 1 : (to - from) / fixed_size;
    const int step = (is_fixed_size ? dinamyc_step : settings->value("READING_SETTING/STEP").toInt());

    range_it = PcdInputIterator(settings, from, to, step);
    from = range_it.getLowerBound();
    to = range_it.getUpperBound();

    Matrix4fVector final_transformations;
    std::vector<double> fit;
    std::vector<double> mean;

    PcdInputIterator it(settings, from, to, step);
    PcdInputIterator it2(settings, from + step, to, step);
    for (; it != PcdInputIterator() && it2 != PcdInputIterator(); ++it, ++it2) {
        Frames frames, transformed_frames;

        frames.push_back(*it);
        frames.push_back(*it2);

        PcdFilters filters(this, settings);
        filters.setInput(frames);
        filters.filter(frames);
        ;

        LinearRegistration<SaCRegistration> linear_sac(this, settings);
        linear_sac.setInput(frames, Eigen::Matrix4f(Eigen::Matrix4f::Identity()));
        Matrix4fVector sac_transformations = linear_sac.align(transformed_frames);

        LinearRegistration<ICPRegistration> linear_icp(this, settings);
        linear_icp.setInput(transformed_frames, Eigen::Matrix4f(Eigen::Matrix4f::Identity()));
        linear_icp.setKeypoints(linear_sac.getTransformedKeypoints());
        Matrix4fVector icp_transformations = linear_icp.align(transformed_frames);

        for (uint i = 1; i < icp_transformations.size(); ++i) {
            final_transformations.push_back(icp_transformations[i] * sac_transformations[i]);
        }

        const KeypointsFrames& kp = linear_icp.getTransformedKeypoints();
        for (uint i = 0; i < kp.size(); ++i) {
            std::vector<double> dis;
            for (uint j = 0; j < kp[i].keypointsPcdPair.first->size(); ++j) {
                const PointType& a = (*kp[i].keypointsPcdPair.first)[j];
                const PointType& b = (*kp[i].keypointsPcdPair.second)[j];
                dis.push_back(pcl::euclideanDistance(a, b));
            }

            mean.push_back(
                std::accumulate(dis.begin(), dis.end(), 0.0f,
                    [](const double& sum, const double& n) { return sum + n; })
                / dis.size());
        }

        const auto f = linear_icp.getFitnessScores();
        std::copy(f.begin(), f.end(), std::back_inserter(fit));
    }

    pcdVizualizer->visualizeCameraPoses(final_transformations);
    pcdVizualizer->plotCameraDistances(fit, false, "FS", "Score");
    pcdVizualizer->plotCameraDistances(mean, false, "Mean", "M Dist");

    Frames frames(PcdInputIterator(settings, from, to, step), PcdInputIterator());
    Frames transformed_frames;

    LinearRegistration<SaCRegistration> linear_sac(this, settings);
    linear_sac.setInput(frames, Eigen::Matrix4f(Eigen::Matrix4f::Identity()));
    Matrix4fVector sac_transformations = linear_sac.align(transformed_frames);

    LinearRegistration<ICPRegistration> linear_icp(this, settings);
    linear_icp.setInput(transformed_frames, Eigen::Matrix4f(Eigen::Matrix4f::Identity()));
    linear_icp.setKeypoints(linear_sac.getTransformedKeypoints());
    Matrix4fVector icp_transformations = linear_icp.align(transformed_frames);

    final_transformations.clear();
    for (uint i = 1; i < icp_transformations.size(); ++i) {
        final_transformations.push_back(icp_transformations[i] * sac_transformations[i]);
    }

    if (settings->value("VISUALIZATION/CPU_TSDF").toBool()) {
        perform_tsdf_integration(frames, final_transformations);
        perform_tsdf_meshing(final_transformations);
    } else {
        pcdVizualizer->redraw();
        if (settings->value("VISUALIZATION/DRAW_ALL_CAMERA_POSES").toBool()) {
            pcdVizualizer->visualizeCameraPoses(final_transformations);
        }
        if (settings->value("VISUALIZATION/DRAW_ALL_CLOUDS").toBool()) {
            pcdVizualizer->visualizePointClouds(transformed_frames);
        }
        if (settings->value("VISUALIZATION/DRAW_ALL_KEYPOINT_CLOUDS").toBool()) {
            pcdVizualizer->visualizeKeypointClouds(linear_icp.getTransformedKeypoints());
        }

        while (!pcdVizualizer->viewer->wasStopped()) {
            pcdVizualizer->spin();
        }
    }
}

void ReconstructionInterface::perform_iterative_reconstruction()
{
    LinearBasedRegistration lbr(this, settings);
    lbr.setVolumeReconstructor(volumeReconstruction);
    lbr.setVisualizer(pcdVizualizer);
    lbr.reconstruct();
}

void ReconstructionInterface::perform_partition_recursive_reconstruction()
{
    MiddleBasedRegistration mbr(this, settings);
    mbr.setVolumeReconstructor(volumeReconstruction);
    mbr.setVisualizer(pcdVizualizer);
    mbr.reconstruct();
}

void ReconstructionInterface::perform_lum_reconstruction()
{
    EdgeBasedRegistration ebr(this, settings);
    ebr.setVolumeReconstructor(volumeReconstruction);
    ebr.setVisualizer(pcdVizualizer);
    ebr.reconstruct();
}

//#######################################################
//#-----------------------SLOTS--------------------------

void ReconstructionInterface::slot_perform_reconstruction()
{
    if (settings->value("ALGORITHM_SETTINGS/LINEAR_RECONSTRUCTION").toBool()) {
        perform_reconstruction();
    } else if (settings->value("ALGORITHM_SETTINGS/LINEAR_RECONSTRUCTION_WITH_LOOPS").toBool()) {
        perform_iterative_reconstruction();
    } else if (settings->value("ALGORITHM_SETTINGS/MIDDLE_BASED_RECONSTRUCTION").toBool()) {
        perform_partition_recursive_reconstruction();
    } else if (settings->value("ALGORITHM_SETTINGS/EDGE_BASED_RECONSTRUCTION_ENABLE").toBool()) {
        perform_lum_reconstruction();
    }
}
