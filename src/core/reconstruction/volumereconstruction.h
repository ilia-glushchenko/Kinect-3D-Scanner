#ifndef VOLUMERECONSTRUCTION_H
#define VOLUMERECONSTRUCTION_H

#include <QDebug>
#include <QObject>
#include <QSettings>
#include <QString>

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>

#include "core/base/scannertypes.h"
#include "io/pclio.h"

//###############################################################
// USAGE:
// constructor();
// ***
// addPointCloudVector();
// addPointCloud();
// ***
// prepareVolume();
// calculateMesh();
// getPoligonMesh();

class VolumeReconstruction : public ScannerBase {
    Q_OBJECT

public:
    typedef boost::shared_ptr<VolumeReconstruction> Ptr;

    VolumeReconstruction(QObject* parent, QSettings* parent_settings);

    void addPointCloudVector(
        const PcdPtrVector& point_cloud_vector,
        const Matrix4fVector& translation_matrix_vector);

    void addPointCloud(
        const PcdPtr& point_cloud,
        const Eigen::Matrix4f& translation_matrix);

    void prepareVolume();

    void calculateMesh();

    void getPoligonMesh(pcl::PolygonMesh& mesh);

private:
    boost::shared_ptr<cpu_tsdf::TSDFVolumeOctree> tsdf;
    pcl::PolygonMesh _mesh;
};

#endif // VOLUMERECONSTRUCTION_H
