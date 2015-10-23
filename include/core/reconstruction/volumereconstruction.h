#ifndef VOLUMERECONSTRUCTION_H
#define VOLUMERECONSTRUCTION_H

#include <QObject>
#include <QSettings>
#include <QString>
#include <QDebug>

#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/octree.h>
#include <cpu_tsdf/tsdf_volume_octree.h>

#include "io/pclio.h"
#include "core/base/types.h"

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

class VolumeReconstruction : public ScannerBase
{
	Q_OBJECT

public:
	VolumeReconstruction(QObject *parent, QSettings* parent_settings);
	void addPointCloudVector(
		PcdPtrVector& point_clouds_vector,
		Matrix4fVector& translation_matrix_vector
	);
	void addPointCloud(
		PcdPtr point_cloud,
		Eigen::Matrix4f& translation_matrix
	);
	void prepareVolume();
	void calculateMesh();
	void getPoligonMesh(pcl::PolygonMesh& mesh);

private:
	boost::shared_ptr<cpu_tsdf::TSDFVolumeOctree> tsdf;
	pcl::PolygonMesh _mesh;

	void prepare_volume();
	void calculate_mesh();
	void add_point_cloud_vector(
		PcdPtrVector& point_cloud_vector,
		Matrix4fVector& translation_matrix_vector
	);
	void add_point_cloud(
		PcdPtr point_cloud,
		Eigen::Matrix4f& translation_matrix
	);
};

#endif // VOLUMERECONSTRUCTION_H
