#ifndef PCDVIZUALIZER_H
#define PCDVIZUALIZER_H

#include <QObject>
#include <QSettings>
#include <QString>
#include <QDebug>

#include <cstdlib>

#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "core/base/types.h"

class PcdVizualizer : public ScannerBase
{
	Q_OBJECT

public:
	PcdVizualizer(QObject *parent, QSettings* parent_settings);
	pcl::visualization::PCLVisualizer::Ptr viewer;

	void redraw();
	void visualizePointClouds(
		Frames& frames
	);
	void visualizeKeypointClouds(
		KeypointsFrames& keypointsFrames
	);
	void visualizeCameraPoses(
		Matrix4fVector& final_translation_matrix_vector
	);
	void visualizeMesh(
		pcl::PolygonMesh& mesh
	);
	
private:
	void set_viewer_pose(
		pcl::visualization::PCLVisualizer& viewer,
		const Eigen::Affine3f& viewer_pose
	);
	void visualize_debug_text();
		
};

#endif // PCDVIZUALIZER_H
