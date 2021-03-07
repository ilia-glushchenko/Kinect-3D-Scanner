#ifndef PCDVIZUALIZER_H
#define PCDVIZUALIZER_H

#include <QDebug>
#include <QObject>
#include <QSettings>
#include <QString>

#include <cstdlib>

#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "core/base/scannertypes.h"

class PcdVizualizer : public ScannerBase {
    Q_OBJECT

public:
    typedef boost::shared_ptr<PcdVizualizer> Ptr;

    pcl::visualization::PCLVisualizer::Ptr viewer;

    PcdVizualizer(QObject* parent, QSettings* parent_settings);

    void redraw();

    void visualizePointClouds(const Frames& frames);

    void visualizeKeypointClouds(const KeypointsFrames& keypointsFrames);

    void visualizeCameraPoses(const Matrix4fVector& final_translation_matrix_vector);

    void visualizeMesh(const pcl::PolygonMesh& mesh);

    void plotNormalDistribution(const std::vector<double>& input_data, const char* title = "Normal Distribution") const;

    void plotCameraDistances(
        const std::vector<double>& data,
        const bool& fixed_range,
        const char* name,
        const char* value) const;

    void spin(const uint& spin_time = 1);

private:
    float gaussian_pdf(const float& x, const float& u, const float& sigma) const;

    void set_viewer_pose(
        pcl::visualization::PCLVisualizer& viewer,
        const Eigen::Affine3f& viewer_pose);

    void visualize_debug_text();
};

#endif // PCDVIZUALIZER_H
