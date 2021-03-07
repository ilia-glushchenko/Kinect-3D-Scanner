#include "gui/pcdvizualizer.h"

#include <pcl/common/distances.h>
#include <pcl/visualization/pcl_plotter.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

PcdVizualizer::PcdVizualizer(QObject* parent, QSettings* parent_settings)
    : ScannerBase(parent, parent_settings)
    , viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
{
    viewer->setBackgroundColor(
        configs.value("VISUALIZATOR_SETTINGS/BG_R").toDouble() / 255.0f,
        configs.value("VISUALIZATOR_SETTINGS/BG_G").toDouble() / 255.0f,
        configs.value("VISUALIZATOR_SETTINGS/BG_B").toDouble() / 255.0f);
    visualize_debug_text();

    if (configs.value("VISUALIZATOR_SETTINGS/DRAW_AXIS").toBool()) {
        viewer->addCoordinateSystem(configs.value("VISUALIZATOR_SETTINGS/AXIS_SIZE").toFloat());
    }
    viewer->initCameraParameters();
    viewer->spinOnce(100);
}

void PcdVizualizer::redraw()
{
    viewer->removeAllShapes();
    viewer->removeAllPointClouds();
    visualize_debug_text();

    //Draw volume cude
    if (configs.value("CPU_TSDF_SETTINGS/DRAW_VOLUME_CUBE").toBool()) {
        const double x_vol = configs.value("CPU_TSDF_SETTINGS/X_VOL").toDouble();
        const double y_vol = configs.value("CPU_TSDF_SETTINGS/Y_VOL").toDouble();
        const double z_vol = configs.value("CPU_TSDF_SETTINGS/Z_VOL").toDouble();
        const double x_shift = configs.value("CPU_TSDF_SETTINGS/X_SHIFT").toDouble();
        const double y_shift = configs.value("CPU_TSDF_SETTINGS/Y_SHIFT").toDouble();
        const double z_shift = configs.value("CPU_TSDF_SETTINGS/Z_SHIFT").toDouble();

        viewer->addCube(-(x_vol / 2.0f) + x_shift, (x_vol / 2.0f) + x_shift,
            -(y_vol / 2.0f) + y_shift, (y_vol / 2.0f) + y_shift,
            -(z_vol / 2.0f) + z_shift, (z_vol / 2.0f) + z_shift);
    }
}

void PcdVizualizer::visualizePointClouds(const Frames& frames)
{
    const int rand_num = rand();
    for (int i = 0; i < frames.size(); i++) {
        pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(frames[i].pointCloudPtr);
        viewer->addPointCloud<PointType>(
            frames[i].pointCloudPtr, rgb, QString("Cloud #%1").arg(i + rand_num).toStdString());
    }
}

void PcdVizualizer::visualizeKeypointClouds(const KeypointsFrames& keypointsFrames)
{
    if (keypointsFrames.empty() == false) {
        const int rand_num = rand();

        for (int i = 0; i < keypointsFrames.size(); i++) {
            if (!keypointsFrames[i].keypointsPcdCorrespondences.empty())
                viewer->addCorrespondences<PointType>(
                    keypointsFrames[i].keypointsPcdPair.first,
                    keypointsFrames[i].keypointsPcdPair.second,
                    keypointsFrames[i].keypointsPcdCorrespondences,
                    QString("Cor %1").arg(i + rand_num).toStdString());
        }

        if (configs.value("VISUALIZATOR_SETTINGS/DRAW_FIRST_LAST_KP_CLOUDS").toBool()) {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color2(
                keypointsFrames[0].keypointsPcdPair.first, 0, 0, 0);
            viewer->addPointCloud<PointType>(
                keypointsFrames[0].keypointsPcdPair.first,
                single_color2, QString("Cloud P1 %1").arg(0 + rand_num).toStdString());
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,
                QString("Cloud P1 %1").arg(0 + rand_num).toStdString());

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color3(
                keypointsFrames[keypointsFrames.size() - 1].keypointsPcdPair.second, 0, 255, 0);
            viewer->addPointCloud<PointType>(
                keypointsFrames[keypointsFrames.size() - 1].keypointsPcdPair.second,
                single_color3,
                QString("Cloud P2 %1").arg(keypointsFrames.size() - 1 + rand_num).toStdString());
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,
                QString("Cloud P2 %1").arg(keypointsFrames.size() - 1 + rand_num).toStdString());
        }

        for (int i = 1; i < keypointsFrames.size(); i++) {
            int r1, g1, b1;
            if (i % 2 == 0) {
                r1 = 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
                g1 = 0;
                b1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
            } else {
                r1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
                g1 = 0;
                b1 = 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
            }

            if (configs.value("VISUALIZATOR_SETTINGS/DRAW_PARE_SECOND_KP_CLOUD").toBool()) {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color1(
                    keypointsFrames[i - 1].keypointsPcdPair.second, r1, g1, b1);
                viewer->addPointCloud<PointType>(
                    keypointsFrames[i - 1].keypointsPcdPair.second,
                    single_color1,
                    QString("Cloud P2 %1").arg(i - 1 + rand_num).toStdString());
                viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,
                    QString("Cloud P2 %1").arg(i - 1 + rand_num).toStdString());
            }

            if (configs.value("VISUALIZATOR_SETTINGS/DRAW_PARE_FIRST_KP_CLOUD").toBool()) {
                if (configs.value("VISUALIZATOR_SETTINGS/DRAW_DIFFERENT_COLOR_PARES").toBool()) {
                    if (i % 2 == 0) {
                        r1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
                        b1 = 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
                    } else {
                        r1 = 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
                        b1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
                    }
                }

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(
                    keypointsFrames[i].keypointsPcdPair.first, r1, g1, b1);
                viewer->addPointCloud<PointType>(
                    keypointsFrames[i].keypointsPcdPair.first,
                    single_color,
                    QString("Cloud P1 %1").arg(i + rand_num).toStdString());
                viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,
                    QString("Cloud P1 %1").arg(i + rand_num).toStdString());
            }
        }
    }
}

void PcdVizualizer::visualizeCameraPoses(
    const Matrix4fVector& final_translation_matrix_vector)
{
    std::vector<double> data(1, 0);

    const float radius = settings->value("VISUALIZATION/CAMERA_SPHERE_RADIUS").toFloat();
    viewer->addSphere(PointType(), radius, QString("%1").arg(0 + std::rand()).toStdString());
    for (int i = 1; i < final_translation_matrix_vector.size(); i++) {
        PointType pA;
        pA.x = final_translation_matrix_vector[i - 1](0, 3);
        pA.y = final_translation_matrix_vector[i - 1](1, 3);
        pA.z = final_translation_matrix_vector[i - 1](2, 3);

        PointType pC;
        pC.x = final_translation_matrix_vector[i](0, 3);
        pC.y = final_translation_matrix_vector[i](1, 3);
        pC.z = final_translation_matrix_vector[i](2, 3);

        data.push_back(pcl::euclideanDistance(pcl::PointXYZ(pA.x, pA.y, pA.z), pcl::PointXYZ(pC.x, pC.y, pC.z)));
        viewer->addSphere(pC, radius, QString("%1").arg(i + std::rand()).toStdString());
    }

    plotNormalDistribution(data, "Camera distances Normal Distribution");
    plotCameraDistances(data, true, "Distances (meters)", "Distances between neighbour cameras");
}

void PcdVizualizer::visualizeMesh(const pcl::PolygonMesh& mesh)
{
    viewer->addPolygonMesh(mesh);
}

void PcdVizualizer::plotNormalDistribution(const std::vector<double>& input_data, const char* title) const
{
    const float& scale_factor = std::pow(10, 2);

    std::vector<int> data;
    std::transform(input_data.begin(), input_data.end(), std::back_inserter(data),
        [&](const double& x) { return std::round(x * scale_factor); });

    std::map<int, float> P;
    std::for_each(data.begin(), data.end(), [&](const int& x) {
        P[x] = P.find(x) != P.end() ? P[x] + 1 : 1;
    });
    std::for_each(P.begin(), P.end(), [&](std::pair<const int, float>& x) {
        x.second /= data.size();
    });

    const float average = std::accumulate(data.begin(), data.end(), 0.0f,
                              [](const float& sum, const float& x) { return sum + x; })
        / data.size();
    const float sigma = std::sqrt(1.0f / data.size() * std::accumulate(data.begin(), data.end(), 0.0f, [&](const float& sum, const float& x) {
                                      return sum + std::pow(x - average, 2.0f);
                                  }));

    const float M = std::accumulate(data.begin(), data.end(), 0.0f, [&](const float& sum, const float& x) {
        return sum + x * P[x];
    });

    const float s = std::sqrt(float(data.size()) / (float(data.size()) - 1.0f) * sigma * sigma);

    std::vector<double> nd;
    std::vector<double> xes;
    for (float x = average - 6 * s; x <= (average + 6 * s); x += 0.2f) {
        nd.push_back(gaussian_pdf(x, average, sigma));
        xes.push_back(x / scale_factor);
    }

    pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter(title);
    plotter->addPlotData(xes.data(), nd.data(), nd.size());
    plotter->addPlotData(xes.data(), nd.data(), nd.size(), "Probability", vtkChart::BAR);
    plotter->setYRange(0, 1);
    plotter->setXTitle("X");
    plotter->setYTitle("Probability");
    plotter->setTitle("Standard deviation");
    plotter->spinOnce();
}

void PcdVizualizer::plotCameraDistances(
    const std::vector<double>& data,
    const bool& fixed_range,
    const char* name,
    const char* value) const
{
    if (name == nullptr || value == nullptr) {
        throw std::invalid_argument("PcdVizualizer::plotCameraDistances name == nullptr || value == nullptr");
    }

    std::vector<double> xes(data.size());
    std::iota(xes.begin(), xes.end(), 0);

    pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter(name);
    plotter->addPlotData(xes.data(), data.data(), data.size());
    plotter->addPlotData(xes.data(), data.data(), data.size(), value, vtkChart::POINTS);
    if (fixed_range) {
        plotter->setYRange(0, 0.1f);
    }
    plotter->setTitle(name);
    plotter->setXTitle("#");
    plotter->setYTitle(value);
    plotter->spinOnce();
}

void PcdVizualizer::spin(const uint& spin_time)
{
    if (!viewer) {
        throw std::runtime_error("PcdVizualizer::spin !viewer");
    }

    viewer->spinOnce(spin_time);
}

//---------------------------------------------------------------

float PcdVizualizer::gaussian_pdf(const float& x, const float& u, const float& sigma) const
{
    const float& power = (x - u) * (x - u) / (2.0f * sigma * sigma);
    const float& e = std::pow(M_E, -power);

    return (1.0f / (sigma * std::sqrt(2.0f * M_PI))) * e;
}

void PcdVizualizer::visualize_debug_text()
{
    if (configs.value("VISUALIZATOR_SETTINGS/DEBUG_INI_ENABLE").toBool()) {
        QString allGroupsString;
        foreach (const QString& group, settings->childGroups()) {
            if (settings->value(QString("%1/ENABLE_IN_VISUALIZATION").arg(group)).toBool()) {
                QString groupString = QString("%1 \n").arg(group);
                settings->beginGroup(group);

                foreach (const QString& key, settings->childKeys()) {
                    if (key != "ENABLE_IN_VISUALIZATION") {
                        groupString.append(QString("%1: %2; ").arg(key, settings->value(key).toString()));
                    }
                }

                settings->endGroup();
                groupString.append("\n\n");

                allGroupsString.append(groupString);
            }
        }
        viewer->addText(
            allGroupsString.toStdString(),
            configs.value("VISUALIZATOR_SETTINGS/DEBUG_INI_X").toInt(),
            configs.value("VISUALIZATOR_SETTINGS/DEBUG_INI_Y").toInt(),
            configs.value("VISUALIZATOR_SETTINGS/DEBUG_INI_FONTSIZE").toInt(),
            configs.value("VISUALIZATOR_SETTINGS/DEBUG_INI_R").toDouble() / 255.0f,
            configs.value("VISUALIZATOR_SETTINGS/DEBUG_INI_G").toDouble() / 255.0f,
            configs.value("VISUALIZATOR_SETTINGS/DEBUG_INI_B").toDouble() / 255.0f);
    }
}

void PcdVizualizer::set_viewer_pose(
    pcl::visualization::PCLVisualizer& viewer,
    const Eigen::Affine3f& viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition(
        pos_vector[0], pos_vector[1], pos_vector[2],
        look_at_vector[0], look_at_vector[1], look_at_vector[2],
        up_vector[0], up_vector[1], up_vector[2]);
}
