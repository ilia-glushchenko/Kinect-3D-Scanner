#include "gui/pcdvizualizer.h"

PcdVizualizer::PcdVizualizer(QObject *parent, QSettings* parent_settings)
	: ScannerBase(parent, parent_settings)
{
	viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer.get()->setBackgroundColor(
		settings->value("VISUALIZATION/BG_R").toDouble() / 255.0f,
		settings->value("VISUALIZATION/BG_G").toDouble() / 255.0f,
		settings->value("VISUALIZATION/BG_B").toDouble() / 255.0f
		);

	visualize_debug_text();

	if (settings->value("VISUALIZATION/DRAW_AXIS").toBool())
		viewer.get()->addCoordinateSystem(settings->value("VISUALIZATION/AXIS_SIZE").toFloat());
	viewer.get()->initCameraParameters();
	viewer.get()->spinOnce(100);
}

void PcdVizualizer::visualizePointClouds(
	Frames& frames
	)
{
	int rand_num = rand();

	for (int i = 0; i < frames.size(); i++)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(frames[i].pointCloudPtr);
		viewer.get()->addPointCloud<PointType>(
			frames[i].pointCloudPtr,
			rgb,
			QString("Cloud #%1").arg(i + rand_num).toStdString()
		);
	}

}

void PcdVizualizer::visualizeKeypointClouds(
	KeypointsFrames& keypointsFrames
	)
{
	if (keypointsFrames.empty() == false)
	{
		int rand_num = rand();

		for (int i = 0; i < keypointsFrames.size(); i++)
		{
			if (!keypointsFrames[i].keypointsPcdCorrespondences.empty())
				viewer.get()->addCorrespondences<PointType>(
					keypointsFrames[i].keypointsPcdPair.first,
					keypointsFrames[i].keypointsPcdPair.second,
					keypointsFrames[i].keypointsPcdCorrespondences,
					QString("Cor %1").arg(i + rand_num).toStdString()
				);
		}

		if (settings->value("VISUALIZATION/DRAW_FIRST_LAST_KP_CLOUDS").toBool())
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color2(
				keypointsFrames[0].keypointsPcdPair.first, 0, 0, 0
			);
			viewer.get()->addPointCloud<PointType>(
				keypointsFrames[0].keypointsPcdPair.first,
				single_color2, QString("Cloud P1 %1").arg(0 + rand_num).toStdString()
			);
			viewer.get()->setPointCloudRenderingProperties(
				pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, 
				QString("Cloud P1 %1").arg(0 + rand_num).toStdString()
			);

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color3(
				keypointsFrames[keypointsFrames.size() - 1].keypointsPcdPair.second, 0, 255, 0
			);
			viewer.get()->addPointCloud<PointType>(
				keypointsFrames[keypointsFrames.size() - 1].keypointsPcdPair.second,
				single_color3, 
				QString("Cloud P2 %1").arg(keypointsFrames.size() - 1 + rand_num).toStdString()
				);
			viewer.get()->setPointCloudRenderingProperties(
				pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, 
				QString("Cloud P2 %1").arg(keypointsFrames.size() - 1 + rand_num).toStdString()
			);
		}

		for (int i = 1; i < keypointsFrames.size(); i++)
		{
			int r1, g1, b1;
			if (i % 2 == 0)
			{
				r1 = 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
				g1 = 0;
				b1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
			}
			else
			{
				r1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
				g1 = 0;
				b1 = 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
			}

			if (settings->value("VISUALIZATION/DRAW_PARE_SECOND_KP_CLOUD").toBool())
			{
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color1(
					keypointsFrames[i - 1].keypointsPcdPair.second, r1, g1, b1
				);
				viewer.get()->addPointCloud<PointType>(
					keypointsFrames[i - 1].keypointsPcdPair.second, 
					single_color1, 
					QString("Cloud P2 %1").arg(i - 1 + rand_num).toStdString()
				);
				viewer.get()->setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, 
					QString("Cloud P2 %1").arg(i - 1 + rand_num).toStdString()
				);
			}

			if (settings->value("VISUALIZATION/DRAW_PARE_FIRST_KP_CLOUD").toBool())
			{
				if (settings->value("VISUALIZATION/DRAW_DIFFERENT_COLOR_PARES").toBool())
				{
					if (i % 2 == 0)
					{
						r1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
						b1 = 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
					}
					else
					{
						r1 = 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
						b1 = 255.0f - 255.0f * ((float)(i + 1) / (float)keypointsFrames.size());
					}
				}

				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(
					keypointsFrames[i].keypointsPcdPair.first, r1, g1, b1
				);
				viewer.get()->addPointCloud<PointType>(
					keypointsFrames[i].keypointsPcdPair.first,
					single_color, 
					QString("Cloud P1 %1").arg(i + rand_num).toStdString()
				);
				viewer.get()->setPointCloudRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, 
					QString("Cloud P1 %1").arg(i + rand_num).toStdString()
				);
			}
		}
	}
}

void PcdVizualizer::visualizeCameraPoses(
	Matrix4fVector& final_translation_matrix_vector
	)
{
	float radius = settings->value("FINAL_SETTINGS/CAMERA_SPHERE_RADIUS").toFloat();
	for (int i = 0; i < final_translation_matrix_vector.size(); i++)
	{
		PointType pC;
		pC.x = final_translation_matrix_vector[i](0, 3);
		pC.y = final_translation_matrix_vector[i](1, 3);
		pC.z = final_translation_matrix_vector[i](2, 3);

		viewer.get()->addSphere(pC, radius, QString("%1").arg(i+std::rand()).toStdString());
	}
}

void PcdVizualizer::visualizeMesh(
	pcl::PolygonMesh& mesh
	)
{
	viewer.get()->addPolygonMesh(mesh);
}

void PcdVizualizer::redraw()
{
	viewer.get()->removeAllShapes();
	viewer.get()->removeAllPointClouds();
	visualize_debug_text();

	//Draw volume cude
	if (settings->value("CPU_TSDF_SETTINGS/DRAW_VOLUME_CUBE").toBool())
	{
		double x_vol = settings->value("CPU_TSDF_SETTINGS/X_VOL").toDouble();
		double y_vol = settings->value("CPU_TSDF_SETTINGS/Y_VOL").toDouble();
		double z_vol = settings->value("CPU_TSDF_SETTINGS/Z_VOL").toDouble();
		double x_shift = settings->value("CPU_TSDF_SETTINGS/X_SHIFT").toDouble();
		double y_shift = settings->value("CPU_TSDF_SETTINGS/Y_SHIFT").toDouble();
		double z_shift = settings->value("CPU_TSDF_SETTINGS/Z_SHIFT").toDouble();
		viewer.get()->addCube(
			-(x_vol / 2.0f) + x_shift, (x_vol / 2.0f) + x_shift,
			-(y_vol / 2.0f) + y_shift, (y_vol / 2.0f) + y_shift,
			-(z_vol / 2.0f) + z_shift, (z_vol / 2.0f) + z_shift
			);
	}
}

//---------------------------------------------------------------

void PcdVizualizer::visualize_debug_text()
{
	if (settings->value("VISUALIZATION/DEBUG_INI_ENABLE").toBool())
	{
		QString allGroupsString;
		foreach(const QString &group, settings->childGroups()) {
			if (settings->value(QString("%1/ENABLE_IN_VISUALIZATION").arg(group)).toBool())
			{
				QString groupString = QString("%1 \n").arg(group);
				settings->beginGroup(group);

				foreach(const QString &key, settings->childKeys()) {
					if (key != "ENABLE_IN_VISUALIZATION")
						groupString.append(QString("%1: %2; ").arg(key, settings->value(key).toString()));
				}

				settings->endGroup();
				groupString.append("\n\n");

				allGroupsString.append(groupString);
			}
		}
		viewer.get()->addText(
			allGroupsString.toStdString(),
			settings->value("VISUALIZATION/DEBUG_INI_X").toInt(),
			settings->value("VISUALIZATION/DEBUG_INI_Y").toInt(),
			settings->value("VISUALIZATION/DEBUG_INI_FONTSIZE").toInt(),
			settings->value("VISUALIZATION/DEBUG_INI_R").toDouble() / 255.0f,
			settings->value("VISUALIZATION/DEBUG_INI_G").toDouble() / 255.0f,
			settings->value("VISUALIZATION/DEBUG_INI_B").toDouble() / 255.0f
		);
	}

}

void PcdVizualizer::set_viewer_pose(
	pcl::visualization::PCLVisualizer& viewer,
	const Eigen::Affine3f& viewer_pose
	)
{
	Eigen::Vector3f pos_vector	   = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector      = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(
		pos_vector[0],	   pos_vector[1],	  pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0],	   up_vector[1],	  up_vector[2]
	);
}
