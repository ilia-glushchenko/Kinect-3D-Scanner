#include "core/reconstruction/volumereconstruction.h"

VolumeReconstruction::VolumeReconstruction(QObject *parent, QSettings* parent_settings): 
	ScannerBase(parent, parent_settings),
	tsdf(new cpu_tsdf::TSDFVolumeOctree)
{	
	const float & x_vol = std::round(settings->value("CPU_TSDF_SETTINGS/X_VOL").toFloat());
	const float & y_vol = std::round(settings->value("CPU_TSDF_SETTINGS/Y_VOL").toFloat());
	const float & z_vol = std::round(settings->value("CPU_TSDF_SETTINGS/Z_VOL").toFloat());
	tsdf->setGridSize(x_vol, y_vol, z_vol);
	
	const int & x_res = settings->value("CPU_TSDF_SETTINGS/X_RES").toInt();
	const int & y_res = settings->value("CPU_TSDF_SETTINGS/Y_RES").toInt();
	const int & z_res = settings->value("CPU_TSDF_SETTINGS/Z_RES").toInt();
	tsdf->setResolution(x_res, y_res, z_res);
	tsdf->setIntegrateColor(true);
	
	Eigen::Affine3d tsdf_center(Eigen::Affine3d::Identity()); // Optionally offset the center
	const double & x_shift = settings->value("CPU_TSDF_SETTINGS/X_SHIFT").toDouble();
	const double & y_shift = settings->value("CPU_TSDF_SETTINGS/Y_SHIFT").toDouble();
	const double & z_shift = settings->value("CPU_TSDF_SETTINGS/Z_SHIFT").toDouble();
	tsdf_center.translation() << x_shift, y_shift, z_shift;
	tsdf->setGlobalTransform(tsdf_center);
	tsdf->reset();
}

void VolumeReconstruction::addPointCloudVector(
		const PcdPtrVector & point_cloud_vector,
		const Matrix4fVector & translation_matrix_vector
	)
{
	for (size_t i = 0; i < point_cloud_vector.size(); i++)
	{
		addPointCloud(point_cloud_vector[i], translation_matrix_vector[i]);
		qDebug() << "TSDF Integration" << i + 1 << "/" << point_cloud_vector.size();
	}
}

void VolumeReconstruction::addPointCloud(
		const PcdPtr & point_cloud,
		const Eigen::Matrix4f & translation_matrix
	)
{
	Eigen::Affine3d trans(Eigen::Affine3d::Identity());
	trans(0, 0) = translation_matrix(0, 0);
	trans(0, 1) = translation_matrix(0, 1);
	trans(0, 2) = translation_matrix(0, 2);
	trans(0, 3) = translation_matrix(0, 3);
	trans(1, 0) = translation_matrix(1, 0);
	trans(1, 1) = translation_matrix(1, 1);
	trans(1, 2) = translation_matrix(1, 2);
	trans(1, 3) = translation_matrix(1, 3);
	trans(2, 0) = translation_matrix(2, 0);
	trans(2, 1) = translation_matrix(2, 1);
	trans(2, 2) = translation_matrix(2, 2);
	trans(2, 3) = translation_matrix(2, 3);

	tsdf->integrateCloud(*point_cloud, NormalPcd(), trans); // Integrate the cloud
}

void VolumeReconstruction::prepareVolume()
{
	float distance = 0; 
	pcl::PointXYZ query_point(1.0, 2.0, -1.0);
	
	tsdf->getFxn(query_point, distance); // distance is normalized by the truncation limit -- goes from -1 to 1
	//pcl::PointCloud<pcl::PointNormal>::Ptr raytraced = tsdf->renderView(pose_to_render_from); // Optionally can render it
	
	if (settings->value("SAVING_FINAL_POINT_CLOUD_SETTINGS/SAVE_VOL").toBool())
	{
		QString filename = settings->value("SAVING_FINAL_POINT_CLOUD_SETTINGS/FINAL_VOL_FILENAME").toString();
		qDebug() << "Saving" << filename.toStdString().c_str() << "...";
		tsdf->save(filename.toStdString());
		qDebug() << "Done!";
	}
}

void VolumeReconstruction::calculateMesh()
{
	qDebug() << "Calculating mesh...";
	cpu_tsdf::MarchingCubesTSDFOctree mc;
	mc.setInputTSDF(tsdf);
	const int & min_weight = settings->value("CPU_TSDF_SETTINGS/MIN_WEIGHT").toInt();
	mc.setMinWeight(min_weight);	// Sets the minimum weight -- i.e. if a voxel sees a point less than 2 times, it will not render  a mesh triangle at that location
	mc.setColorByRGB(true);			// If true, tries to use the RGB values of the TSDF for meshing -- required if you want a colored mesh	
	mc.reconstruct(_mesh);
	qDebug() << "Done!";

	if (settings->value("SAVING_FINAL_POINT_CLOUD_SETTINGS/SAVE_PLY").toBool())
	{
		QString filename = settings->value("SAVING_FINAL_POINT_CLOUD_SETTINGS/FINAL_PLY_FILENAME").toString();
		qDebug() << "Saving" << filename.toStdString().c_str() << "...";
		pclio::save_one_polygon_mesh(filename, _mesh);
		qDebug() << "Done!";
	}
}

void VolumeReconstruction::getPoligonMesh(
	pcl::PolygonMesh & mesh
	)
{
	mesh = _mesh;
}
