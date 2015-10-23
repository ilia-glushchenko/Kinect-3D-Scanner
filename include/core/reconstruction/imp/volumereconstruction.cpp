#include "core/reconstruction/volumereconstruction.h"

VolumeReconstruction::VolumeReconstruction(QObject *parent, QSettings* parent_settings)
	: ScannerBase(parent, parent_settings)
{
	tsdf = boost::shared_ptr<cpu_tsdf::TSDFVolumeOctree>(new cpu_tsdf::TSDFVolumeOctree);
	float x_vol = std::round(settings->value("CPU_TSDF_SETTINGS/X_VOL").toFloat());
	float y_vol = std::round(settings->value("CPU_TSDF_SETTINGS/Y_VOL").toFloat());
	float z_vol = std::round(settings->value("CPU_TSDF_SETTINGS/Z_VOL").toFloat());
	tsdf.get()->setGridSize(x_vol, y_vol, z_vol);
	int x_res = settings->value("CPU_TSDF_SETTINGS/X_RES").toInt();
	int y_res = settings->value("CPU_TSDF_SETTINGS/Y_RES").toInt();
	int z_res = settings->value("CPU_TSDF_SETTINGS/Z_RES").toInt();
	tsdf.get()->setResolution(x_res, y_res, z_res);
	tsdf.get()->setIntegrateColor(true);
	Eigen::Affine3d tsdf_center = Eigen::Affine3d::Identity(); // Optionally offset the center
	double x_shift = settings->value("CPU_TSDF_SETTINGS/X_SHIFT").toDouble();
	double y_shift = settings->value("CPU_TSDF_SETTINGS/Y_SHIFT").toDouble();
	double z_shift = settings->value("CPU_TSDF_SETTINGS/Z_SHIFT").toDouble();
	tsdf_center.translation() << x_shift, y_shift, z_shift;
	tsdf->setGlobalTransform(tsdf_center);
	tsdf.get()->reset();
}

void VolumeReconstruction::addPointCloudVector(
	PcdPtrVector& point_clouds_vector,
	Matrix4fVector& translation_matrix_vector
	)
{
	add_point_cloud_vector(point_clouds_vector, translation_matrix_vector);
}

void VolumeReconstruction::addPointCloud(
	PcdPtr point_cloud,
	Eigen::Matrix4f& translation_matrix
	)
{
	add_point_cloud(point_cloud, translation_matrix);
}

void VolumeReconstruction::prepareVolume()
{
	prepare_volume();
}

void VolumeReconstruction::calculateMesh()
{
	calculate_mesh();
}

void VolumeReconstruction::getPoligonMesh(
	pcl::PolygonMesh& mesh
	)
{
	mesh = _mesh;
}

//--------------------------------------------------------------------------------

void VolumeReconstruction::prepare_volume()
{
	float distance; pcl::PointXYZ query_point(1.0, 2.0, -1.0);
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

void VolumeReconstruction::calculate_mesh()
{
	qDebug() << "Calculating mesh...";
	cpu_tsdf::MarchingCubesTSDFOctree mc;
	mc.setInputTSDF(tsdf);
	int min_weight = settings->value("CPU_TSDF_SETTINGS/MIN_WEIGHT").toInt();
	mc.setMinWeight(min_weight);	// Sets the minimum weight -- i.e. if a voxel sees a point less than 2 times, it will not render  a mesh triangle at that location
	mc.setColorByRGB(true);			// If true, tries to use the RGB values of the TSDF for meshing -- required if you want a colored mesh	
	mc.reconstruct(_mesh);
	qDebug() << "Done!";

	if (settings->value("SAVING_FINAL_POINT_CLOUD_SETTINGS/SAVE_PLY").toBool())
	{
		QString filename = settings->value("SAVING_FINAL_POINT_CLOUD_SETTINGS/FINAL_PLY_FILENAME").toString();
		qDebug() << "Saving" << filename.toStdString().c_str() << "...";
		PclIO::save_one_polygon_mesh(filename, _mesh);
		qDebug() << "Done!";
	}
}

void VolumeReconstruction::add_point_cloud_vector(
	PcdPtrVector& point_cloud_vector,
	Matrix4fVector& translation_matrix_vector
	)
{
	for (size_t i = 0; i < point_cloud_vector.size(); i++)
	{
		pcl::PointCloud<pcl::Normal>::Ptr tmp(new pcl::PointCloud<pcl::Normal>);
		tmp.get()->width  = 0;
		tmp.get()->height = 0;
		tmp.get()->resize(0);

		Eigen::Matrix4f transformation = translation_matrix_vector[i];

		Eigen::Affine3d trans = Eigen::Affine3d::Identity();
		trans(0, 0) = transformation(0, 0);
		trans(0, 1) = transformation(0, 1);
		trans(0, 2) = transformation(0, 2);
		trans(0, 3) = transformation(0, 3);
		trans(1, 0) = transformation(1, 0);
		trans(1, 1) = transformation(1, 1);
		trans(1, 2) = transformation(1, 2);
		trans(1, 3) = transformation(1, 3);
		trans(2, 0) = transformation(2, 0);
		trans(2, 1) = transformation(2, 1);
		trans(2, 2) = transformation(2, 2);
		trans(2, 3) = transformation(2, 3);

		tsdf->integrateCloud(*point_cloud_vector[i].get(), *tmp.get(), trans); // Integrate the cloud
		// Note, the normals aren't being used in the default settings. Feel free to pass in an empty cloud

		qDebug() << "TSDF Integration" << i + 1 << "/" << point_cloud_vector.size();
	}
}

void VolumeReconstruction::add_point_cloud(
	PcdPtr point_cloud,
	Eigen::Matrix4f& translation_matrix
	)
{
	pcl::PointCloud<pcl::Normal>::Ptr tmp(new pcl::PointCloud<pcl::Normal>);
	tmp.get()->width = 0;
	tmp.get()->height = 0;
	tmp.get()->resize(0);

	Eigen::Matrix4f transformation = translation_matrix;

	Eigen::Affine3d trans = Eigen::Affine3d::Identity();
	trans(0, 0) = transformation(0, 0);
	trans(0, 1) = transformation(0, 1);
	trans(0, 2) = transformation(0, 2);
	trans(0, 3) = transformation(0, 3);
	trans(1, 0) = transformation(1, 0);
	trans(1, 1) = transformation(1, 1);
	trans(1, 2) = transformation(1, 2);
	trans(1, 3) = transformation(1, 3);
	trans(2, 0) = transformation(2, 0);
	trans(2, 1) = transformation(2, 1);
	trans(2, 2) = transformation(2, 2);
	trans(2, 3) = transformation(2, 3);

	tsdf->integrateCloud(*point_cloud.get(), *tmp.get(), trans); // Integrate the cloud

	qDebug() << "TSDF Integration 1 / 1";
}