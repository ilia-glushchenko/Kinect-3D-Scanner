#include "pclio.h"

PclIO::PclIO(QObject *parent)
{
	setParent(parent);
}

void PclIO::save_one_point_cloud(
	QString filename,
	PcdPtr point_cloud_ptr
	)
{
	if (filename.isEmpty())
		return;

	if (filename.contains(".pcd"))
		pcl::io::savePCDFileBinary(filename.toStdString(), *point_cloud_ptr.get());
	else if (filename.contains(".ply"))
		pcl::io::savePLYFileASCII(filename.toStdString(), *point_cloud_ptr.get());
	else
		return;
}

void PclIO::save_one_polygon_mesh(
	QString filename,
	pcl::PolygonMesh& mesh
	)
{
	if (filename.isEmpty())
		return;

	if (filename.contains(".ply"))
		pcl::io::save(filename.toStdString(), mesh);
}

void PclIO::save_point_cloud_vector(
	QString filename_pattern,
	PcdPtrVector* point_cloud_vector
	)
{
	if (point_cloud_vector->empty() ||
		filename_pattern.isEmpty())
		return;

	for (int i = 0; i < point_cloud_vector->size(); i++)
		save_one_point_cloud(filename_pattern.arg(i), point_cloud_vector->at(i));
}

void PclIO::load_one_point_cloud(
	QString filename,
	PcdPtr point_cloud_ptr
	)
{
	if (filename.isEmpty())
		return;

	pcl::io::loadPCDFile(filename.toStdString(), *point_cloud_ptr.get());

	scale_one_point_cloud(point_cloud_ptr);
}

void PclIO::scale_one_point_cloud(
	PcdPtr point_cloud_ptr
	)
{
	for (int j = 0; j < point_cloud_ptr.get()->points.size(); j++)
	{
		if (pcl_isnan(point_cloud_ptr.get()->points[j].z) == false)
		{
			point_cloud_ptr.get()->points[j].z /= 1000.0f;
			point_cloud_ptr.get()->points[j].y /= 1000.0f;
			point_cloud_ptr.get()->points[j].x /= 1000.0f;
		}
	}
}