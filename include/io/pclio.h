#ifndef PCLIO_H
#define PCLIO_H

#include <QObject>
#include <QString>
#include <QDebug>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <core/base/types.h>

class PclIO : public QObject
{
	Q_OBJECT

public:
	PclIO(QObject *parent);

	static void save_one_point_cloud(
		QString filename,
		PcdPtr point_cloud_ptr
	);
	static void save_one_polygon_mesh(
		QString filename,
		pcl::PolygonMesh& mesh
	);
	static void save_point_cloud_vector(
		QString filename_pattern,
		PcdPtrVector* point_cloud_vector
	);
	static void load_one_point_cloud(
		QString filename,
		PcdPtr point_cloud_ptr
	);
	static void scale_one_point_cloud(
		PcdPtr point_cloud_ptr
	);
};

#endif // PCLIO_H
