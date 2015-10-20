#ifndef SAMPLECONSENSUSDISTANCEFILTER_H
#define SAMPLECONSENSUSDISTANCEFILTER_H

#include <QObject>
#include <QSettings>
#include <QString>
#include <QDebug>

#include <boost/thread/thread.hpp>
#include <Eigen/StdVector>
#include <pcl/common/common_headers.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/correspondence.h>
#include <pcl/visualization/pcl_visualizer.h>

struct Correspondences
{
	pcl::Correspondences* correspondences;
	std::vector<float>    averege_z_distance;
};

class SampleConsensusDistanceFilter : public QObject
{
	Q_OBJECT

	typedef pcl::PointXYZRGB		   PointType;
	typedef pcl::PointCloud<PointType> Pcd;
	typedef boost::shared_ptr<Pcd>	   PcdPtr;

public:
	SampleConsensusDistanceFilter(QObject *parent);
	void calculate(
		PcdPtr src_input_point_cloud_ptr,
		PcdPtr src_target_point_cloud_ptr,
		pcl::Correspondences src_correspondences_vector,
		PcdPtr dst_input_point_cloud_ptr,
		PcdPtr dst_target_point_cloud_ptr,
		pcl::Correspondences* dst_correspondences_vector
	);

private:
	QSettings*   settings;
	
	PcdPtr _src_input_point_cloud_ptr;
	PcdPtr _src_target_point_cloud_ptr;
	pcl::Correspondences _src_correspondences_vector;

	PcdPtr _dst_input_point_cloud_ptr;
	PcdPtr _dst_target_point_cloud_ptr;
	pcl::Correspondences* _dst_correspondences_vector;

	Correspondences _correspondences;

	float  distance_threshold;
	double inlier_threshold;
	int	   max_iter;
	int	   triplets_size;
	bool   allow_zero_distances;

	std::vector<std::vector<bool>> estimated_correspondences_vector;
	
	void perform_sac();
	void sort_correspondences();
	void remove_zero_distances_correspondences();
	void calculate_all_triplets();
	void calculate_one_triplet_transformation(
		pcl::Correspondences correspondences_vector,
		PcdPtr input_point_cloud_ptr,
		PcdPtr target_point_cloud_ptr
	);
	void estimate_one_triplet_rejections(
		PcdPtr input_point_cloud_ptr,
		PcdPtr target_point_cloud_ptr
	);
	void perform_final_correspondences_rejection();
	void fill_dst_point_clouds();

	void iterative_rejection();
};

#endif // SAMPLECONSENSUSDISTANCEFILTER_H
