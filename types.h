#ifndef TYPES_H
#define TYPES_H

#define WIDTH  640
#define HEIGHT 480

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#define DISABLED_INLIER_THRESHOLD 999999999.0f

#include <QObject>
#include <boost/thread/thread.hpp>
#include <Eigen/StdVector>
#include <pcl/common/common_headers.h>
#include <pcl/correspondence.h>
#include <opencv2/opencv.hpp>
#include "scannerbase.h"

typedef pcl::PointXYZRGB		     PointType;
typedef pcl::PointCloud<PointType>	 Pcd;
typedef boost::shared_ptr<Pcd>		 PcdPtr;

typedef pcl::PointNormal		     PointNormal;
typedef pcl::PointCloud<PointNormal> NormalPcd;
typedef NormalPcd::Ptr				 NormalPcdPtr;

typedef std::vector<PcdPtr>				  PcdPtrVector;
typedef std::vector<NormalPcdPtr>		  NormalPcdPtrVector;
typedef std::vector<pcl::Correspondences> CorrespondencesVector;

typedef std::pair<PcdPtr, PcdPtr>			   KeypointsPcdPair;
typedef std::pair<NormalPcdPtr, NormalPcdPtr>  KeypointsNormalPcdPair;

typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> Matrix4fVector;
typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>> Affine3fVector;

typedef std::vector<float> CalibMap;
typedef std::vector<int>   DepthMap;

struct Frame
{
	PcdPtr		 pointCloudPtr;
	NormalPcdPtr pointCloudNormalPcdPtr;
	cv::Mat		 pointCloudImage;
	std::vector<int> pointCloudIndexes;
};
typedef std::vector<Frame> Frames;

/** \brief Main keypoint clouds storage unit. */
struct KeypointsFrame
{
	KeypointsPcdPair	   keypointsPcdPair;
	KeypointsNormalPcdPair keypointsNormalPcdPair;
	pcl::Correspondences   keypointsPcdCorrespondences;
};
typedef std::vector<KeypointsFrame> KeypointsFrames;

#endif // TYPES_H