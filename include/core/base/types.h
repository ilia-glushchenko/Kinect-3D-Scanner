#ifndef TYPES_H
#define TYPES_H

#define WIDTH  640
#define HEIGHT 480

#define DISABLED_INLIER_THRESHOLD 999999999.0f

#include <QObject>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include <pcl/common/transforms.h>

#include "core/base/scannerbase.h"
#include "io/pclio.h"

typedef std::vector<int>			 DepthMap;
typedef pcl::PointXYZRGB		     PointType;
typedef pcl::PointCloud<PointType>	 Pcd;
typedef Pcd::Ptr					 PcdPtr;

typedef pcl::PointNormal		     PointNormal;
typedef pcl::PointCloud<PointNormal> NormalPcd;
typedef NormalPcd::Ptr				 NormalPcdPtr;

typedef std::vector<PcdPtr>				  PcdPtrVector;
typedef std::vector<NormalPcdPtr>		  NormalPcdPtrVector;
typedef std::vector<pcl::Correspondences> CorrespondencesVector;

typedef std::pair<PcdPtr, PcdPtr>			   KeypointsPcdPair;
typedef std::pair<NormalPcdPtr, NormalPcdPtr>  KeypointsNormalPcdPair;

typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> Matrix4fVector;


struct Frame
{
	PcdPtr		 pointCloudPtr;
	NormalPcdPtr pointCloudNormalPcdPtr;
	cv::Mat		 pointCloudImage;
	std::vector<int> pointCloudIndexes;

	Frame() :
		pointCloudPtr(new Pcd),
		pointCloudNormalPcdPtr(new NormalPcd)
	{
	}
		
	Frame & operator=(const Frame & other)
	{
		if (this != &other)
		{
			if (other.pointCloudPtr)
			{
				pointCloudPtr = PcdPtr(new Pcd(*other.pointCloudPtr));
			}

			if (other.pointCloudNormalPcdPtr)
			{
				pointCloudNormalPcdPtr = NormalPcdPtr(new NormalPcd(*other.pointCloudNormalPcdPtr));
			}
			
			pointCloudImage = other.pointCloudImage;
			pointCloudIndexes = other.pointCloudIndexes;
		}

		return *this;
	}


	bool load(const QString & cloud_path, const QString & image_path)
	{
		if (   boost::filesystem::exists(cloud_path.toStdString()) 
			&& boost::filesystem::exists(image_path.toStdString()))
		{
			PcdPtr cloud(new Pcd);
			pclio::load_one_point_cloud(cloud_path, cloud);
			pclio::scale_one_point_cloud(cloud);
			
			cv::Mat image = cv::imread(image_path.toStdString());

			if (!cloud->empty() && !image.empty())
			{
				pointCloudImage = image;
				pointCloudPtr = cloud;
				pointCloudIndexes.clear();
				pointCloudNormalPcdPtr->clear();

				return true;
			}
		}

		return false;
	}


	Frame transform(const Eigen::Matrix4f & transformation) const
	{
		if (!pointCloudPtr || !pointCloudNormalPcdPtr)
		{
			throw std::invalid_argument("Frame::transform !pointCloudPtr || !pointCloudNormalPcdPtr");
		}

		Frame result;
		result.pointCloudImage = pointCloudImage;
		result.pointCloudIndexes = pointCloudIndexes;

		if (!pointCloudPtr->empty())
		{
			pcl::transformPointCloud(*pointCloudPtr, *result.pointCloudPtr, transformation);
		}
		if (!pointCloudNormalPcdPtr->empty())
		{
			pcl::transformPointCloud(*pointCloudNormalPcdPtr, *result.pointCloudNormalPcdPtr, transformation);
		}

		return result;
	}
};
typedef std::vector<Frame> Frames;


/** \brief Main keypoint clouds storage unit. */
struct KeypointsFrame
{
	pcl::Correspondences   keypointsPcdCorrespondences;
	KeypointsPcdPair	   keypointsPcdPair;
	KeypointsNormalPcdPair keypointsNormalPcdPair;
	
	KeypointsFrame() :
		keypointsPcdPair(PcdPtr(new Pcd), PcdPtr(new Pcd)),
		keypointsNormalPcdPair(NormalPcdPtr(new NormalPcd), NormalPcdPtr(new NormalPcd))
	{
	}

	KeypointsFrame & operator=(const KeypointsFrame & other)
	{
		if (!other.keypointsPcdPair.first || !other.keypointsPcdPair.second)
		{
			throw std::invalid_argument(
				"KeypointsFrame::operator= !other.keypointsPcdPair.first || !other.keypointsPcdPair.second"
			);
		}
		if (!other.keypointsNormalPcdPair.first || !other.keypointsNormalPcdPair.second)
		{
			throw std::invalid_argument(
				"KeypointsFrame::operator= !other.keypointsNormalPcdPair.first || !other.keypointsNormalPcdPair.second"
			);
		}


		if (this != &other)
		{
			keypointsPcdCorrespondences = other.keypointsPcdCorrespondences;

			keypointsPcdPair = std::make_pair(
				PcdPtr(new Pcd(*other.keypointsPcdPair.first)), 
				PcdPtr(new Pcd(*other.keypointsPcdPair.second))
			);
			
			keypointsNormalPcdPair = std::make_pair(
				NormalPcdPtr(new NormalPcd(*other.keypointsNormalPcdPair.first)),
				NormalPcdPtr(new NormalPcd(*other.keypointsNormalPcdPair.second))
			);
		}

		return *this;
	}

	KeypointsFrame & operator+=(const KeypointsFrame & other)
	{
		if (!other.keypointsPcdPair.first || !other.keypointsPcdPair.second)
		{
			throw std::invalid_argument(
				"KeypointsFrame::operator+= !other.keypointsPcdPair.first || !other.keypointsPcdPair.second"
			);
		}

		if (!other.keypointsNormalPcdPair.first || !other.keypointsNormalPcdPair.second)
		{
			throw std::invalid_argument(
				"KeypointsFrame::operator+= !other.keypointsNormalPcdPair.first || !other.keypointsNormalPcdPair.second"
			);
		}


		std::copy(other.keypointsPcdCorrespondences.begin(), other.keypointsPcdCorrespondences.end(),
			std::back_inserter(keypointsPcdCorrespondences));

		std::copy(other.keypointsPcdPair.first->begin(), other.keypointsPcdPair.first->end(),
			std::back_inserter(keypointsPcdPair.first->points));
		keypointsPcdPair.first->resize(keypointsPcdPair.first->size());
		std::copy(other.keypointsPcdPair.second->begin(), other.keypointsPcdPair.second->end(),
			std::back_inserter(keypointsPcdPair.second->points));
		keypointsPcdPair.second->resize(keypointsPcdPair.second->size());

		std::copy(other.keypointsNormalPcdPair.first->begin(), other.keypointsNormalPcdPair.first->end(),
			std::back_inserter(keypointsNormalPcdPair.first->points));
		keypointsNormalPcdPair.first->resize(keypointsNormalPcdPair.first->size());
		std::copy(other.keypointsNormalPcdPair.second->begin(), other.keypointsNormalPcdPair.second->end(),
			std::back_inserter(keypointsNormalPcdPair.second->points));
		keypointsNormalPcdPair.second->resize(keypointsNormalPcdPair.second->size());

		return *this;
	}

	KeypointsFrame transformFirst(const Eigen::Matrix4f & transformation)
	{
		if (!keypointsPcdPair.first || !keypointsPcdPair.second)
		{
			throw std::invalid_argument(
				"KeypointsFrame::transform !keypointsPcdPair.first || !keypointsPcdPair.second"
			);
		}

		if (!keypointsNormalPcdPair.first || !keypointsNormalPcdPair.second)
		{
			throw std::invalid_argument(
				"KeypointsFrame::transform !keypointsNormalPcdPair.first || !keypointsNormalPcdPair.second"
			);
		}

		KeypointsFrame result;
		result.keypointsPcdCorrespondences = keypointsPcdCorrespondences;
		*result.keypointsPcdPair.second = *keypointsPcdPair.second;
		*result.keypointsNormalPcdPair.second = *keypointsNormalPcdPair.second;

		if (!keypointsPcdPair.first->empty())
		{
			pcl::transformPointCloud(
				*keypointsPcdPair.first, *result.keypointsPcdPair.first, transformation
			);
		}

		if (!keypointsNormalPcdPair.first->empty())
		{
			pcl::transformPointCloud(
				*keypointsNormalPcdPair.first, *result.keypointsNormalPcdPair.first, transformation
			);
		}

		return result;
	}

	KeypointsFrame transformSecond(const Eigen::Matrix4f & transformation)
	{
		if (!keypointsPcdPair.first || !keypointsPcdPair.second)
		{
			throw std::invalid_argument(
				"KeypointsFrame::transform !keypointsPcdPair.first || !keypointsPcdPair.second"
			);
		}

		if (!keypointsNormalPcdPair.first || !keypointsNormalPcdPair.second)
		{
			throw std::invalid_argument(
				"KeypointsFrame::transform !keypointsNormalPcdPair.first || !keypointsNormalPcdPair.second"
			);
		}

		KeypointsFrame result;
		result.keypointsPcdCorrespondences = keypointsPcdCorrespondences;
		*result.keypointsPcdPair.first = *keypointsPcdPair.first;
		*result.keypointsNormalPcdPair.first = *keypointsNormalPcdPair.first;

		if (!keypointsPcdPair.second->empty())
		{
			pcl::transformPointCloud(
				*keypointsPcdPair.second, *result.keypointsPcdPair.second, transformation
			);
		}

		if (!keypointsPcdPair.second->empty())
		{
			pcl::transformPointCloud(
				*keypointsNormalPcdPair.second, *result.keypointsNormalPcdPair.second, transformation
			);
		}

		return result;
	}

	KeypointsFrame transform(const Eigen::Matrix4f & transformation)
	{
		KeypointsFrame result = transformFirst(transformation);		
		return result.transformSecond(transformation);
	}

};
typedef std::vector<KeypointsFrame> KeypointsFrames;

#endif // TYPES_H