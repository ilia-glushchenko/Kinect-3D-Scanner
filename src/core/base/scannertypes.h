#pragma once

#define WIDTH 640
#define HEIGHT 480

#define DISABLED_INLIER_THRESHOLD 999999999.0f

#include <QObject>

#include <Eigen/StdVector>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "core/base/scannerbase.h"
#include "io/pclio.h"

typedef std::vector<int> DepthMap;

using PointType = pcl::PointXYZRGB;
using Pcd = pcl::PointCloud<PointType>;
using PcdPtr = Pcd::Ptr;
using PcdPtrVector = std::vector<PcdPtr>;
using NormalType = pcl::PointNormal;
using NormalPcd = pcl::PointCloud<NormalType>;
using NormalPcdPtr = NormalPcd::Ptr;
typedef std::vector<pcl::Correspondences> CorrespondencesVector;

typedef std::pair<PcdPtr, PcdPtr> KeypointsPcdPair;
typedef std::pair<NormalPcdPtr, NormalPcdPtr> KeypointsNormalPcdPair;

typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > Matrix4fVector;

struct Frame 
{
    PcdPtr pointCloudPtr;
    NormalPcdPtr pointCloudNormalPcdPtr;
    cv::Mat pointCloudImage;
    std::vector<int> pointCloudIndexes;

    Frame()
        : pointCloudPtr(std::make_shared<Pcd>())
        , pointCloudNormalPcdPtr(std::make_shared<NormalPcd>())
    {
    }

    Frame& operator=(const Frame& other)
    {
        if (this != &other) {
            if (other.pointCloudPtr) {
                pointCloudPtr = std::make_shared<Pcd>(*other.pointCloudPtr);
            }

            if (other.pointCloudNormalPcdPtr) {
                pointCloudNormalPcdPtr = std::make_shared<NormalPcd>(*other.pointCloudNormalPcdPtr);
            }

            pointCloudImage = other.pointCloudImage;
            pointCloudIndexes = other.pointCloudIndexes;
        }

        return *this;
    }

    inline bool load(const QString& cloud_path, const QString& image_path)
    {
        if (boost::filesystem::exists(cloud_path.toStdString())
            && boost::filesystem::exists(image_path.toStdString())) 
        {
            auto cloud = std::make_shared<Pcd>();
            pcl_io::load_one_point_cloud(cloud_path, cloud);
            pcl_io::scale_one_point_cloud(cloud);

            cv::Mat image = cv::imread(image_path.toStdString());

            if (!cloud->empty() && !image.empty()) {
                pointCloudImage = image;
                pointCloudPtr = cloud;
                pointCloudIndexes.clear();
                pointCloudNormalPcdPtr->clear();

                return true;
            }
        }

        return false;
    }

    inline Frame transform(const Eigen::Matrix4f& transformation) const
    {
        if (!pointCloudPtr || !pointCloudNormalPcdPtr) {
            throw std::invalid_argument("Frame::transform !pointCloudPtr || !pointCloudNormalPcdPtr");
        }

        Frame result;
        result.pointCloudImage = pointCloudImage;
        result.pointCloudIndexes = pointCloudIndexes;

        if (!pointCloudPtr->empty()) {
            pcl::transformPointCloud(*pointCloudPtr, *result.pointCloudPtr, transformation);
        }
        if (!pointCloudNormalPcdPtr->empty()) {
            pcl::transformPointCloud(*pointCloudNormalPcdPtr, *result.pointCloudNormalPcdPtr, transformation);
        }

        return result;
    }
};
typedef std::vector<Frame> Frames;

/** \brief Main key point clouds storage unit. */
struct KeypointsFrame 
{
    pcl::Correspondences keypointsPcdCorrespondences;
    KeypointsPcdPair keypointsPcdPair;
    KeypointsNormalPcdPair keypointsNormalPcdPair;

    KeypointsFrame()
        : keypointsPcdPair(std::make_shared<Pcd>(), 
                           std::make_shared<Pcd>())
        , keypointsNormalPcdPair(std::make_shared<NormalPcd>(), 
                                 std::make_shared<NormalPcd>())
    {
    }

    KeypointsFrame& operator=(const KeypointsFrame& other)
    {
        if (!other.keypointsPcdPair.first || !other.keypointsPcdPair.second) {
            throw std::invalid_argument(
                "KeypointsFrame::operator= !other.keypointsPcdPair.first || !other.keypointsPcdPair.second");
        }
        if (!other.keypointsNormalPcdPair.first || !other.keypointsNormalPcdPair.second) {
            throw std::invalid_argument(
                "KeypointsFrame::operator= !other.keypointsNormalPcdPair.first || !other.keypointsNormalPcdPair.second");
        }

        if (this != &other) {
            keypointsPcdCorrespondences = other.keypointsPcdCorrespondences;

            keypointsPcdPair = std::make_pair(
                std::make_shared<Pcd>(*other.keypointsPcdPair.first),
                std::make_shared<Pcd>(*other.keypointsPcdPair.second));

            keypointsNormalPcdPair = std::make_pair(
                std::make_shared<NormalPcd>(*other.keypointsNormalPcdPair.first),
                std::make_shared<NormalPcd>(*other.keypointsNormalPcdPair.second));
        }

        return *this;
    }

    KeypointsFrame& operator+=(const KeypointsFrame& other)
    {
        if (!other.keypointsPcdPair.first || !other.keypointsPcdPair.second) {
            throw std::invalid_argument(
                "KeypointsFrame::operator+= !other.keypointsPcdPair.first || !other.keypointsPcdPair.second");
        }

        if (!other.keypointsNormalPcdPair.first || !other.keypointsNormalPcdPair.second) {
            throw std::invalid_argument(
                "KeypointsFrame::operator+= !other.keypointsNormalPcdPair.first || !other.keypointsNormalPcdPair.second");
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

    inline KeypointsFrame transformFirst(const Eigen::Matrix4f& transformation)
    {
        if (!keypointsPcdPair.first || !keypointsPcdPair.second) {
            throw std::invalid_argument(
                "KeypointsFrame::transform !keypointsPcdPair.first || !keypointsPcdPair.second");
        }

        if (!keypointsNormalPcdPair.first || !keypointsNormalPcdPair.second) {
            throw std::invalid_argument(
                "KeypointsFrame::transform !keypointsNormalPcdPair.first || !keypointsNormalPcdPair.second");
        }

        KeypointsFrame result;
        result.keypointsPcdCorrespondences = keypointsPcdCorrespondences;
        *result.keypointsPcdPair.second = *keypointsPcdPair.second;
        *result.keypointsNormalPcdPair.second = *keypointsNormalPcdPair.second;

        if (!keypointsPcdPair.first->empty()) {
            pcl::transformPointCloud(
                *keypointsPcdPair.first, *result.keypointsPcdPair.first, transformation);
        }

        if (!keypointsNormalPcdPair.first->empty()) {
            pcl::transformPointCloud(
                *keypointsNormalPcdPair.first, *result.keypointsNormalPcdPair.first, transformation);
        }

        return result;
    }

    inline KeypointsFrame transformSecond(const Eigen::Matrix4f& transformation)
    {
        if (!keypointsPcdPair.first || !keypointsPcdPair.second) {
            throw std::invalid_argument(
                "KeypointsFrame::transform !keypointsPcdPair.first || !keypointsPcdPair.second");
        }

        if (!keypointsNormalPcdPair.first || !keypointsNormalPcdPair.second) {
            throw std::invalid_argument(
                "KeypointsFrame::transform !keypointsNormalPcdPair.first || !keypointsNormalPcdPair.second");
        }

        KeypointsFrame result;
        result.keypointsPcdCorrespondences = keypointsPcdCorrespondences;
        *result.keypointsPcdPair.first = *keypointsPcdPair.first;
        *result.keypointsNormalPcdPair.first = *keypointsNormalPcdPair.first;

        if (!keypointsPcdPair.second->empty()) {
            pcl::transformPointCloud(
                *keypointsPcdPair.second, *result.keypointsPcdPair.second, transformation);
        }

        if (!keypointsPcdPair.second->empty()) {
            pcl::transformPointCloud(
                *keypointsNormalPcdPair.second, *result.keypointsNormalPcdPair.second, transformation);
        }

        return result;
    }

    inline KeypointsFrame transform(const Eigen::Matrix4f& transformation)
    {
        KeypointsFrame result = transformFirst(transformation);
        return result.transformSecond(transformation);
    }
};
typedef std::vector<KeypointsFrame> KeypointsFrames;
