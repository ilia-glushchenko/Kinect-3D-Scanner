#ifndef PCLGICP
#define PCLGICP

#include <gicp/gicp.h>

#include "core/base/scannertypes.h"

class PCLGeneralizedICP {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef dgc::gicp::GICPPoint GICPPoint;
    typedef dgc::gicp::GICPPointSet GICPPointCloud;
    typedef boost::shared_ptr<GICPPointCloud> GICPPointCloudPtr;

    PCLGeneralizedICP()
        : gicp_epsilon(1e-5)
        , max_dist(0.3)
    {
    }

    void setInputCloud(const Pcd::Ptr& input_cloud)
    {
        gicp_input_cloud = pcl_pointcloud2gicp_pointcloud(input_cloud);
    }

    void setTargetCloud(const Pcd::Ptr& target_cloud)
    {
        gicp_target_cloud = pcl_pointcloud2gicp_pointcloud(target_cloud);
    }

    void align()
    {
        if (!gicp_input_cloud || !gicp_target_cloud) {
            throw std::invalid_argument("PCLGeneralizedICP::align !gicp_input_cloud || !gicp_target_cloud");
        }

        dgc_transform_t dgc_base_transformation;
        dgc_transform_identity(dgc_base_transformation);
        dgc_transform_t dgc_final_transformation;
        dgc_transform_identity(dgc_final_transformation);

        gicp_target_cloud->SetDebug(false);
        gicp_target_cloud->SetMaxIterationInner(200);
        gicp_target_cloud->SetMaxIteration(500);

        const int& n = gicp_target_cloud->AlignScan(
            &(*gicp_input_cloud),
            dgc_base_transformation,
            dgc_final_transformation,
            max_dist);

        final_transformation = gdc_mat2eigen_mat(dgc_final_transformation);
    }

    bool hasConverged()
    {
        return true;
    }

    Eigen::Matrix4f getFinalTransformation()
    {
        return final_transformation;
    }

    float getFitnessScore()
    {
        return 1;
    }

private:
    GICPPointCloudPtr gicp_input_cloud;
    GICPPointCloudPtr gicp_target_cloud;

    const double gicp_epsilon;
    const double max_dist;
    Eigen::Matrix4f final_transformation;
    Eigen::Matrix4f base_transformation;

    GICPPoint pclpoint2gicpoint(const PointType& pcl_point)
    {
        GICPPoint gicp_point;
        gicp_point.x = pcl_point.x;
        gicp_point.y = pcl_point.y;
        gicp_point.z = pcl_point.z;

        return gicp_point;
    }

    GICPPointCloudPtr pcl_pointcloud2gicp_pointcloud(const Pcd::Ptr& pcl_cloud)
    {
        GICPPointCloudPtr gicp_cloud_ptr(new GICPPointCloud);

        std::for_each(pcl_cloud->begin(), pcl_cloud->end(), [&](const PointType& p) {
            gicp_cloud_ptr->AppendPoint(pclpoint2gicpoint(p));
        });

        gicp_cloud_ptr->BuildKDTree();
        gicp_cloud_ptr->ComputeMatrices();
        gicp_cloud_ptr->SetGICPEpsilon(gicp_epsilon);

        return gicp_cloud_ptr;
    }

    Eigen::Matrix4f gdc_mat2eigen_mat(const dgc_transform_t& gdc_mat)
    {
        Eigen::Matrix4f eigen_mat;

        eigen_mat(0, 0) = gdc_mat[0][0];
        eigen_mat(0, 1) = gdc_mat[0][1];
        eigen_mat(0, 2) = gdc_mat[0][2];
        eigen_mat(0, 3) = gdc_mat[0][3];

        eigen_mat(1, 0) = gdc_mat[1][0];
        eigen_mat(1, 1) = gdc_mat[1][1];
        eigen_mat(1, 2) = gdc_mat[1][2];
        eigen_mat(1, 3) = gdc_mat[1][3];

        eigen_mat(2, 0) = gdc_mat[2][0];
        eigen_mat(2, 1) = gdc_mat[2][1];
        eigen_mat(2, 2) = gdc_mat[2][2];
        eigen_mat(2, 3) = gdc_mat[2][3];

        eigen_mat(3, 0) = gdc_mat[3][0];
        eigen_mat(3, 1) = gdc_mat[3][1];
        eigen_mat(3, 2) = gdc_mat[3][2];
        eigen_mat(3, 3) = gdc_mat[3][3];

        return eigen_mat;
    }
};

#endif //PCLGICP
