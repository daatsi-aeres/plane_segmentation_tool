#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <memory>

class PlaneSegmenter
{
public:
    using NormalT = pcl::PointNormal;
    using CloudT = pcl::PointCloud<NormalT>;

    struct Params {
        double ransac_distance_threshold_init = 0.01;
        double ransac_distance_threshold_final = 0.005;
        double cluster_tolerance = 0.02;
        int min_cluster_size = 100;
        int sor_mean_k = 50;
        double sor_stddev_mul_thresh = 1.0;
        double mls_radius = 0.01;
    };

    PlaneSegmenter(const Params& params, bool visualize = false);

    void run(const CloudT::Ptr& roi,
             pcl::ModelCoefficients::Ptr& plane_coeff,
             CloudT::Ptr& plane_cloud,
             pcl::PointIndices::Ptr& plane_inliers);

private:
    CloudT::Ptr voxelizeCloud(const CloudT::Ptr& cloud);
    CloudT::Ptr removeOutliers(const CloudT::Ptr& cloud);
    CloudT::Ptr smoothCloud(const CloudT::Ptr& cloud);
    bool segmentPlane(const CloudT::Ptr& input, pcl::ModelCoefficients::Ptr& coeff, pcl::PointIndices::Ptr& inliers);
    CloudT::Ptr extractInliers(const CloudT::Ptr& cloud, const pcl::PointIndices::Ptr& indices);
    CloudT::Ptr extractLargestCluster(const CloudT::Ptr& cloud);

    void visualize(const CloudT::Ptr& raw_plane,
                   const CloudT::Ptr& smoothed,
                   const CloudT::Ptr& filtered,
                   const CloudT::Ptr& final_plane,
                   const CloudT::Ptr& roi,
                   const CloudT::Ptr& cluster);

    Params params_;
    bool visualize_;
};

