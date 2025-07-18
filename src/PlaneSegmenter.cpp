#include "PlaneSegmenter.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/common/io.h>
#include <iostream>

PlaneSegmenter::PlaneSegmenter(const Params& params, bool visualize)
    : params_(params), visualize_(visualize) {}

void PlaneSegmenter::run(const CloudT::Ptr& roi,
                         pcl::ModelCoefficients::Ptr& plane_coeff,
                         CloudT::Ptr& plane_cloud,
                         pcl::PointIndices::Ptr& plane_inliers)
{
    pcl::ModelCoefficients::Ptr init_coeff(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr init_inliers(new pcl::PointIndices());

    if (!segmentPlane(roi, init_coeff, init_inliers)) {
        std::cerr << "\033[1;31mInitial RANSAC failed\033[0m" << std::endl;
        return;
    }

    auto first_plane = extractInliers(roi, init_inliers);
    auto biggest_cluster = extractLargestCluster(first_plane);
    auto downsampled = voxelizeCloud(biggest_cluster);
    auto filtered = removeOutliers(downsampled);
    auto smoothed = smoothCloud(filtered);

    pcl::ModelCoefficients::Ptr final_coeff(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr final_inliers(new pcl::PointIndices());

    if (!segmentPlane(smoothed, final_coeff, final_inliers)) {
        std::cerr << "\033[1;31mFinal RANSAC failed\033[0m" << std::endl;
        return;
    }

    plane_coeff = final_coeff;
    plane_cloud = extractInliers(smoothed, final_inliers);
    plane_inliers = final_inliers;

    if (visualize_) {
        visualize(first_plane, smoothed, filtered, plane_cloud, roi, biggest_cluster);
    }

    std::cout << "\033[1;32mPlane segmentation complete.\033[0m" << std::endl;
}

bool PlaneSegmenter::segmentPlane(const CloudT::Ptr& input,
                                  pcl::ModelCoefficients::Ptr& coeff,
                                  pcl::PointIndices::Ptr& inliers)
{
    pcl::SACSegmentation<NormalT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(coeff == nullptr ? params_.ransac_distance_threshold_init : params_.ransac_distance_threshold_final);
    seg.setInputCloud(input);
    seg.segment(*inliers, *coeff);

    return inliers->indices.size() > 100;
}

PlaneSegmenter::CloudT::Ptr PlaneSegmenter::extractInliers(const CloudT::Ptr& cloud,
                                                            const pcl::PointIndices::Ptr& indices)
{
    pcl::ExtractIndices<NormalT> extract;
    CloudT::Ptr output(new CloudT);
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*output);
    return output;
}

PlaneSegmenter::CloudT::Ptr PlaneSegmenter::extractLargestCluster(const CloudT::Ptr& cloud)
{
    pcl::search::KdTree<NormalT>::Ptr tree(new pcl::search::KdTree<NormalT>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<NormalT> ec;
    ec.setClusterTolerance(params_.cluster_tolerance);
    ec.setMinClusterSize(params_.min_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusters);

    if (clusters.empty()) return cloud;

    auto largest = clusters.front();
    for (const auto& cluster : clusters)
        if (cluster.indices.size() > largest.indices.size())
            largest = cluster;

    pcl::ExtractIndices<NormalT> extract;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices(largest));
    CloudT::Ptr result(new CloudT);
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*result);
    return result;
}

PlaneSegmenter::CloudT::Ptr PlaneSegmenter::voxelizeCloud(const CloudT::Ptr& cloud)
{
    pcl::VoxelGrid<NormalT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);
    CloudT::Ptr downsampled(new CloudT);
    voxel.filter(*downsampled);
    return downsampled;
}

PlaneSegmenter::CloudT::Ptr PlaneSegmenter::removeOutliers(const CloudT::Ptr& cloud)
{
    pcl::StatisticalOutlierRemoval<NormalT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(params_.sor_mean_k);
    sor.setStddevMulThresh(params_.sor_stddev_mul_thresh);
    CloudT::Ptr filtered(new CloudT);
    sor.filter(*filtered);
    return filtered;
}

PlaneSegmenter::CloudT::Ptr PlaneSegmenter::smoothCloud(const CloudT::Ptr& cloud)
{
    pcl::MovingLeastSquares<NormalT, NormalT> mls;
    mls.setInputCloud(cloud);
    //mls.setPolynomialFit(true);
    mls.setPolynomialOrder(1);
    mls.setSearchRadius(params_.mls_radius);
    CloudT::Ptr smoothed(new CloudT);
    mls.process(*smoothed);
    return smoothed;
}

void PlaneSegmenter::visualize(const CloudT::Ptr& raw_plane,
                               const CloudT::Ptr& smoothed,
                               const CloudT::Ptr& filtered,
                               const CloudT::Ptr& final_plane,
                               const CloudT::Ptr& roi,
                               const CloudT::Ptr& cluster)
{
    // Optional: Use pcl::visualization if you want to show in GUI
    std::cout << "[VISUALIZATION STUB] Cloud sizes:" << std::endl;
    std::cout << " - ROI: " << roi->size() << std::endl;
    std::cout << " - First RANSAC Plane: " << raw_plane->size() << std::endl;
    std::cout << " - Clustered: " << cluster->size() << std::endl;
    std::cout << " - Filtered: " << filtered->size() << std::endl;
    std::cout << " - Smoothed: " << smoothed->size() << std::endl;
    std::cout << " - Final Plane: " << final_plane->size() << std::endl;
}

