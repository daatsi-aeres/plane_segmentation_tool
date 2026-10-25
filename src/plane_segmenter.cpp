#include "plane_seg/plane_segmenter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

#include <pcl/PointIndices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>

namespace plane_seg {
namespace {

using Clock = std::chrono::steady_clock;

Cloud::Ptr extract(const Cloud::ConstPtr& cloud, const pcl::PointIndices::Ptr& indices) {
  pcl::ExtractIndices<PointT> ex;
  ex.setInputCloud(cloud);
  ex.setIndices(indices);
  auto out = pcl::make_shared<Cloud>();
  ex.filter(*out);
  return out;
}

bool fitPlane(const Cloud::ConstPtr& cloud, double threshold, int max_iterations,
              const Params& p, pcl::PointIndices::Ptr& inliers, pcl::ModelCoefficients& coeff) {
  if (cloud->size() < 3) return false;
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  if (p.expected_normal.squaredNorm() > 0.0f) {
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);  // plane perpendicular to the axis
    seg.setAxis(p.expected_normal.normalized());
    seg.setEpsAngle(p.max_normal_angle_deg * M_PI / 180.0);
  } else {
    seg.setModelType(pcl::SACMODEL_PLANE);
  }
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(threshold);
  seg.setMaxIterations(max_iterations);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, coeff);
  return coeff.values.size() == 4 && !inliers->indices.empty();
}

}  // namespace

double Result::totalMs() const {
  double total = 0.0;
  for (const auto& s : stages) total += s.ms;
  return total;
}

PlaneSegmenter::PlaneSegmenter(Params params) : params_(std::move(params)) {}

Result PlaneSegmenter::segment(const Cloud::ConstPtr& input) const {
  Result result;
  const auto& p = params_;

  Cloud::Ptr cloud;
  auto stage = [&](const char* name, auto&& fn) {
    const auto t0 = Clock::now();
    cloud = fn();
    const std::chrono::duration<double, std::milli> dt = Clock::now() - t0;
    result.stages.push_back({name, cloud->size(), dt.count()});
  };
  auto fail = [&](std::string msg) {
    result.message = std::move(msg);
    return result;
  };

  stage("remove_nan", [&] {
    auto out = pcl::make_shared<Cloud>();
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*input, *out, idx);
    return out;
  });

  if ((p.roi_min.array() < p.roi_max.array()).all()) {
    stage("roi_crop", [&] {
      pcl::CropBox<PointT> crop;
      crop.setInputCloud(cloud);
      crop.setMin(p.roi_min.homogeneous());
      crop.setMax(p.roi_max.homogeneous());
      auto out = pcl::make_shared<Cloud>();
      crop.filter(*out);
      return out;
    });
  }

  if (p.voxel_leaf_size > 0.0f) {
    stage("voxel_grid", [&] {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(p.voxel_leaf_size, p.voxel_leaf_size, p.voxel_leaf_size);
      auto out = pcl::make_shared<Cloud>();
      vg.filter(*out);
      return out;
    });
  }

  if (p.sor_mean_k > 0 && p.sor_stddev_mul > 0.0) {
    stage("outlier_removal", [&] {
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud(cloud);
      sor.setMeanK(p.sor_mean_k);
      sor.setStddevMulThresh(p.sor_stddev_mul);
      auto out = pcl::make_shared<Cloud>();
      sor.filter(*out);
      return out;
    });
  }

  bool ok = true;
  stage("ransac_coarse", [&] {
    auto inliers = pcl::make_shared<pcl::PointIndices>();
    pcl::ModelCoefficients coeff;
    ok = fitPlane(cloud, p.ransac_threshold_coarse, p.ransac_max_iterations, p, inliers, coeff);
    return ok ? extract(cloud, inliers) : pcl::make_shared<Cloud>();
  });
  if (!ok) return fail("coarse RANSAC found no plane");

  stage("largest_cluster", [&] {
    auto tree = pcl::make_shared<pcl::search::KdTree<PointT>>();
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(p.cluster_tolerance);
    ec.setMinClusterSize(p.min_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusters);
    if (clusters.empty()) return pcl::make_shared<Cloud>();
    const auto largest = std::max_element(
        clusters.begin(), clusters.end(),
        [](const auto& a, const auto& b) { return a.indices.size() < b.indices.size(); });
    return extract(cloud, pcl::make_shared<pcl::PointIndices>(*largest));
  });
  if (cloud->empty()) return fail("no cluster reached min_cluster_size");
  const Cloud::Ptr cluster = cloud;

  if (p.mls_radius > 0.0) {
    stage("mls_smoothing", [&] {
      pcl::MovingLeastSquares<PointT, PointT> mls;
      mls.setInputCloud(cloud);
      mls.setSearchMethod(pcl::make_shared<pcl::search::KdTree<PointT>>());
      mls.setPolynomialOrder(1);
      mls.setSearchRadius(p.mls_radius);
      mls.setComputeNormals(false);
      auto out = pcl::make_shared<Cloud>();
      mls.process(*out);
      return out;
    });
  }

  pcl::ModelCoefficients fine;
  stage("ransac_fine", [&] {
    auto inliers = pcl::make_shared<pcl::PointIndices>();
    ok = fitPlane(cloud, p.ransac_threshold_fine, p.ransac_max_iterations, p, inliers, fine);
    return ok ? extract(cloud, inliers) : pcl::make_shared<Cloud>();
  });
  if (!ok) return fail("fine RANSAC found no plane");
  const Eigen::Vector4f coeff(fine.values[0], fine.values[1], fine.values[2], fine.values[3]);

  // MLS gives a stable plane estimate but bends points near creases (e.g. where
  // the face meets a side wall). Measure the face on the unsmoothed cluster
  // instead: keep its points within the fine threshold of the refined plane.
  stage("face_inliers", [&] {
    const Eigen::Vector3f n = coeff.head<3>();
    const float inv = 1.0f / n.norm();
    auto out = pcl::make_shared<Cloud>();
    for (const auto& pt : *cluster)
      if (std::abs((n.dot(pt.getVector3fMap()) + coeff[3]) * inv) <= p.ransac_threshold_fine)
        out->push_back(pt);
    return out;
  });
  if (static_cast<int>(cloud->size()) < p.min_plane_inliers)
    return fail("fewer than min_plane_inliers points on the refined plane");

  const auto t0 = Clock::now();
  result.pose = estimatePlanePose(*cloud, coeff, p.viewpoint);
  const std::chrono::duration<double, std::milli> dt = Clock::now() - t0;
  result.stages.push_back({"pose", cloud->size(), dt.count()});

  const float d = -result.pose.normal.dot(result.pose.center);
  result.coefficients << result.pose.normal, d;
  result.plane = cloud;
  result.success = true;
  return result;
}

}  // namespace plane_seg
