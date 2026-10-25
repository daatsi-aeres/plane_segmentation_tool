#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/ModelCoefficients.h>

#include "plane_seg/plane_pose.hpp"
#include "plane_seg/types.hpp"

namespace plane_seg {

struct Params {
  // Optional region of interest (axis-aligned box in the cloud frame).
  // Disabled unless roi_min < roi_max on every axis.
  Eigen::Vector3f roi_min = Eigen::Vector3f::Zero();
  Eigen::Vector3f roi_max = Eigen::Vector3f::Zero();

  // Pre-filtering. A value <= 0 disables the stage.
  float voxel_leaf_size = 0.010f;  // [m]
  int sor_mean_k = 20;
  double sor_stddev_mul = 1.0;

  // Coarse plane hypothesis.
  double ransac_threshold_coarse = 0.010;  // [m]
  int ransac_max_iterations = 1000;

  // Optional orientation prior: only accept planes whose normal is within
  // max_normal_angle_deg of expected_normal (either sign). Zero vector disables.
  // For a top-down bin-picking camera this is the optical axis.
  Eigen::Vector3f expected_normal = Eigen::Vector3f::Zero();
  double max_normal_angle_deg = 45.0;

  // Keep the largest connected patch of the coarse plane.
  double cluster_tolerance = 0.015;  // [m]
  int min_cluster_size = 100;

  // Refinement: MLS smoothing, then a tight RANSAC fit.
  double mls_radius = 0.020;             // [m], <= 0 disables
  double ransac_threshold_fine = 0.004;  // [m]
  int min_plane_inliers = 100;

  // Sensor origin in the cloud frame; the plane normal is oriented toward it.
  Eigen::Vector3f viewpoint = Eigen::Vector3f::Zero();
};

struct StageStats {
  std::string name;
  std::size_t points_out = 0;
  double ms = 0.0;
};

struct Result {
  bool success = false;
  std::string message;
  Eigen::Vector4f coefficients = Eigen::Vector4f::Zero();  // normal oriented toward viewpoint
  Cloud::Ptr plane;                                        // final inliers
  PlanePose pose;
  std::vector<StageStats> stages;

  double totalMs() const;
};

// Geometry-first segmentation of the dominant planar face in a point cloud:
//   NaN removal -> ROI crop -> voxel grid -> statistical outlier removal -> coarse RANSAC
//   -> largest Euclidean cluster -> MLS smoothing -> fine RANSAC -> face pose
class PlaneSegmenter {
 public:
  explicit PlaneSegmenter(Params params = {});

  Result segment(const Cloud::ConstPtr& input) const;

  const Params& params() const { return params_; }

 private:
  Params params_;
};

}  // namespace plane_seg
