#pragma once

#include <Eigen/Geometry>

#include "plane_seg/types.hpp"

namespace plane_seg {

// 6-DoF pose of a segmented planar face.
//
// Frame convention:
//   z: plane normal, oriented toward the sensor viewpoint
//   x: long side of the minimum-area bounding rectangle of the face
//   y: z cross x (right-handed)
//   origin: center of that rectangle, on the plane
struct PlanePose {
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector2f extent = Eigen::Vector2f::Zero();  // (length, width), length >= width
  float rms_error = 0.0f;                             // RMS point-to-plane distance [m]
};

// Estimates the face pose from the plane inliers and plane coefficients
// (a, b, c, d) with a*x + b*y + c*z + d = 0.
PlanePose estimatePlanePose(const Cloud& plane, const Eigen::Vector4f& coefficients,
                            const Eigen::Vector3f& viewpoint = Eigen::Vector3f::Zero());

}  // namespace plane_seg
