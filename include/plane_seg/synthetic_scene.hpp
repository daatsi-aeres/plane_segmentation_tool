#pragma once

#include <Eigen/Geometry>

#include "plane_seg/types.hpp"

namespace plane_seg {

// A box seen from above, for tests and benchmarks with known ground truth.
// The top face lies in the local xy-plane of face_pose, with its normal along
// local +z; the box body extends along local -z.
struct SceneSpec {
  Eigen::Isometry3f face_pose = Eigen::Isometry3f::Identity();
  Eigen::Vector2f face_size{0.40f, 0.30f};  // (length along x, width along y) [m]
  float box_depth = 0.20f;                  // [m]
  float spacing = 0.004f;                   // sample spacing [m]
  float noise_sigma = 0.0f;                 // isotropic Gaussian noise [m]
  float outlier_ratio = 0.0f;               // uniform outliers, fraction of surface points
  bool side_walls = true;                   // box sides, as visible from the origin
  bool coplanar_distractor = false;         // separate patch in the same plane
  unsigned seed = 42;
};

Cloud::Ptr makeBoxScene(const SceneSpec& spec);

}  // namespace plane_seg
