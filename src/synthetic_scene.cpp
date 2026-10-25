#include "plane_seg/synthetic_scene.hpp"

#include <cmath>
#include <random>

namespace plane_seg {

Cloud::Ptr makeBoxScene(const SceneSpec& spec) {
  std::mt19937 rng(spec.seed);
  std::normal_distribution<float> noise(0.0f, spec.noise_sigma);
  auto cloud = pcl::make_shared<Cloud>();

  // Samples a rectangle spanned by a and b from corner c (face frame), as a
  // depth sensor at the origin would: surfaces facing away are not seen, and
  // the sample density falls with the cosine of the viewing angle.
  auto add_patch = [&](const Eigen::Vector3f& c, const Eigen::Vector3f& a, const Eigen::Vector3f& b,
                       const Eigen::Vector3f& outward) {
    const Eigen::Vector3f center_w = spec.face_pose * (c + 0.5f * (a + b));
    const Eigen::Vector3f normal_w = spec.face_pose.linear() * outward;
    const float cos_view = normal_w.dot(-center_w.normalized());
    if (cos_view <= 0.05f) return;
    const float step = spec.spacing / std::sqrt(cos_view);

    const int na = std::max(1, static_cast<int>(std::round(a.norm() / step)));
    const int nb = std::max(1, static_cast<int>(std::round(b.norm() / step)));
    for (int i = 0; i <= na; ++i) {
      for (int j = 0; j <= nb; ++j) {
        Eigen::Vector3f q = spec.face_pose * (c + a * (float(i) / na) + b * (float(j) / nb));
        if (spec.noise_sigma > 0.0f) q += Eigen::Vector3f(noise(rng), noise(rng), noise(rng));
        cloud->push_back(PointT(q.x(), q.y(), q.z()));
      }
    }
  };

  const float L = spec.face_size.x(), W = spec.face_size.y(), D = spec.box_depth;
  const Eigen::Vector3f ex = Eigen::Vector3f::UnitX(), ey = Eigen::Vector3f::UnitY(), ez = Eigen::Vector3f::UnitZ();
  add_patch({-L / 2, -W / 2, 0}, L * ex, W * ey, ez);  // top face
  if (spec.side_walls) {
    add_patch({L / 2, -W / 2, 0}, W * ey, -D * ez, ex);
    add_patch({-L / 2, -W / 2, 0}, W * ey, -D * ez, -ex);
    add_patch({-L / 2, W / 2, 0}, L * ex, -D * ez, ey);
    add_patch({-L / 2, -W / 2, 0}, L * ex, -D * ez, -ey);
  }
  if (spec.coplanar_distractor) {
    // A neighboring object at the same height, separated by a 15 cm gap.
    add_patch({L / 2 + 0.15f, -0.05f, 0}, 0.10f * ex, 0.10f * ey, ez);
  }

  const auto n_outliers = static_cast<std::size_t>(spec.outlier_ratio * cloud->size());
  const float half = 0.5f * std::max(L, W) + 0.10f;
  std::uniform_real_distribution<float> ux(-half, half), uz(-D - 0.05f, 0.10f);
  for (std::size_t i = 0; i < n_outliers; ++i) {
    const Eigen::Vector3f q = spec.face_pose * Eigen::Vector3f(ux(rng), ux(rng), uz(rng));
    cloud->push_back(PointT(q.x(), q.y(), q.z()));
  }

  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

}  // namespace plane_seg
