#include <cmath>
#include <fstream>

#include <gtest/gtest.h>

#include "plane_seg/config.hpp"
#include "plane_seg/plane_segmenter.hpp"
#include "plane_seg/synthetic_scene.hpp"

namespace {

using plane_seg::Params;
using plane_seg::PlaneSegmenter;
using plane_seg::SceneSpec;

constexpr float kDeg = static_cast<float>(EIGEN_PI / 180.0);

// Box top 1 m in front of a sensor at the origin, face normal toward the sensor.
SceneSpec facingSensor(float tilt_deg = 0.0f, float yaw_deg = 0.0f) {
  SceneSpec s;
  s.face_pose.translation() << 0.05f, -0.02f, 1.0f;
  s.face_pose.linear() = (Eigen::AngleAxisf(EIGEN_PI, Eigen::Vector3f::UnitX()) *
                          Eigen::AngleAxisf(tilt_deg * kDeg, Eigen::Vector3f::UnitY()) *
                          Eigen::AngleAxisf(yaw_deg * kDeg, Eigen::Vector3f::UnitZ()))
                             .toRotationMatrix();
  return s;
}

float angleDeg(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  return std::acos(std::clamp(a.normalized().dot(b.normalized()), -1.0f, 1.0f)) / kDeg;
}

void expectPose(const plane_seg::Result& r, const SceneSpec& s, float max_angle_deg,
                float max_center_m, float max_extent_m) {
  ASSERT_TRUE(r.success) << r.message;
  const Eigen::Vector3f n_true = s.face_pose.linear().col(2);
  const Eigen::Vector3f x_true = s.face_pose.linear().col(0);
  EXPECT_LT(angleDeg(r.pose.normal, n_true), max_angle_deg);
  // The long axis is only defined up to sign.
  EXPECT_LT(std::min(angleDeg(r.pose.transform.linear().col(0), x_true),
                     angleDeg(r.pose.transform.linear().col(0), -x_true)),
            2.0f * max_angle_deg + 1.0f);
  EXPECT_LT((r.pose.center - s.face_pose.translation()).norm(), max_center_m);
  EXPECT_NEAR(r.pose.extent.x(), s.face_size.x(), max_extent_m);
  EXPECT_NEAR(r.pose.extent.y(), s.face_size.y(), max_extent_m);
}

TEST(PlaneSegmenter, CleanFace) {
  const auto s = facingSensor();
  const auto r = PlaneSegmenter().segment(plane_seg::makeBoxScene(s));
  expectPose(r, s, 0.5f, 0.008f, 0.015f);
  EXPECT_LT(r.pose.rms_error, 0.001f);
}

TEST(PlaneSegmenter, NoisyFaceWithOutliers) {
  auto s = facingSensor();
  s.noise_sigma = 0.003f;
  s.outlier_ratio = 0.10f;
  const auto r = PlaneSegmenter().segment(plane_seg::makeBoxScene(s));
  expectPose(r, s, 1.0f, 0.010f, 0.020f);
}

TEST(PlaneSegmenter, TiltedAndRotatedFace) {
  const auto s = facingSensor(25.0f, 30.0f);
  const auto r = PlaneSegmenter().segment(plane_seg::makeBoxScene(s));
  expectPose(r, s, 0.5f, 0.008f, 0.015f);
}

TEST(PlaneSegmenter, ClusteringRejectsCoplanarNeighbor) {
  auto s = facingSensor();
  s.coplanar_distractor = true;
  const auto r = PlaneSegmenter().segment(plane_seg::makeBoxScene(s));
  // Without the cluster stage the extent would grow by the gap plus the patch.
  expectPose(r, s, 0.5f, 0.008f, 0.015f);
}

TEST(PlaneSegmenter, NormalFacesViewpoint) {
  const auto s = facingSensor(10.0f);
  const auto r = PlaneSegmenter().segment(plane_seg::makeBoxScene(s));
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_GT(r.pose.normal.dot(Eigen::Vector3f::Zero() - r.pose.center), 0.0f);
  EXPECT_NEAR(r.pose.transform.linear().determinant(), 1.0f, 1e-4f);
}

TEST(PlaneSegmenter, TooFewPointsFailsCleanly) {
  auto cloud = pcl::make_shared<plane_seg::Cloud>();
  cloud->push_back({0.f, 0.f, 1.f});
  cloud->push_back({0.1f, 0.f, 1.f});
  const auto r = PlaneSegmenter().segment(cloud);
  EXPECT_FALSE(r.success);
  EXPECT_FALSE(r.message.empty());
}

TEST(Config, ReadsKnownKeysAndRejectsUnknown) {
  const std::string path = testing::TempDir() + "params.yaml";
  {
    std::ofstream f(path);
    f << "# comment\nvoxel_leaf_size: 0.01\nmin_cluster_size: 250   # inline\nviewpoint_z: -0.5\n";
  }
  const Params p = plane_seg::loadParams(path);
  EXPECT_FLOAT_EQ(p.voxel_leaf_size, 0.01f);
  EXPECT_EQ(p.min_cluster_size, 250);
  EXPECT_FLOAT_EQ(p.viewpoint.z(), -0.5f);
  EXPECT_DOUBLE_EQ(p.mls_radius, Params{}.mls_radius);

  {
    std::ofstream f(path);
    f << "not_a_key: 1\n";
  }
  EXPECT_THROW(plane_seg::loadParams(path), std::runtime_error);
}

}  // namespace

namespace {

TEST(PlaneSegmenter, RoiCropSelectsTheBoxInTheBox) {
  // Two boxes; the ROI keeps only the second one.
  auto a = facingSensor();
  auto b = facingSensor(15.0f, 20.0f);
  b.face_pose.translation() << 0.60f, 0.0f, 1.1f;
  auto cloud = plane_seg::makeBoxScene(a);
  *cloud += *plane_seg::makeBoxScene(b);

  Params p;
  p.roi_min << 0.30f, -0.40f, 0.5f;
  p.roi_max << 0.95f, 0.40f, 1.5f;
  const auto r = PlaneSegmenter(p).segment(cloud);
  expectPose(r, b, 0.5f, 0.008f, 0.015f);
  EXPECT_EQ(r.stages.at(1).name, "roi_crop");
}

}  // namespace
