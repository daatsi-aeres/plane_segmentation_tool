// Monte Carlo accuracy on synthetic box scenes with known ground truth.
//
//   evaluate [--trials N] [--noise SIGMA_M] [--outliers RATIO] [--max-tilt DEG] [--prior 0|1]
//
// Each trial draws a random face pose (tilt up to --max-tilt, any yaw, 0.7-1.3 m
// from the sensor) and face size, runs the pipeline, and records the errors.
// A result whose normal is more than 10 degrees off counts as a wrong plane
// (usually a box side) and is reported separately from the error statistics.
// --prior 1 sets the orientation prior to the camera axis (top-down picking).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "plane_seg/plane_segmenter.hpp"
#include "plane_seg/synthetic_scene.hpp"

namespace {

struct Summary {
  double mean = 0, p95 = 0, max = 0;
};

Summary summarize(std::vector<double> v) {
  Summary s;
  if (v.empty()) return s;
  std::sort(v.begin(), v.end());
  for (double x : v) s.mean += x;
  s.mean /= v.size();
  s.p95 = v[static_cast<std::size_t>(0.95 * (v.size() - 1))];
  s.max = v.back();
  return s;
}

}  // namespace

int main(int argc, char** argv) {
  int trials = 200;
  float noise = 0.002f, outliers = 0.05f, max_tilt = 40.0f;
  bool prior = false;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string arg = argv[i];
    if (arg == "--trials") trials = std::stoi(argv[i + 1]);
    else if (arg == "--noise") noise = std::stof(argv[i + 1]);
    else if (arg == "--outliers") outliers = std::stof(argv[i + 1]);
    else if (arg == "--max-tilt") max_tilt = std::stof(argv[i + 1]);
    else if (arg == "--prior") prior = std::stoi(argv[i + 1]) != 0;
  }

  constexpr float kDeg = static_cast<float>(EIGEN_PI / 180.0);
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> uni(0.0f, 1.0f);
  plane_seg::Params params;
  if (prior) params.expected_normal = Eigen::Vector3f::UnitZ();
  const plane_seg::PlaneSegmenter segmenter(params);

  std::vector<double> normal_deg, yaw_deg, center_mm, length_mm, width_mm, ms;
  int failures = 0, wrong_plane = 0;
  for (int t = 0; t < trials; ++t) {
    plane_seg::SceneSpec s;
    s.seed = 1000 + t;
    s.noise_sigma = noise;
    s.outlier_ratio = outliers;
    s.face_size << 0.25f + 0.25f * uni(rng), 0.15f + 0.10f * uni(rng);
    const float tilt = max_tilt * uni(rng), tilt_dir = 360.0f * uni(rng), yaw = 360.0f * uni(rng);
    s.face_pose.translation() << 0.2f * (uni(rng) - 0.5f), 0.2f * (uni(rng) - 0.5f), 0.7f + 0.6f * uni(rng);
    const Eigen::Vector3f tilt_axis(std::cos(tilt_dir * kDeg), std::sin(tilt_dir * kDeg), 0.0f);
    s.face_pose.linear() = (Eigen::AngleAxisf(tilt * kDeg, tilt_axis) *
                            Eigen::AngleAxisf(EIGEN_PI, Eigen::Vector3f::UnitX()) *
                            Eigen::AngleAxisf(yaw * kDeg, Eigen::Vector3f::UnitZ()))
                               .toRotationMatrix();

    const auto r = segmenter.segment(plane_seg::makeBoxScene(s));
    if (!r.success) {
      ++failures;
      continue;
    }
    const Eigen::Matrix3f R = s.face_pose.linear();
    const Eigen::Matrix3f Rh = r.pose.transform.linear();
    const float cos_n = std::clamp(Rh.col(2).dot(R.col(2)), -1.0f, 1.0f);
    const float cos_x = std::clamp(std::abs(Rh.col(0).dot(R.col(0))), 0.0f, 1.0f);
    if (std::acos(cos_n) / kDeg > 10.0f) {
      ++wrong_plane;
      continue;
    }
    normal_deg.push_back(std::acos(cos_n) / kDeg);
    yaw_deg.push_back(std::acos(cos_x) / kDeg);
    center_mm.push_back(1000.0 * (r.pose.center - s.face_pose.translation()).norm());
    length_mm.push_back(1000.0 * std::abs(r.pose.extent.x() - s.face_size.x()));
    width_mm.push_back(1000.0 * std::abs(r.pose.extent.y() - s.face_size.y()));
    ms.push_back(r.totalMs());
  }

  std::printf("%d trials, noise %.1f mm, outliers %.0f%%, tilt <= %.0f deg, prior %s\n",
              trials, 1000 * noise, 100 * outliers, max_tilt, prior ? "on" : "off");
  std::printf("no plane: %d, wrong plane (normal > 10 deg off): %d\n\n", failures, wrong_plane);
  std::printf("%-22s %8s %8s %8s\n", "error", "mean", "p95", "max");
  auto row = [](const char* name, const std::vector<double>& v) {
    const auto s = summarize(v);
    std::printf("%-22s %8.2f %8.2f %8.2f\n", name, s.mean, s.p95, s.max);
  };
  row("normal [deg]", normal_deg);
  row("in-plane axis [deg]", yaw_deg);
  row("center [mm]", center_mm);
  row("length [mm]", length_mm);
  row("width [mm]", width_mm);
  row("runtime [ms]", ms);
  return failures == 0 && wrong_plane == 0 ? 0 : 3;
}
