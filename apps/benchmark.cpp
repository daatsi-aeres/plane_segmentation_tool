// Per-stage latency of the pipeline over repeated runs.
//
//   benchmark [input.pcd] [--runs N] [--config params.yaml]
//
// Without an input file it uses a synthetic box scene of about 100k points.

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>

#include "plane_seg/config.hpp"
#include "plane_seg/plane_segmenter.hpp"
#include "plane_seg/synthetic_scene.hpp"

int main(int argc, char** argv) {
  std::string input, config;
  int runs = 50;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--runs" && i + 1 < argc) runs = std::stoi(argv[++i]);
    else if (arg == "--config" && i + 1 < argc) config = argv[++i];
    else input = arg;
  }

  plane_seg::Cloud::Ptr cloud;
  if (input.empty()) {
    plane_seg::SceneSpec spec;
    spec.face_pose.translation() << 0.05f, -0.02f, 1.0f;
    spec.face_pose.linear() = Eigen::AngleAxisf(EIGEN_PI, Eigen::Vector3f::UnitX()).toRotationMatrix();
    spec.spacing = 0.0015f;
    spec.noise_sigma = 0.002f;
    spec.outlier_ratio = 0.05f;
    cloud = plane_seg::makeBoxScene(spec);
    input = "synthetic box scene";
  } else {
    cloud = pcl::make_shared<plane_seg::Cloud>();
    if (pcl::io::loadPCDFile(input, *cloud) < 0) {
      std::cerr << "error: cannot read " << input << "\n";
      return 1;
    }
  }

  plane_seg::Params params;
  if (!config.empty()) params = plane_seg::loadParams(config);
  const plane_seg::PlaneSegmenter segmenter(params);

  std::vector<std::string> order;
  std::map<std::string, std::vector<double>> ms;
  std::vector<double> total;
  for (int i = 0; i < runs; ++i) {
    const auto r = segmenter.segment(cloud);
    if (!r.success) {
      std::cerr << "run " << i << " failed: " << r.message << "\n";
      return 2;
    }
    for (const auto& s : r.stages) {
      if (!ms.count(s.name)) order.push_back(s.name);
      ms[s.name].push_back(s.ms);
    }
    total.push_back(r.totalMs());
  }

  auto stats = [](std::vector<double> v) {
    std::sort(v.begin(), v.end());
    double mean = 0;
    for (double x : v) mean += x;
    return std::make_pair(mean / v.size(), v[static_cast<std::size_t>(0.95 * (v.size() - 1))]);
  };

  std::printf("%s: %zu points, %d runs\n\n%-18s %9s %9s\n", input.c_str(), cloud->size(), runs,
              "stage", "mean ms", "p95 ms");
  for (const auto& name : order) {
    const auto [mean, p95] = stats(ms[name]);
    std::printf("%-18s %9.2f %9.2f\n", name.c_str(), mean, p95);
  }
  const auto [mean, p95] = stats(total);
  std::printf("%-18s %9.2f %9.2f\n", "total", mean, p95);
  return 0;
}
