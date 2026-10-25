#include "plane_seg/config.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace plane_seg {
namespace {

std::string trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t\r");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r");
  return s.substr(b, e - b + 1);
}

template <typename T>
T parse(const std::string& key, const std::string& value) {
  std::istringstream in(value);
  T out{};
  if (!(in >> out) || !(in >> std::ws).eof())
    throw std::runtime_error("bad value for '" + key + "': " + value);
  return out;
}

}  // namespace

Params loadParams(const std::string& path, Params params) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open config: " + path);

  std::string line;
  int line_no = 0;
  while (std::getline(file, line)) {
    ++line_no;
    line = trim(line.substr(0, line.find('#')));
    if (line.empty()) continue;

    const auto colon = line.find(':');
    if (colon == std::string::npos)
      throw std::runtime_error(path + ":" + std::to_string(line_no) + ": expected 'key: value'");
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1));

    if (key == "voxel_leaf_size") params.voxel_leaf_size = parse<float>(key, value);
    else if (key == "sor_mean_k") params.sor_mean_k = parse<int>(key, value);
    else if (key == "sor_stddev_mul") params.sor_stddev_mul = parse<double>(key, value);
    else if (key == "ransac_threshold_coarse") params.ransac_threshold_coarse = parse<double>(key, value);
    else if (key == "ransac_max_iterations") params.ransac_max_iterations = parse<int>(key, value);
    else if (key == "cluster_tolerance") params.cluster_tolerance = parse<double>(key, value);
    else if (key == "min_cluster_size") params.min_cluster_size = parse<int>(key, value);
    else if (key == "mls_radius") params.mls_radius = parse<double>(key, value);
    else if (key == "ransac_threshold_fine") params.ransac_threshold_fine = parse<double>(key, value);
    else if (key == "min_plane_inliers") params.min_plane_inliers = parse<int>(key, value);
    else if (key == "roi_min_x") params.roi_min.x() = parse<float>(key, value);
    else if (key == "roi_min_y") params.roi_min.y() = parse<float>(key, value);
    else if (key == "roi_min_z") params.roi_min.z() = parse<float>(key, value);
    else if (key == "roi_max_x") params.roi_max.x() = parse<float>(key, value);
    else if (key == "roi_max_y") params.roi_max.y() = parse<float>(key, value);
    else if (key == "roi_max_z") params.roi_max.z() = parse<float>(key, value);
    else if (key == "expected_normal_x") params.expected_normal.x() = parse<float>(key, value);
    else if (key == "expected_normal_y") params.expected_normal.y() = parse<float>(key, value);
    else if (key == "expected_normal_z") params.expected_normal.z() = parse<float>(key, value);
    else if (key == "max_normal_angle_deg") params.max_normal_angle_deg = parse<double>(key, value);
    else if (key == "viewpoint_x") params.viewpoint.x() = parse<float>(key, value);
    else if (key == "viewpoint_y") params.viewpoint.y() = parse<float>(key, value);
    else if (key == "viewpoint_z") params.viewpoint.z() = parse<float>(key, value);
    else throw std::runtime_error(path + ":" + std::to_string(line_no) + ": unknown key '" + key + "'");
  }
  return params;
}

}  // namespace plane_seg
