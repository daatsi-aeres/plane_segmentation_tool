// Segments the dominant planar face in a PCD file and prints its 6-DoF pose.
//
//   segment_plane <input.pcd> [--config params.yaml] [--output plane.pcd] [--view]

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>

#include "plane_seg/config.hpp"
#include "plane_seg/plane_segmenter.hpp"

#ifdef PLANE_SEG_WITH_VIEWER
#include <pcl/visualization/pcl_visualizer.h>
#endif

namespace {

void usage() {
  std::cerr << "usage: segment_plane <input.pcd> [--config params.yaml] [--output plane.pcd]"
#ifdef PLANE_SEG_WITH_VIEWER
               " [--view]"
#endif
               "\n";
}

void printReport(const plane_seg::Result& r) {
  std::printf("%-18s %10s %9s\n", "stage", "points", "ms");
  for (const auto& s : r.stages) std::printf("%-18s %10zu %9.2f\n", s.name.c_str(), s.points_out, s.ms);
  std::printf("%-18s %10s %9.2f\n\n", "total", "", r.totalMs());

  const auto& pose = r.pose;
  const Eigen::Quaternionf q(pose.transform.linear());
  std::printf("center [m]        %.4f %.4f %.4f\n", pose.center.x(), pose.center.y(), pose.center.z());
  std::printf("normal            %.4f %.4f %.4f\n", pose.normal.x(), pose.normal.y(), pose.normal.z());
  std::printf("quaternion xyzw   %.4f %.4f %.4f %.4f\n", q.x(), q.y(), q.z(), q.w());
  std::printf("face extent [m]   %.4f x %.4f\n", pose.extent.x(), pose.extent.y());
  std::printf("plane RMS [mm]    %.2f\n", 1000.0f * pose.rms_error);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    usage();
    return 1;
  }
  std::string input = argv[1], config, output = "output/plane.pcd";
  bool view = false;
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) config = argv[++i];
    else if (arg == "--output" && i + 1 < argc) output = argv[++i];
    else if (arg == "--view") view = true;
    else {
      usage();
      return 1;
    }
  }

  plane_seg::Params params;
  try {
    if (!config.empty()) params = plane_seg::loadParams(config);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }

  auto cloud = pcl::make_shared<plane_seg::Cloud>();
  if (pcl::io::loadPCDFile(input, *cloud) < 0) {
    std::cerr << "error: cannot read " << input << "\n";
    return 1;
  }
  std::cout << "loaded " << cloud->size() << " points from " << input << "\n\n";

  const plane_seg::Result result = plane_seg::PlaneSegmenter(params).segment(cloud);
  if (!result.success) {
    std::cerr << "segmentation failed: " << result.message << "\n";
    return 2;
  }
  printReport(result);

  const auto parent = std::filesystem::path(output).parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);
  pcl::io::savePCDFileBinary(output, *result.plane);
  std::cout << "\nplane inliers written to " << output << "\n";

#ifdef PLANE_SEG_WITH_VIEWER
  if (view) {
    pcl::visualization::PCLVisualizer viewer("plane_seg");
    viewer.setBackgroundColor(0.08, 0.08, 0.08);
    pcl::visualization::PointCloudColorHandlerCustom<plane_seg::PointT> grey(cloud, 140, 140, 140);
    pcl::visualization::PointCloudColorHandlerCustom<plane_seg::PointT> green(result.plane, 40, 220, 90);
    viewer.addPointCloud(cloud, grey, "input");
    viewer.addPointCloud(result.plane, green, "plane");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "plane");
    viewer.addCoordinateSystem(0.1, Eigen::Affine3f(result.pose.transform.matrix()), "pose");
    viewer.addCoordinateSystem(0.05, "sensor");
    viewer.initCameraParameters();
    viewer.spin();
  }
#else
  if (view) std::cerr << "built without the PCL viewer; --view ignored\n";
#endif
  return 0;
}
