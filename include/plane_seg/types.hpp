#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace plane_seg {

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;

}  // namespace plane_seg
