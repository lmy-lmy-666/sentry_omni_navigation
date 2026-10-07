#pragma once

#include <cmath>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace point_lio
{
// The 2D SLAM map uses odom (map->odom is identity in mapping mode).
// Every PCD chunk must use that same frame, including periodic saves.
// Never change the accumulator: it remains in the LIO frame until discarded.
template<typename PointT>
bool saveRegisteredPcd(
  const std::string & path, const pcl::PointCloud<PointT> & cloud,
  const Eigen::Affine3d & odom_to_lio, bool transform_received)
{
  if (!transform_received || !odom_to_lio.matrix().allFinite() || cloud.empty()) {
    return false;
  }
  pcl::PointCloud<PointT> registered;
  pcl::transformPointCloud(cloud, registered, odom_to_lio.cast<float>().matrix());
  return pcl::io::savePCDFileBinary(path, registered) == 0;
}
}  // namespace point_lio
