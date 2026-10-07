#include <filesystem>
#include <iostream>
#include <stdexcept>
#include "registered_pcd.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  const std::string file = std::string(argv[1]) + "/registered_pcd_test.pcd";
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int i = 0; i < 100; ++i) {
    pcl::PointXYZI p;
    p.x = i * .05f; p.y = (i % 7) * .1f; p.z = (i % 13) * .1f; p.intensity = i;
    cloud.push_back(p);
  }
  int cases = 0;
  for (double yaw : {0., .6, -2., 3.14}) {
    for (double pitch : {0., .5235987756, -.4}) {
      Eigen::Affine3d tf = Eigen::Affine3d::Identity();
      tf.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();
      tf.translation() << .3, -.7, .45;
      for (int repeat = 0; repeat < 2; ++repeat) {
        if (!point_lio::saveRegisteredPcd(file, cloud, tf, true)) {return 3;}
        pcl::PointCloud<pcl::PointXYZI> saved;
        if (pcl::io::loadPCDFile(file, saved) != 0) {return 4;}
        for (size_t i = 0; i < cloud.size(); ++i) {
          Eigen::Vector3d expected = tf * Eigen::Vector3d(cloud[i].x, cloud[i].y, cloud[i].z);
          if ((saved[i].getVector3fMap().cast<double>() - expected).norm() > 1e-5 ||
              saved[i].intensity != cloud[i].intensity || cloud[i].x != i*.05f) {
            throw std::runtime_error("PCD frame mismatch, field loss, or accumulator mutation");
          }
        }
        ++cases;
      }
    }
  }
  std::filesystem::remove(file);
  if (point_lio::saveRegisteredPcd(file, cloud, Eigen::Affine3d::Identity(), false) ||
      std::filesystem::exists(file)) {return 5;}
  auto invalid = Eigen::Affine3d::Identity(); invalid.translation().x() = NAN;
  if (point_lio::saveRegisteredPcd(file, cloud, invalid, true)) {return 6;}
  std::cout << "PASS " << cases << " PCD roundtrips, absent/invalid TF rejected; input unchanged\n";
}
