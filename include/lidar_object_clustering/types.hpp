#pragma once

#include <cstddef>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_object_clustering
{

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

struct BoundingBoxData
{
  Eigen::Vector3f min = Eigen::Vector3f::Zero();
  Eigen::Vector3f max = Eigen::Vector3f::Zero();
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f size = Eigen::Vector3f::Zero();
  std::size_t point_count = 0;
};

}  // namespace lidar_object_clustering
