#pragma once

#include <vector>

#include <pcl/PointIndices.h>

#include "lidar_object_clustering/types.hpp"

namespace lidar_object_clustering
{

class EuclideanCluster
{
public:
  EuclideanCluster(double tolerance, int min_cluster_size, int max_cluster_size);

  std::vector<pcl::PointIndices> extract(const PointCloudT::ConstPtr & cloud) const;

private:
  double tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
};

}  // namespace lidar_object_clustering
