#pragma once

#include "lidar_object_clustering/types.hpp"

namespace lidar_object_clustering
{

class VoxelFilter
{
public:
  explicit VoxelFilter(float leaf_size);

  PointCloudT::Ptr filter(const PointCloudT::ConstPtr & cloud) const;

private:
  float leaf_size_;
};

}  // namespace lidar_object_clustering
