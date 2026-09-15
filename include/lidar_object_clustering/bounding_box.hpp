#pragma once

#include <pcl/PointIndices.h>

#include "lidar_object_clustering/types.hpp"

namespace lidar_object_clustering
{

class BoundingBox
{
public:
  static BoundingBoxData compute(
    const PointCloudT::ConstPtr & cloud,
    const pcl::PointIndices & indices);
};

}  // namespace lidar_object_clustering
