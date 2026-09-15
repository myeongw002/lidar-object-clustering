#pragma once

#include "lidar_object_clustering/types.hpp"

namespace lidar_object_clustering
{

class RoiFilter
{
public:
  RoiFilter(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

  PointCloudT::Ptr filter(const PointCloudT::ConstPtr & cloud) const;

private:
  float min_x_;
  float max_x_;
  float min_y_;
  float max_y_;
  float min_z_;
  float max_z_;
};

}  // namespace lidar_object_clustering
