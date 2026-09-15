#pragma once

#include "lidar_object_clustering/types.hpp"

namespace lidar_object_clustering
{

class GroundRemoval
{
public:
  GroundRemoval(double distance_threshold, int max_iterations, double eps_angle_deg);

  PointCloudT::Ptr filter(const PointCloudT::ConstPtr & cloud) const;

private:
  double distance_threshold_;
  int max_iterations_;
  double eps_angle_deg_;
};

}  // namespace lidar_object_clustering
