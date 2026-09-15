#include "lidar_object_clustering/voxel_filter.hpp"

#include <pcl/filters/voxel_grid.h>

namespace lidar_object_clustering
{

VoxelFilter::VoxelFilter(float leaf_size)
: leaf_size_(leaf_size)
{
}

PointCloudT::Ptr VoxelFilter::filter(const PointCloudT::ConstPtr & cloud) const
{
  auto output = std::make_shared<PointCloudT>();
  if (!cloud || cloud->empty()) {
    return output;
  }

  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel.filter(*output);
  return output;
}

}  // namespace lidar_object_clustering
