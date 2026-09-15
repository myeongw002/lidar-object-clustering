#include "lidar_object_clustering/roi_filter.hpp"

#include <Eigen/Core>
#include <pcl/filters/crop_box.h>

namespace lidar_object_clustering
{

RoiFilter::RoiFilter(
  float min_x, float max_x,
  float min_y, float max_y,
  float min_z, float max_z)
: min_x_(min_x),
  max_x_(max_x),
  min_y_(min_y),
  max_y_(max_y),
  min_z_(min_z),
  max_z_(max_z)
{
}

PointCloudT::Ptr RoiFilter::filter(const PointCloudT::ConstPtr & cloud) const
{
  auto output = std::make_shared<PointCloudT>();
  if (!cloud || cloud->empty()) {
    return output;
  }

  pcl::CropBox<PointT> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, 1.0F));
  crop.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1.0F));
  crop.filter(*output);
  return output;
}

}  // namespace lidar_object_clustering
