#include "lidar_object_clustering/bounding_box.hpp"

#include <algorithm>
#include <limits>

namespace lidar_object_clustering
{

BoundingBoxData BoundingBox::compute(
  const PointCloudT::ConstPtr & cloud,
  const pcl::PointIndices & indices)
{
  BoundingBoxData box;
  box.point_count = indices.indices.size();

  if (!cloud || indices.indices.empty()) {
    return box;
  }

  Eigen::Vector3f min_pt(
    std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max());
  Eigen::Vector3f max_pt(
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::lowest());

  for (const int index : indices.indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) {
      continue;
    }

    const auto & point = cloud->points[static_cast<std::size_t>(index)];
    min_pt.x() = std::min(min_pt.x(), point.x);
    min_pt.y() = std::min(min_pt.y(), point.y);
    min_pt.z() = std::min(min_pt.z(), point.z);
    max_pt.x() = std::max(max_pt.x(), point.x);
    max_pt.y() = std::max(max_pt.y(), point.y);
    max_pt.z() = std::max(max_pt.z(), point.z);
  }

  box.min = min_pt;
  box.max = max_pt;
  box.center = 0.5F * (min_pt + max_pt);
  box.size = max_pt - min_pt;
  return box;
}

}  // namespace lidar_object_clustering
