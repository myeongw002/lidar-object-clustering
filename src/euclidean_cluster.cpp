#include "lidar_object_clustering/euclidean_cluster.hpp"

#include <memory>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace lidar_object_clustering
{

EuclideanCluster::EuclideanCluster(
  double tolerance,
  int min_cluster_size,
  int max_cluster_size)
: tolerance_(tolerance),
  min_cluster_size_(min_cluster_size),
  max_cluster_size_(max_cluster_size)
{
}

std::vector<pcl::PointIndices> EuclideanCluster::extract(
  const PointCloudT::ConstPtr & cloud) const
{
  std::vector<pcl::PointIndices> clusters;
  if (!cloud || cloud->empty()) {
    return clusters;
  }

  auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<PointT> extraction;
  extraction.setClusterTolerance(tolerance_);
  extraction.setMinClusterSize(min_cluster_size_);
  extraction.setMaxClusterSize(max_cluster_size_);
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(cloud);
  extraction.extract(clusters);

  return clusters;
}

}  // namespace lidar_object_clustering
