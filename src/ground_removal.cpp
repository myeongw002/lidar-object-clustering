#include "lidar_object_clustering/ground_removal.hpp"

#include <cmath>
#include <memory>

#include <Eigen/Core>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace lidar_object_clustering
{

GroundRemoval::GroundRemoval(
  double distance_threshold,
  int max_iterations,
  double eps_angle_deg)
: distance_threshold_(distance_threshold),
  max_iterations_(max_iterations),
  eps_angle_deg_(eps_angle_deg)
{
}

PointCloudT::Ptr GroundRemoval::filter(const PointCloudT::ConstPtr & cloud) const
{
  auto output = std::make_shared<PointCloudT>();
  if (!cloud || cloud->empty()) {
    return output;
  }

  constexpr double kPi = 3.14159265358979323846;

  pcl::SACSegmentation<PointT> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setAxis(Eigen::Vector3f::UnitZ());
  segmentation.setEpsAngle(eps_angle_deg_ * kPi / 180.0);
  segmentation.setDistanceThreshold(distance_threshold_);
  segmentation.setMaxIterations(max_iterations_);
  segmentation.setInputCloud(cloud);

  auto inliers = std::make_shared<pcl::PointIndices>();
  auto coefficients = std::make_shared<pcl::ModelCoefficients>();
  segmentation.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    *output = *cloud;
    return output;
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*output);
  return output;
}

}  // namespace lidar_object_clustering
