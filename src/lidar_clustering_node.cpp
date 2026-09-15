#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lidar_object_clustering/bounding_box.hpp"
#include "lidar_object_clustering/euclidean_cluster.hpp"
#include "lidar_object_clustering/ground_removal.hpp"
#include "lidar_object_clustering/roi_filter.hpp"
#include "lidar_object_clustering/types.hpp"
#include "lidar_object_clustering/voxel_filter.hpp"

namespace lidar_object_clustering
{

class LidarClusteringNode : public rclcpp::Node
{
public:
  LidarClusteringNode()
  : Node("lidar_clustering_node")
  {
    const auto input_topic = declare_parameter<std::string>("input_cloud_topic", "/pc_interpoled");

    const auto roi_min_x = declare_parameter<double>("roi.min_x", 0.3);
    const auto roi_max_x = declare_parameter<double>("roi.max_x", 50.0);
    const auto roi_min_y = declare_parameter<double>("roi.min_y", -15.0);
    const auto roi_max_y = declare_parameter<double>("roi.max_y", 15.0);
    const auto roi_min_z = declare_parameter<double>("roi.min_z", -2.5);
    const auto roi_max_z = declare_parameter<double>("roi.max_z", 3.0);

    const auto ground_distance = declare_parameter<double>("ground.distance_threshold", 0.15);
    const auto ground_iterations = declare_parameter<int>("ground.max_iterations", 100);
    const auto ground_eps_angle = declare_parameter<double>("ground.eps_angle_deg", 15.0);

    const auto voxel_leaf = declare_parameter<double>("voxel.leaf_size", 0.08);

    const auto cluster_tolerance = declare_parameter<double>("clustering.tolerance", 0.35);
    const auto cluster_min_size = declare_parameter<int>("clustering.min_cluster_size", 5);
    const auto cluster_max_size = declare_parameter<int>("clustering.max_cluster_size", 5000);

    const auto nonground_topic = declare_parameter<std::string>("topics.nonground", "/lidar/nonground");
    const auto voxel_topic = declare_parameter<std::string>("topics.voxel", "/lidar/voxel");
    const auto clusters_topic = declare_parameter<std::string>("topics.clusters", "/lidar/clusters");
    const auto boxes_topic = declare_parameter<std::string>("topics.boxes", "/lidar/cluster_boxes");
    const auto objects_topic = declare_parameter<std::string>("topics.objects", "/lidar/objects");

    roi_filter_ = std::make_unique<RoiFilter>(
      static_cast<float>(roi_min_x), static_cast<float>(roi_max_x),
      static_cast<float>(roi_min_y), static_cast<float>(roi_max_y),
      static_cast<float>(roi_min_z), static_cast<float>(roi_max_z));
    ground_removal_ = std::make_unique<GroundRemoval>(
      ground_distance, ground_iterations, ground_eps_angle);
    voxel_filter_ = std::make_unique<VoxelFilter>(static_cast<float>(voxel_leaf));
    euclidean_cluster_ = std::make_unique<EuclideanCluster>(
      cluster_tolerance, cluster_min_size, cluster_max_size);

    nonground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      nonground_topic, rclcpp::SensorDataQoS());
    voxel_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      voxel_topic, rclcpp::SensorDataQoS());
    clusters_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      clusters_topic, rclcpp::SensorDataQoS());
    boxes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(boxes_topic, 10);
    objects_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(objects_topic, 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarClusteringNode::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "LiDAR clustering ready: input=%s, voxel=%.3f m, cluster_tol=%.3f m",
      input_topic.c_str(), voxel_leaf, cluster_tolerance);
  }

private:
  static std::array<std::uint8_t, 3> color_for_cluster(std::size_t id)
  {
    static constexpr std::array<std::array<std::uint8_t, 3>, 12> palette{{
      {{230, 25, 75}}, {{60, 180, 75}}, {{0, 130, 200}}, {{245, 130, 48}},
      {{145, 30, 180}}, {{70, 240, 240}}, {{240, 50, 230}}, {{210, 245, 60}},
      {{250, 190, 212}}, {{0, 128, 128}}, {{220, 190, 255}}, {{170, 110, 40}}
    }};
    return palette[id % palette.size()];
  }

  void publish_xyz_cloud(
    const PointCloudT::ConstPtr & cloud,
    const std_msgs::msg::Header & header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher) const
  {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = header;
    publisher->publish(output);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto input = std::make_shared<PointCloudT>();
    pcl::fromROSMsg(*msg, *input);

    auto cleaned = std::make_shared<PointCloudT>();
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*input, *cleaned, valid_indices);

    const auto roi = roi_filter_->filter(cleaned);
    const auto nonground = ground_removal_->filter(roi);
    const auto voxel = voxel_filter_->filter(nonground);
    const auto cluster_indices = euclidean_cluster_->extract(voxel);

    publish_xyz_cloud(nonground, msg->header, nonground_pub_);
    publish_xyz_cloud(voxel, msg->header, voxel_pub_);
    publish_clusters_and_boxes(voxel, cluster_indices, msg->header);

    RCLCPP_DEBUG(
      get_logger(),
      "points input=%zu roi=%zu nonground=%zu voxel=%zu clusters=%zu",
      input->size(), roi->size(), nonground->size(), voxel->size(), cluster_indices.size());
  }

  void publish_clusters_and_boxes(
    const PointCloudT::ConstPtr & cloud,
    const std::vector<pcl::PointIndices> & cluster_indices,
    const std_msgs::msg::Header & header)
  {
    pcl::PointCloud<pcl::PointXYZRGB> colored;
    colored.points.reserve(cloud->size());
    colored.height = 1;
    colored.is_dense = true;

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    vision_msgs::msg::Detection3DArray objects;
    objects.header = header;

    for (std::size_t cluster_id = 0; cluster_id < cluster_indices.size(); ++cluster_id) {
      const auto color = color_for_cluster(cluster_id);
      const auto & indices = cluster_indices[cluster_id];

      for (const int index : indices.indices) {
        if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) {
          continue;
        }
        const auto & source = cloud->points[static_cast<std::size_t>(index)];
        pcl::PointXYZRGB point;
        point.x = source.x;
        point.y = source.y;
        point.z = source.z;
        point.r = color[0];
        point.g = color[1];
        point.b = color[2];
        colored.points.push_back(point);
      }

      const auto box = BoundingBox::compute(cloud, indices);

      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = "lidar_clusters";
      marker.id = static_cast<int>(cluster_id);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = box.center.x();
      marker.pose.position.y = box.center.y();
      marker.pose.position.z = box.center.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = std::max(0.01F, box.size.x());
      marker.scale.y = std::max(0.01F, box.size.y());
      marker.scale.z = std::max(0.01F, box.size.z());
      marker.color.r = static_cast<float>(color[0]) / 255.0F;
      marker.color.g = static_cast<float>(color[1]) / 255.0F;
      marker.color.b = static_cast<float>(color[2]) / 255.0F;
      marker.color.a = 0.28F;
      markers.markers.push_back(marker);

      vision_msgs::msg::Detection3D detection;
      detection.header = header;
      detection.id = std::to_string(cluster_id);
      detection.bbox.center.position.x = box.center.x();
      detection.bbox.center.position.y = box.center.y();
      detection.bbox.center.position.z = box.center.z();
      detection.bbox.center.orientation.w = 1.0;
      detection.bbox.size.x = box.size.x();
      detection.bbox.size.y = box.size.y();
      detection.bbox.size.z = box.size.z();
      objects.detections.push_back(std::move(detection));
    }

    colored.width = static_cast<std::uint32_t>(colored.points.size());

    sensor_msgs::msg::PointCloud2 cluster_msg;
    pcl::toROSMsg(colored, cluster_msg);
    cluster_msg.header = header;

    clusters_pub_->publish(cluster_msg);
    boxes_pub_->publish(markers);
    objects_pub_->publish(objects);
  }

  std::unique_ptr<RoiFilter> roi_filter_;
  std::unique_ptr<GroundRemoval> ground_removal_;
  std::unique_ptr<VoxelFilter> voxel_filter_;
  std::unique_ptr<EuclideanCluster> euclidean_cluster_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boxes_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr objects_pub_;
};

}  // namespace lidar_object_clustering

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_object_clustering::LidarClusteringNode>());
  rclcpp::shutdown();
  return 0;
}
