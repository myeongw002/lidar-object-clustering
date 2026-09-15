# lidar_object_clustering

ROS 2 Humble C++ package for LiDAR object clustering.

The package consumes a LiDAR `sensor_msgs/msg/PointCloud2`, applies ROI filtering, RANSAC ground removal, voxel downsampling, Euclidean clustering, and publishes debug clouds plus 3D bounding boxes.

## Pipeline

```text
PointCloud2
  -> ROI crop
  -> RANSAC ground removal
  -> VoxelGrid
  -> Euclidean clustering
  -> cluster-colored cloud + 3D boxes + Detection3DArray
```

Default input:

```text
/pc_interpoled
```

Published topics:

```text
/lidar/nonground      sensor_msgs/msg/PointCloud2
/lidar/voxel          sensor_msgs/msg/PointCloud2
/lidar/clusters       sensor_msgs/msg/PointCloud2
/lidar/cluster_boxes  visualization_msgs/msg/MarkerArray
/lidar/objects        vision_msgs/msg/Detection3DArray
```

## Build

```bash
cd ~/ROS2/compa_ws/src
git clone https://github.com/myeongw002/lidar-object-clustering.git

cd ~/ROS2/compa_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lidar_object_clustering
source install/setup.bash
```

## Run

```bash
ros2 launch lidar_object_clustering clustering.launch.py
```

To run the same pipeline on raw VLP-16 points:

```bash
ros2 launch lidar_object_clustering clustering.launch.py \
  input_cloud_topic:=/velodyne_points
```

## Initial parameters

The values in `config/clustering.yaml` are starting points and should be tuned on real data.

- RANSAC ground distance threshold: 0.15 m
- Ground normal tolerance: 15 deg
- Voxel leaf size: 0.08 m
- Euclidean cluster tolerance: 0.35 m
- Minimum cluster size: 5 points
- Maximum cluster size: 5000 points

The input topic is parameterized so raw VLP-16 and interpolated clouds can be compared using the same downstream pipeline.
