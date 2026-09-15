cmake_minimum_required(VERSION 3.8)
project(lidar_object_clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)

add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_clustering_node
  src/lidar_clustering_node.cpp
  src/roi_filter.cpp
  src/ground_removal.cpp
  src/voxel_filter.cpp
  src/euclidean_cluster.cpp
  src/bounding_box.cpp
)

target_include_directories(lidar_clustering_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(lidar_clustering_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(lidar_clustering_node
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  vision_msgs
  pcl_conversions
)

install(TARGETS lidar_clustering_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
