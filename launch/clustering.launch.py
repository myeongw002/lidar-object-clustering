from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('lidar_object_clustering'),
        'config',
        'clustering.yaml',
    )

    config_file = LaunchConfiguration('config_file')
    input_cloud_topic = LaunchConfiguration('input_cloud_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to clustering parameter YAML file',
        ),
        DeclareLaunchArgument(
            'input_cloud_topic',
            default_value='/pc_interpoled',
            description='Input PointCloud2 topic',
        ),
        Node(
            package='lidar_object_clustering',
            executable='lidar_clustering_node',
            name='lidar_clustering_node',
            output='screen',
            parameters=[
                config_file,
                {'input_cloud_topic': input_cloud_topic},
            ],
        ),
    ])
