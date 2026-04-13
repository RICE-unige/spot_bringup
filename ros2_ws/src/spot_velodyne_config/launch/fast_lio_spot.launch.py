"""Launch FAST-LIO2 (spark-fast-lio) for the Spot VLP-16 profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    lidar_topic = LaunchConfiguration("lidar_topic")
    imu_topic = LaunchConfiguration("imu_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="/spot_bringup/config/fast_lio_spot.yaml",
                description="Spot-specific FAST-LIO2 parameter file.",
            ),
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/velodyne/points",
                description="LiDAR point cloud topic.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/fast_lio/imu",
                description="IMU topic from Spot SDK publisher.",
            ),
            Node(
                package="spark_fast_lio",
                executable="spark_lio_mapping",
                name="fast_lio_mapping",
                parameters=[config_file],
                remappings=[
                    ("lidar", lidar_topic),
                    ("imu", imu_topic),
                ],
                output="screen",
            ),
        ]
    )
