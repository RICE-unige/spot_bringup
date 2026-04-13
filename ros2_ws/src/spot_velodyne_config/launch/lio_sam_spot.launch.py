from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value="/spot_bringup/config/lio_sam_spot.yaml",
                description="Spot-specific LIO-SAM parameter file.",
            ),
            Node(
                package="lio_sam",
                executable="lio_sam_imuPreintegration",
                name="lio_sam_imuPreintegration",
                parameters=[params_file],
                output="screen",
            ),
            Node(
                package="lio_sam",
                executable="lio_sam_imageProjection",
                name="lio_sam_imageProjection",
                parameters=[params_file],
                output="screen",
            ),
            Node(
                package="lio_sam",
                executable="lio_sam_featureExtraction",
                name="lio_sam_featureExtraction",
                parameters=[params_file],
                output="screen",
            ),
            Node(
                package="lio_sam",
                executable="lio_sam_mapOptimization",
                name="lio_sam_mapOptimization",
                parameters=[params_file],
                output="screen",
            ),
        ]
    )
