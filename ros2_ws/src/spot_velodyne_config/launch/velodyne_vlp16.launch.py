from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_ip = LaunchConfiguration("device_ip")
    frame_id = LaunchConfiguration("frame_id")
    parent_frame = LaunchConfiguration("parent_frame")
    packets_topic = LaunchConfiguration("packets_topic")
    points_topic = LaunchConfiguration("points_topic")
    mola_points_topic = LaunchConfiguration("mola_points_topic")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_ip", default_value="192.168.1.201"),
            DeclareLaunchArgument("frame_id", default_value="velodyne"),
            DeclareLaunchArgument("parent_frame", default_value="body"),
            DeclareLaunchArgument("packets_topic", default_value="/velodyne_packets"),
            DeclareLaunchArgument("points_topic", default_value="/velodyne/points"),
            DeclareLaunchArgument("mola_points_topic", default_value="/velodyne/points_mola"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.25"),
            DeclareLaunchArgument("roll", default_value="0.0"),
            DeclareLaunchArgument("pitch", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            Node(
                package="velodyne_driver",
                executable="velodyne_driver_node",
                name="velodyne_driver",
                output="screen",
                parameters=[
                    {
                        "device_ip": device_ip,
                        "frame_id": frame_id,
                        "model": "VLP16",
                        "rpm": 600.0,
                        "port": 2368,
                        "gps_time": False,
                        "timestamp_first_packet": True,
                    }
                ],
                remappings=[("velodyne_packets", packets_topic)],
            ),
            Node(
                package="velodyne_pointcloud",
                executable="velodyne_transform_node",
                name="velodyne_transform",
                output="screen",
                parameters=[
                    {
                        "calibration": "/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml",
                        "model": "VLP16",
                        "min_range": 0.9,
                        "max_range": 130.0,
                        "view_direction": 0.0,
                        "fixed_frame": "",
                        "target_frame": "",
                        "organize_cloud": False,
                    }
                ],
                remappings=[
                    ("velodyne_packets", packets_topic),
                    ("velodyne_points", points_topic),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="body_to_velodyne_tf",
                output="screen",
                arguments=[
                    "--x",
                    x,
                    "--y",
                    y,
                    "--z",
                    z,
                    "--roll",
                    roll,
                    "--pitch",
                    pitch,
                    "--yaw",
                    yaw,
                    "--frame-id",
                    parent_frame,
                    "--child-frame-id",
                    frame_id,
                ],
            ),
            Node(
                package="spot_velodyne_config",
                executable="pointcloud_sanitizer.py",
                name="mola_pointcloud_sanitizer",
                output="screen",
                parameters=[
                    {
                        "input_topic": points_topic,
                        "output_topic": mola_points_topic,
                    }
                ],
            ),
        ]
    )
