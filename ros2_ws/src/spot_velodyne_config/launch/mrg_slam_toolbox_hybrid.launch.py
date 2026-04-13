"""Launch the MRG + slam_toolbox hybrid 2D/3D mapping support nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    slam_toolbox_config = LaunchConfiguration("slam_toolbox_config")
    cloud_topic = LaunchConfiguration("cloud_topic")
    raw_scan_topic = LaunchConfiguration("raw_scan_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    target_frame = LaunchConfiguration("target_frame")
    scan_parent_frame = LaunchConfiguration("scan_parent_frame")
    scan_frame = LaunchConfiguration("scan_frame")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument(
                "slam_toolbox_config",
                default_value="/spot_bringup/config/slam_toolbox_mrg_hybrid.yaml",
            ),
            DeclareLaunchArgument("cloud_topic", default_value="/velodyne/points"),
            DeclareLaunchArgument("raw_scan_topic", default_value="/slam_toolbox/scan_raw"),
            DeclareLaunchArgument("scan_topic", default_value="/slam_toolbox/scan"),
            DeclareLaunchArgument("restamp_scan", default_value="True"),
            DeclareLaunchArgument("target_frame", default_value="mrg_velodyne"),
            DeclareLaunchArgument("scan_parent_frame", default_value="body"),
            DeclareLaunchArgument("scan_frame", default_value="slam_toolbox_velodyne"),
            DeclareLaunchArgument("base_frame", default_value="body"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("scan_x", default_value="0.0"),
            DeclareLaunchArgument("scan_y", default_value="0.0"),
            DeclareLaunchArgument("scan_z", default_value="0.25"),
            DeclareLaunchArgument("scan_roll", default_value="0.0"),
            DeclareLaunchArgument("scan_pitch", default_value="0.0"),
            DeclareLaunchArgument("scan_yaw", default_value="0.0"),
            DeclareLaunchArgument("min_height", default_value="-0.15"),
            DeclareLaunchArgument("max_height", default_value="0.35"),
            DeclareLaunchArgument("angle_increment", default_value="0.008726646"),
            DeclareLaunchArgument("scan_time", default_value="0.1"),
            DeclareLaunchArgument("range_min", default_value="0.8"),
            DeclareLaunchArgument("range_max", default_value="45.0"),
            DeclareLaunchArgument("transform_tolerance", default_value="0.25"),
            DeclareLaunchArgument("transform_publish_period", default_value="0.05"),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                namespace="slam_toolbox",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "target_frame": target_frame,
                        "transform_tolerance": ParameterValue(
                            LaunchConfiguration("transform_tolerance"), value_type=float
                        ),
                        "min_height": ParameterValue(LaunchConfiguration("min_height"), value_type=float),
                        "max_height": ParameterValue(LaunchConfiguration("max_height"), value_type=float),
                        "angle_min": -3.141592653589793,
                        "angle_max": 3.141592653589793,
                        "angle_increment": ParameterValue(
                            LaunchConfiguration("angle_increment"), value_type=float
                        ),
                        "scan_time": ParameterValue(LaunchConfiguration("scan_time"), value_type=float),
                        "range_min": ParameterValue(LaunchConfiguration("range_min"), value_type=float),
                        "range_max": ParameterValue(LaunchConfiguration("range_max"), value_type=float),
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                remappings=[
                    ("cloud_in", cloud_topic),
                    ("scan", raw_scan_topic),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="slam_toolbox_velodyne_static_tf",
                output="screen",
                arguments=[
                    LaunchConfiguration("scan_x"),
                    LaunchConfiguration("scan_y"),
                    LaunchConfiguration("scan_z"),
                    LaunchConfiguration("scan_yaw"),
                    LaunchConfiguration("scan_pitch"),
                    LaunchConfiguration("scan_roll"),
                    scan_parent_frame,
                    scan_frame,
                ],
            ),
            Node(
                package="spot_velodyne_config",
                executable="laserscan_qos_relay.py",
                name="laserscan_qos_relay",
                namespace="slam_toolbox",
                output="screen",
                parameters=[
                    {
                        "input_topic": raw_scan_topic,
                        "output_topic": scan_topic,
                        "restamp": ParameterValue(LaunchConfiguration("restamp_scan"), value_type=bool),
                        "output_frame": scan_frame,
                    }
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                namespace="slam_toolbox",
                output="screen",
                parameters=[
                    slam_toolbox_config,
                    {
                        "use_sim_time": use_sim_time,
                        "base_frame": base_frame,
                        "odom_frame": odom_frame,
                        "map_frame": map_frame,
                        "scan_topic": scan_topic,
                        "transform_publish_period": ParameterValue(
                            LaunchConfiguration("transform_publish_period"), value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
