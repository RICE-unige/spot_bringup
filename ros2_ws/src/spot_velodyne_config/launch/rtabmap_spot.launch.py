from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    return _value(context, name).lower() in ("1", "true", "yes", "on")


def _float(context, name):
    return float(_value(context, name))


def _int(context, name):
    return int(_value(context, name))


def launch_setup(context, *args, **kwargs):
    namespace = _value(context, "namespace")
    scan_cloud_topic = _value(context, "scan_cloud_topic")
    frame_id = _value(context, "frame_id")
    odom_frame_id = _value(context, "odom_frame_id")
    map_frame_id = _value(context, "map_frame_id")
    database_path = _value(context, "database_path")
    log_level = _value(context, "log_level")

    voxel_size = _value(context, "voxel_size")
    max_correspondence_distance = _value(context, "max_correspondence_distance")
    range_min = _value(context, "range_min")
    range_max = _value(context, "range_max")
    detection_rate = _value(context, "detection_rate")
    global_cloud_topic = _value(context, "global_cloud_topic")
    global_grid_topic = _value(context, "global_grid_topic")
    live_cloud_topic = _value(context, "live_cloud_topic")

    icp_params = {
        "frame_id": frame_id,
        "odom_frame_id": odom_frame_id,
        "publish_tf": _bool(context, "publish_tf_odom"),
        "wait_for_transform": _float(context, "wait_for_transform"),
        "approx_sync": False,
        "topic_queue_size": _int(context, "topic_queue_size"),
        "sync_queue_size": _int(context, "sync_queue_size"),
        "qos": _int(context, "qos"),
        "use_sim_time": _bool(context, "use_sim_time"),
        "Icp/PointToPlane": "true",
        "Icp/PointToPlaneK": "20",
        "Icp/PointToPlaneRadius": "0.0",
        "Icp/MaxCorrespondenceDistance": max_correspondence_distance,
        "Icp/RangeMin": range_min,
        "Icp/RangeMax": range_max,
        "Icp/ReciprocalCorrespondences": "false",
        "Icp/VoxelSize": voxel_size,
        "Icp/PM": "false",
        "Icp/PMOutlierRatio": "0.65",
        "Icp/CorrespondenceRatio": "0.15",
        "Icp/Epsilon": "0.001",
        "Icp/Iterations": "40",
        "Odom/Deskewing": "true",
        "Odom/GuessMotion": "true",
        "Odom/ScanKeyFrameThr": "0.6",
        "Odom/ResetCountdown": "1",
    }

    rtabmap_params = {
        "subscribe_depth": False,
        "subscribe_rgbd": False,
        "subscribe_rgb": False,
        "subscribe_stereo": False,
        "subscribe_scan": False,
        "subscribe_scan_cloud": True,
        "subscribe_odom_info": True,
        "frame_id": frame_id,
        "map_frame_id": map_frame_id,
        # Empty means use the odometry topic from icp_odometry instead of looking up odom from TF.
        "odom_frame_id": _value(context, "rtabmap_odom_frame_id"),
        "publish_tf": _bool(context, "publish_tf_map"),
        "database_path": database_path,
        "wait_for_transform": _float(context, "wait_for_transform"),
        "approx_sync": False,
        "topic_queue_size": _int(context, "topic_queue_size"),
        "sync_queue_size": _int(context, "sync_queue_size"),
        "qos_scan": _int(context, "qos"),
        "qos_odom": _int(context, "qos"),
        "scan_normal_k": 20,
        "use_sim_time": _bool(context, "use_sim_time"),
        "Mem/IncrementalMemory": "true",
        "Mem/InitWMWithAllNodes": "false",
        "Reg/Strategy": "1",
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "true",
        "RGBD/ProximityByTime": "false",
        "RGBD/ProximityPathMaxNeighbors": "10",
        "Rtabmap/DetectionRate": detection_rate,
        "Grid/Sensor": "0",
        "Grid/3D": "true",
        "Grid/CellSize": voxel_size,
        "Grid/RangeMin": range_min,
        "Grid/RangeMax": range_max,
        "Grid/RayTracing": "true",
        "Grid/MapFrameProjection": "true",
        "Grid/NormalsSegmentation": "true",
        "Grid/NormalK": "20",
        "Grid/MaxGroundAngle": "45",
        "Grid/MaxGroundHeight": "0.25",
        "Grid/MinGroundHeight": "-1.0",
        "Grid/MaxObstacleHeight": "2.0",
        "Grid/NoiseFilteringRadius": "0.0",
        "Grid/NoiseFilteringMinNeighbors": "3",
        "Icp/PointToPlane": "true",
        "Icp/PointToPlaneK": "20",
        "Icp/MaxCorrespondenceDistance": max_correspondence_distance,
        "Icp/RangeMin": range_min,
        "Icp/RangeMax": range_max,
        "Icp/VoxelSize": voxel_size,
    }

    rtabmap_args = []
    if _bool(context, "delete_db_on_start"):
        rtabmap_args.append("--delete_db_on_start")

    nodes = [
        Node(
            package="rtabmap_odom",
            executable="icp_odometry",
            name="icp_odometry",
            namespace=namespace,
            output="screen",
            emulate_tty=True,
            parameters=[icp_params],
            remappings=[
                ("scan_cloud", scan_cloud_topic),
                ("odom", "odom"),
            ],
            arguments=["--ros-args", "--log-level", f"icp_odometry:={log_level}"],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            namespace=namespace,
            output="screen",
            emulate_tty=True,
            parameters=[rtabmap_params],
            remappings=[
                ("scan_cloud", scan_cloud_topic),
                ("odom", "odom"),
                ("odom_info", "odom_info"),
                ("map", "map"),
            ],
            arguments=rtabmap_args + ["--ros-args", "--log-level", f"rtabmap:={log_level}"],
        ),
    ]

    if _bool(context, "global_map_assembler_enabled"):
        nodes.append(
            Node(
                package="rtabmap_util",
                executable="map_assembler",
                name="rtabmap_global_map_assembler",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "regenerate_local_grids": _bool(context, "global_map_regenerate_local_grids"),
                        "map_always_update": True,
                        "map_cleanup": True,
                        "map_empty_ray_tracing": True,
                        "cloud_output_voxelized": True,
                        "cloud_subtract_filtering": False,
                        "octomap_tree_depth": _int(context, "octomap_tree_depth"),
                        "Grid/Sensor": "0",
                        "Grid/3D": "true",
                        "Grid/CellSize": voxel_size,
                        "Grid/RangeMin": range_min,
                        "Grid/RangeMax": range_max,
                        "Grid/RayTracing": "true",
                        "Grid/MapFrameProjection": "true",
                        "Grid/NormalsSegmentation": "true",
                        "Grid/NormalK": "20",
                        "Grid/MaxGroundAngle": "45",
                        "Grid/MaxGroundHeight": "0.25",
                        "Grid/MinGroundHeight": "-1.0",
                        "Grid/MaxObstacleHeight": "2.0",
                        "Grid/NoiseFilteringRadius": "0.0",
                        "Grid/NoiseFilteringMinNeighbors": "3",
                    }
                ],
                remappings=[
                    ("mapData", f"/{namespace}/mapData"),
                    ("rtabmap/get_map_data", f"/{namespace}/get_map_data"),
                    ("cloud_map", global_cloud_topic),
                    ("cloud_ground", f"/{namespace}/global_cloud_ground"),
                    ("cloud_obstacles", f"/{namespace}/global_cloud_obstacles"),
                    ("grid_prob_map", global_grid_topic),
                    ("map", f"/{namespace}/global_map"),
                    ("octomap_full", f"/{namespace}/global_octomap_full"),
                    ("octomap_binary", f"/{namespace}/global_octomap_binary"),
                    ("octomap_grid", f"/{namespace}/global_octomap_grid"),
                    ("octomap_ground", f"/{namespace}/global_octomap_ground"),
                    ("octomap_obstacles", f"/{namespace}/global_octomap_obstacles"),
                    ("octomap_empty_space", f"/{namespace}/global_octomap_empty_space"),
                    ("octomap_global_frontier_space", f"/{namespace}/global_octomap_global_frontier_space"),
                    ("octomap_occupied_space", f"/{namespace}/global_octomap_occupied_space"),
                ],
            )
        )

    if _bool(context, "live_cloud_assembler_enabled"):
        nodes.append(
            Node(
                package="rtabmap_util",
                executable="point_cloud_assembler",
                name="rtabmap_live_cloud_assembler",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "fixed_frame_id": map_frame_id,
                        "frame_id": map_frame_id,
                        "max_clouds": _int(context, "live_cloud_max_clouds"),
                        "skip_clouds": _int(context, "live_cloud_skip_clouds"),
                        "circular_buffer": False,
                        "linear_update": _float(context, "live_cloud_linear_update"),
                        "angular_update": _float(context, "live_cloud_angular_update"),
                        "wait_for_transform": _float(context, "live_cloud_wait_for_transform"),
                        "range_min": _float(context, "range_min"),
                        "range_max": _float(context, "range_max"),
                        "voxel_size": _float(context, "live_cloud_voxel_size"),
                        "noise_radius": _float(context, "live_cloud_noise_radius"),
                        "noise_min_neighbors": _int(context, "live_cloud_noise_min_neighbors"),
                        "remove_z": False,
                        "qos": _int(context, "qos"),
                    }
                ],
                remappings=[
                    ("cloud", scan_cloud_topic),
                    ("assembled_cloud", live_cloud_topic),
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="rtabmap"),
            DeclareLaunchArgument("scan_cloud_topic", default_value="/rtabmap/velodyne_points"),
            DeclareLaunchArgument("frame_id", default_value="rtabmap_body"),
            DeclareLaunchArgument("odom_frame_id", default_value="rtabmap_odom"),
            DeclareLaunchArgument("rtabmap_odom_frame_id", default_value=""),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("database_path", default_value="/spot_bringup/maps/rtabmap_spot.db"),
            DeclareLaunchArgument("delete_db_on_start", default_value="True"),
            DeclareLaunchArgument("detection_rate", default_value="2.0"),
            DeclareLaunchArgument("publish_tf_odom", default_value="True"),
            DeclareLaunchArgument("publish_tf_map", default_value="True"),
            DeclareLaunchArgument("topic_queue_size", default_value="10"),
            DeclareLaunchArgument("sync_queue_size", default_value="10"),
            DeclareLaunchArgument("qos", default_value="2"),
            DeclareLaunchArgument("wait_for_transform", default_value="0.3"),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument("voxel_size", default_value="0.15"),
            DeclareLaunchArgument("max_correspondence_distance", default_value="1.5"),
            DeclareLaunchArgument("range_min", default_value="0.9"),
            DeclareLaunchArgument("range_max", default_value="50.0"),
            DeclareLaunchArgument("global_map_assembler_enabled", default_value="True"),
            DeclareLaunchArgument("global_map_regenerate_local_grids", default_value="False"),
            DeclareLaunchArgument("global_cloud_topic", default_value="/rtabmap/global_cloud_map"),
            DeclareLaunchArgument("global_grid_topic", default_value="/rtabmap/global_grid_prob_map"),
            DeclareLaunchArgument("octomap_tree_depth", default_value="16"),
            DeclareLaunchArgument("live_cloud_assembler_enabled", default_value="False"),
            DeclareLaunchArgument("live_cloud_topic", default_value="/rtabmap/live_cloud_map"),
            DeclareLaunchArgument("live_cloud_max_clouds", default_value="20"),
            DeclareLaunchArgument("live_cloud_skip_clouds", default_value="1"),
            DeclareLaunchArgument("live_cloud_linear_update", default_value="0.10"),
            DeclareLaunchArgument("live_cloud_angular_update", default_value="0.0872665"),
            DeclareLaunchArgument("live_cloud_wait_for_transform", default_value="1.0"),
            DeclareLaunchArgument("live_cloud_voxel_size", default_value="0.10"),
            DeclareLaunchArgument("live_cloud_noise_radius", default_value="0.0"),
            DeclareLaunchArgument("live_cloud_noise_min_neighbors", default_value="5"),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=launch_setup),
        ]
    )
