cmake_minimum_required(VERSION 3.8)
project(spot_velodyne_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/hybrid_map_to_odom_tf_bridge.py
    scripts/laserscan_qos_relay.py
    scripts/map_to_odom_tf_bridge.py
    scripts/mrg_map_align_bridge.py
    scripts/pointcloud_sanitizer.py
    scripts/spot_sdk_imu_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
