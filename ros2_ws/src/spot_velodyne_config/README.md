# Spot Velodyne VLP-16-A

This package publishes the Spot-mounted Velodyne VLP-16-A for the MOLA SLAM profile.

Detected sensor state on 2026-04-11:

- Model: `VLP-16-A`
- Serial: `11206213390933`
- Sensor IP: `192.168.1.201`
- Data destination: `255.255.255.255:2368`
- Telemetry port: `8308`
- Motor: `600 RPM`
- Returns: `Strongest`

The production SLAM path uses the ROS 2 Velodyne driver and `velodyne_convert_node` so MOLA receives `/velodyne/points` in the LiDAR frame with `x`, `y`, `z`, `intensity`, `ring`, and timing fields. It does not transform the cloud into `body` before MOLA; the static `body -> velodyne` transform is published separately and must be updated when the physical mount calibration is known.

Launch:

```bash
ros2 launch spot_velodyne_config velodyne_vlp16.launch.py
```

Key overrides:

```bash
ros2 launch spot_velodyne_config velodyne_vlp16.launch.py \
  device_ip:=192.168.1.201 \
  parent_frame:=body \
  frame_id:=velodyne \
  x:=0.0 y:=0.0 z:=0.25 roll:=0.0 pitch:=0.0 yaw:=0.0
```
