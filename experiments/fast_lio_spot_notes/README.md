# FAST-LIO Quarantine Notes

FAST-LIO was removed from the production `ros2_ws/src` path when the supported SLAM profile moved to MOLA LiDAR odometry.

The previous Spot-specific FAST-LIO files were preserved in the pre-change backup under:

```text
/home/spot/spot_bringup_pre_mola_20260411_163310/ros2_ws/src/spark-fast-lio/spark_fast_lio/config/velodyne_spot.yaml
/home/spot/spot_bringup_pre_mola_20260411_163310/ros2_ws/src/spark-fast-lio/spark_fast_lio/launch/mapping_spot.launch.yaml
```

Do not restore FAST-LIO into the production bringup unless it is reworked as a separate experimental profile with its own Dockerfile and validation checklist.
