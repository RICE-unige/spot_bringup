# Spot ROS2 Bringup

![Docker](https://img.shields.io/badge/Docker-Enabled-2496ED?logo=docker&logoColor=white)
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)
![Spot SDK](https://img.shields.io/badge/Spot%20SDK-5.0.1-0F766E)
![3D Mapping](https://img.shields.io/badge/mrg__slam-Supported-0F766E)
![Hybrid SLAM](https://img.shields.io/badge/mrg%2Bslam__toolbox-Experimental-D97706)
![RTAB-Map](https://img.shields.io/badge/RTAB--Map-Experimental-D97706)
![SLAM](https://img.shields.io/badge/MOLA-Experimental-D97706)
![LIO-SAM](https://img.shields.io/badge/LIO--SAM-Experimental-D97706)

Docker-based bringup for Boston Dynamics Spot with a ROS 2 driver path, optional RViz, external Velodyne VLP-16 support, a supported `mrg_slam` 3D mapping workflow, and several experimental SLAM paths.

## Overview

This repository is the lab bringup stack for the RICELab Spot platform.

Default behavior is intentionally conservative:

- no automatic lease claim
- no automatic power on
- no automatic stand
- no autonomous motion

Containers use `network_mode: host`, so ROS 2 topics are visible from other machines on the same DDS settings.

> [!IMPORTANT]
> This repository is a bringup and mapping stack. It is not the full motion-control stack for Spot.

## Quick Start

```bash
git clone --recursive https://github.com/RICE-unige/spot_bringup.git
cd spot_bringup

cp .env.example .env
mkdir -p secrets
printf '%s' 'your_spot_password' > secrets/spot_password.txt
chmod 600 secrets/spot_password.txt
```

Build the supported images:

```bash
DOCKER_BUILDKIT=0 docker build -f Dockerfile -t spot_bringup_driver:humble .
DOCKER_BUILDKIT=0 docker build -f Dockerfile.slam -t spot_bringup_slam:humble .
DOCKER_BUILDKIT=0 docker build -f Dockerfile.mrg -t spot_bringup_mrg:humble .
```

Start the driver:

```bash
docker compose up -d spot-driver
```

On the RICELab SpotCORE, the repository already lives at:

```bash
cd /home/spot/spot_bringup
```

> [!NOTE]
> On the current SpotCORE host, `docker compose build` is unreliable. Use the explicit `DOCKER_BUILDKIT=0 docker build ...` commands above.

## Required Configuration

Create `.env` from `.env.example` and set the values you actually need:

```bash
SPOT_HOSTNAME=192.168.80.3
SPOT_USERNAME=user
SPOT_PASSWORD_FILE=/run/spot-secrets/spot_password.txt
SPOT_NAME=spot

ROS_DOMAIN_ID=17
DISPLAY=:15100
```

Notes:

- keep the real password in `./secrets/spot_password.txt`
- keep `SPOT_PASSWORD_FILE=/run/spot-secrets/spot_password.txt`
- keep `SPOT_NAME=spot` unless you have a reason to remove the namespace
- if `SPOT_NAME=` is empty, clear the robot name field in the RViz Spot panel

The mapping paths assume the external Velodyne is reachable at:

```text
192.168.1.201
```

## Supported Paths

| Path | Command | Status | Purpose |
| --- | --- | --- | --- |
| Spot driver | `docker compose up -d spot-driver` | Supported | Spot state, TF, cameras, diagnostics |
| RViz | `docker compose --profile rviz up -d spot-driver spot-rviz` | Supported | Basic Spot visualization |
| Velodyne | `docker compose --profile slam up -d velodyne` | Supported | VLP-16 packets and point cloud |
| mrg_slam | `docker compose --profile slam up -d velodyne mrg-slam` | Supported | Main 3D LiDAR mapping path |
| mrg_slam RViz | `docker compose --profile slam --profile rviz up -d mrg-rviz` | Supported | 3D mapping view with Spot control panel |
| Stop all bringup containers | `./tools/stop_all.sh` | Supported | Clean shutdown across profiles |

## Experimental Paths

Build extra images only if you need these paths:

```bash
DOCKER_BUILDKIT=0 docker build -f Dockerfile.rtabmap -t spot_bringup_rtabmap:humble .
DOCKER_BUILDKIT=0 docker build -f Dockerfile.liosam -t spot_bringup_liosam:humble .
```

| Path | Command | Status | Purpose |
| --- | --- | --- | --- |
| MRG + slam_toolbox | `docker compose --profile mrg-hybrid up -d` | Experimental | 2D occupancy map plus aligned 3D MRG map |
| MRG + slam_toolbox RViz | `docker compose --profile mrg-hybrid-rviz up -d` | Experimental | Hybrid view with 2D map, 3D map, TF, and Spot panel |
| RTAB-Map | `docker compose --profile rtabmap up -d spot-driver velodyne-rtabmap rtabmap rtabmap-map-odom-bridge` | Experimental | LiDAR ICP SLAM comparison path |
| RTAB-Map RViz | `docker compose --profile rtabmap-rviz up -d rtabmap-rviz` | Experimental | RTAB-Map visualization |
| MOLA LO | `docker compose --profile slam up -d mola-lo` | Experimental | LiDAR odometry development path |
| LIO-SAM | `docker compose --profile liosam up -d velodyne-liosam spot-sdk-imu lio-sam` | Experimental | Isolated LIO-SAM integration |
| LIO-SAM RViz | `docker compose --profile liosam-rviz up -d lio-sam-rviz` | Experimental | LIO-SAM visualization |
| LIO-SAM bridge | `docker compose --profile liosam-bridge up -d lio-sam-map-odom-bridge` | Development only | Opt-in `map -> odom` bridge |

> [!WARNING]
> `mrg_slam` and `mrg + slam_toolbox` are the only working mapping path. The hybrid, RTAB-Map, MOLA, and LIO-SAM paths are for development and comparison work.

## Standard Workflows

### 1. Safe Spot bringup

```bash
docker compose up -d spot-driver
docker compose logs -f spot-driver
```

This gives you Spot state, TF, and the camera topics without claiming or moving the robot.

### 2. Supported 3D mapping

```bash
docker compose --profile slam --profile rviz up -d mrg-rviz
```

This is the main mapping workflow. It starts the Spot driver, Velodyne pipeline, `mrg_slam`, the `map -> odom` bridge, and RViz.

Useful outputs:

- `/mrg_slam/map_points`
- `/floor_detection/floor_points`
- `/scan_matching_odometry/odom`

Save results:

```bash
./tools/save_mrg_map.sh maps/my_map.pcd 0.2
./tools/save_mrg_graph.sh graphs/my_graph
```

### 3. Experimental hybrid 2D + 3D mapping

```bash
docker compose --profile mrg-hybrid-rviz up -d
```

This profile is set up so:

- `slam_toolbox` owns the live `map -> odom -> body` localization path
- `mrg_slam` runs in isolated `mrg_*` frames
- a separate bridge aligns the MRG 3D map into the same `map` frame for RViz

Use this path when you want a 2D occupancy map and the 3D MRG map at the same time.

### 4. Experimental SLAM comparisons

RTAB-Map:

```bash
docker compose --profile rtabmap-rviz up -d rtabmap-rviz
```

LIO-SAM:

```bash
docker compose --profile liosam up -d velodyne-liosam spot-sdk-imu lio-sam
docker compose --profile liosam-rviz up -d lio-sam-rviz
```

MOLA:

```bash
docker compose --profile slam up -d mola-lo
```

## RViz

Allow Docker to use your display:

```bash
xhost +local:docker
```

If RViz does not show up:

- check `DISPLAY` in `.env`
- make sure X11 or VNC is actually running
- use `mrg-rviz` for the supported mapping view

## Stop and Inspect

Stop everything from this repository:

```bash
./tools/stop_all.sh
```

Common checks:

```bash
docker compose ps
docker compose logs -f spot-driver
docker compose logs -f mrg-slam
docker compose logs -f velodyne
docker exec -it spot_driver bash
```

## Practical Notes

### Spot driver

- services are namespaced under `/spot/...` when `SPOT_NAME=spot`
- the RViz Spot panel is the simplest way to claim, power, stand, and sit during supervised tests

### Velodyne

Current payload assumptions:

```text
Model: VLP-16-A
IP: 192.168.1.201
RPM: 600
Returns: Strongest
```

### MOLA

`mola-lo` is wired in, but on the current SpotCORE host the installed `ros-humble-mola*` runtime still crashes on the live Velodyne stream. Keep it experimental.

### LIO-SAM

The current Spot SDK IMU path is not yet a validated field IMU source. Do not treat LIO-SAM as the production mapping workflow.

## Minimal Troubleshooting

### Cannot connect to Spot

```bash
ping 192.168.80.3
```

Check:

- `SPOT_HOSTNAME`
- `SPOT_USERNAME`
- `./secrets/spot_password.txt`

### No Velodyne points

```bash
ping 192.168.1.201
docker compose logs -f velodyne
```

### RViz buttons do not work

If `SPOT_NAME=` is empty, clear the robot name field in the Spot RViz panel.

## Repository Layout

```text
spot_bringup/
|- config/          # Driver and SLAM configs
|- ros2_ws/         # Local ROS 2 support package
|- rviz/            # RViz configs
|- tools/           # Helper scripts
|- spot_ros2/       # Upstream driver submodule
|- docker-compose.yaml
|- Dockerfile
|- Dockerfile.mrg
|- Dockerfile.rtabmap
|- Dockerfile.slam
`- Dockerfile.liosam
```

## References

- [spot_ros2](https://github.com/bdaiinstitute/spot_ros2)
- [Boston Dynamics developer documentation](https://dev.bostondynamics.com/)
- [ROS 2 Humble documentation](https://docs.ros.org/en/humble/)
- [mrg_slam](https://github.com/aserbremen/mrg_slam)
- [MOLA documentation](https://docs.mola-slam.org/latest/)
