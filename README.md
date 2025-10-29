# Spot ROS2 Bringup

[![MIT License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Docker](https://img.shields.io/badge/Docker-Enabled-2496ED?logo=docker&logoColor=white)](https://www.docker.com/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)

Docker-based deployment for Boston Dynamics Spot ROS2 driver with RViz visualization.

## Prerequisites

- Docker (>= 20.10)
- Network access to Spot robot
- X11/VNC server for RViz (optional)

## Quick Start

```bash
# Clone with submodules
git clone --recursive https://github.com/RICE-unige/spot_bringup.git
cd spot_bringup

# Configure credentials
cp .env.example .env
vim .env  # Edit with your Spot's IP, username, password

# Build image
docker build -t spot_ros2_rviz:latest .

# Enable X11 (for RViz)
xhost +local:docker

# Start driver
docker-compose up #for the RICELab spot, skip the other steps and start from here. 
```

## 📁 Repository Structure

```
spot_bringup/
├── spot_ros2/              # Submodule - upstream spot_ros2 driver
├── config/
│   └── spot_config.yaml    # Driver configuration
├── Dockerfile              # Image with ROS2 Humble + Spot SDK + RViz2
├── docker-compose.yaml     # Container orchestration
├── .env.example           # Credentials template
└── .gitignore             # Protects .env
```

## Setup

### 1. Clone Repository

```bash
git clone --recursive https://github.com/RICE-unige/spot_bringup.git
cd spot_bringup
```

> [!IMPORTANT]
> The `--recursive` flag is required to fetch the `spot_ros2` submodule and its nested submodules.

If already cloned without `--recursive`:
```bash
git submodule update --init --recursive
```

### 2. Configure Credentials

**Option A: Environment variables (Recommended)**
```bash
cp .env.example .env
vim .env
```

Edit `.env`:
```bash
DISPLAY=:15100              # Your VNC/X11 display
SPOT_NAME=spot
BOSDYN_CLIENT_USERNAME=user
BOSDYN_CLIENT_PASSWORD=your_password
SPOT_IP=192.168.80.3
```

**Option B: Configuration file**

Edit `config/spot_config.yaml`:
```yaml
username: "user"
password: "your_password"  
hostname: "192.168.80.3"
```

### 3. Build Docker Image

```bash
docker build -t spot_ros2_rviz:latest .
# or with sudo
sudo docker build -t spot_ros2_rviz:latest .
```

### 4. Enable X11 (for RViz)

```bash
# For VNC (adjust display number)
DISPLAY=:15100 xhost +local:docker

# For X11
xhost +local:docker
```

## Usage

### Start Driver

```bash
docker-compose up          # Foreground
docker-compose up -d       # Background
```

### Stop Driver

```bash
docker-compose down
```

### Access Container

```bash
docker exec -it spot_driver bash
```

### View Logs

```bash
docker-compose logs -f
```

## RViz Control

> [!WARNING]
> **RViz Panel Service Names:** If `spot_name: ""` (empty) in `config/spot_config.yaml`, you must clear the robot name field in the RViz Spot Driver panel. By default, RViz sets the name to "spot", which adds a namespace (`/spot/claim`), causing buttons to fail. Leave the field empty to use root-level services (`/claim`).


## Configuration

Edit `config/spot_config.yaml` to customize behavior. Key parameters:

```yaml
# Connection
username: "user"
password: "your_password"
hostname: "192.168.80.3"

# Automatic behaviors
auto_claim: False           # Auto-claim lease on startup
auto_power_on: False        # Auto-power motors
auto_stand: False           # Auto-stand after power on

# Cameras
rgb_cameras: False          # False for greyscale cameras
initialize_spot_cam: False  # Enable SpotCam payload

# Sensors
use_velodyne: True          # Enable Velodyne lidar
velodyne_rate: 10.0         # Point cloud rate (Hz)

# Update rates
robot_state_rate: 50.0      # Joint states/TF (Hz)
image_rate: 15.0            # Camera images (Hz)
```

See config file for complete options with inline documentation.

## ROS2 Topics & Services

### Key Topics

```bash
# Robot state
/joint_states
/odometry
/tf

# Cameras
/camera/frontleft/image
/camera/frontright/image
/depth/frontleft/image

# Velodyne
/velodyne/points

# Status
/status/battery_states
/status/estop
```

### Control Services

```bash
# Inside container
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

# Lease control
ros2 service call /claim std_srvs/srv/Trigger
ros2 service call /release std_srvs/srv/Trigger

# Basic control
ros2 service call /power_on std_srvs/srv/Trigger
ros2 service call /stand std_srvs/srv/Trigger
ros2 service call /sit std_srvs/srv/Trigger
ros2 service call /power_off std_srvs/srv/Trigger

# Arm control (if equipped)
ros2 service call /arm_stow std_srvs/srv/Trigger
ros2 service call /arm_unstow std_srvs/srv/Trigger
```

### Example Sequence

```bash
docker exec -it spot_driver bash
source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash

ros2 service call /claim std_srvs/srv/Trigger
ros2 service call /power_on std_srvs/srv/Trigger
sleep 2
ros2 service call /stand std_srvs/srv/Trigger
sleep 3
ros2 service call /sit std_srvs/srv/Trigger
ros2 service call /power_off std_srvs/srv/Trigger
ros2 service call /release std_srvs/srv/Trigger
```

## Troubleshooting

### Cannot connect to Spot

```bash
# Check network
ping 192.168.80.3

# Verify credentials in .env or config/spot_config.yaml
# Ensure Spot is powered on and WiFi is active
```

### RViz doesn't display

```bash
# Check DISPLAY variable
echo $DISPLAY

# Re-enable X11
xhost +local:docker

# For VNC, ensure correct display number in .env
DISPLAY=:15100
```

### Image/camera errors

Set `rgb_cameras: False` in `config/spot_config.yaml` if you have greyscale cameras. Leave this as false for the RICELab spot since we have grayscale cameras.

### Permission denied (Docker)

```bash
sudo usermod -aG docker $USER
newgrp docker
```

### Container crashes

```bash
# Check logs
docker-compose logs

# Verify config syntax
cat config/spot_config.yaml

# Ensure credentials are correct
```

## Updating spot_ros2

```bash
cd spot_ros2
git pull origin main
cd ..
git add spot_ros2
git commit -m "Update spot_ros2 submodule"
```

## Resources

- [Spot ROS2 Driver](https://github.com/bdaiinstitute/spot_ros2)
- [Boston Dynamics Spot Documentation](https://dev.bostondynamics.com/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

---

**License:** Follows upstream spot_ros2 driver license
