FROM ubuntu:22.04

ARG EXPERIMENTAL_ZENOH_RMW=FALSE

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gettext-base \
    git \
    locales \
    sudo \
    wget \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    libpython3-dev \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-opencv \
    python3-pil \
    python3-pip \
    python3-rosdep \
    python3-tk \
    qttools5-dev \
    ros-dev-tools \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ros-base \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-rviz2 \
    $(if [ "$EXPERIMENTAL_ZENOH_RMW" = "TRUE" ]; then echo "ros-humble-rmw-zenoh-cpp"; fi) \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws/src
COPY spot_ros2/ /ros_ws/src/

RUN if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi
RUN ARCH="$(dpkg --print-architecture)" \
    && echo "Building spot_ros2 for ${ARCH}" \
    && /ros_ws/src/install_spot_ros2.sh --"${ARCH}"
RUN python3 -m pip install --no-cache-dir \
    bosdyn-api==5.0.1 \
    bosdyn-choreography-client==5.0.1 \
    bosdyn-client==5.0.1 \
    bosdyn-core==5.0.1 \
    bosdyn-mission==5.0.1 \
    && python3 -c "import bosdyn.api, bosdyn.client, bosdyn.mission"
RUN python3 -m pip install --no-cache-dir \
    "aiortc>=1.9.0" \
    && python3 -c "import aiortc, cv2, PIL.Image"

WORKDIR /ros_ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

COPY tools/render_cyclonedds_config.sh /usr/local/bin/render_cyclonedds_config
COPY tools/spot_driver_entrypoint.sh /usr/local/bin/spot_driver_entrypoint
RUN chmod +x /usr/local/bin/render_cyclonedds_config /usr/local/bin/spot_driver_entrypoint

ENTRYPOINT ["/usr/local/bin/spot_driver_entrypoint"]
CMD ["ros2", "launch", "spot_driver", "spot_driver.launch.py", "config_file:=/tmp/spot_config.generated.yaml", "launch_rviz:=False"]
