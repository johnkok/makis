# syntax=docker/dockerfile:1
# Raspberry Pi 5 (ARM64) — ROS 2 Humble + SLAM + Nav2
# Build on RPi5:  docker build -t makis-ros2 .
# Or cross-build: docker buildx build --platform linux/arm64 -t makis-ros2 .

FROM arm64v8/ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]

# ── System deps ──────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-serial \
    python3-smbus2 \
    python3-colcon-common-extensions \
    i2c-tools \
    usbutils \
    git \
    wget \
    curl \
    build-essential \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-common \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-hls-lfcd-lds-driver \
    ros-${ROS_DISTRO}-v4l2-camera \
    libcamera0 \
    libcamera-dev \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-web-video-server \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# ── Python deps ───────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    smbus2==0.4.3 \
    pyserial==3.5 \
    transforms3d==0.4.1

# ── Build workspace ───────────────────────────────────────────
WORKDIR /ros2_ws
COPY ros2_ws/src ./src

# Build camera_ros from source (not in Humble apt index)
RUN git clone --depth 1 --branch main \
    https://github.com/christianrauch/camera_ros.git \
    /ros2_ws/src/camera_ros

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build/

# ── Entrypoint ────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "tank_bringup", "bringup.launch.py"]
