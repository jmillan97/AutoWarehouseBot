FROM osrf/ros:kilted-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Core dev tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl wget \
    python3-pip python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (safe even if already initialized)
RUN rosdep init || true && rosdep update

# Nav + SLAM toolbox
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-rmw-cyclonedds-cpp \
    ros-kilted-navigation2 \
    ros-kilted-nav2-bringup \
    ros-kilted-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Gazebo (Gazebo Sim + ROS integration).
# Kilted recommends Gazebo "Ionic" as the matching Gazebo release line. :contentReference[oaicite:4]{index=4}
# Package names can vary by distro; ros_gz is the key integration layer.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Common QoL: source ROS automatically
RUN echo "source /opt/ros/kilted/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspaces/warehousebot