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

# Nav + SLAM
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-rmw-cyclonedds-cpp \
    ros-kilted-navigation2 \
    ros-kilted-nav2-bringup \
    ros-kilted-slam-toolbox \
    ros-kilted-cartographer-ros \
    ros-kilted-robot-localization \
    ros-kilted-xacro \
    ros-kilted-joint-state-publisher \
    ros-kilted-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Ionic + ROS integration
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Python deps for summon server
RUN pip3 install --break-system-packages --ignore-installed fastapi uvicorn pydantic

# Environment
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 base in every shell
RUN echo "source /opt/ros/kilted/setup.bash" >> /etc/bash.bashrc

# Source workspace if built (safe no-op if not built yet)
RUN echo "if [ -f /workspaces/warehousebot/install/setup.bash ]; then source /workspaces/warehousebot/install/setup.bash; fi" >> /etc/bash.bashrc

WORKDIR /workspaces/warehousebot
