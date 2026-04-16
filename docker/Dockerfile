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

RUN rosdep init || true && rosdep update

# Nav + SLAM
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-rmw-fastrtps-cpp \
    ros-kilted-navigation2 \
    ros-kilted-nav2-bringup \
    ros-kilted-slam-toolbox \
    ros-kilted-cartographer-ros \
    ros-kilted-robot-localization \
    ros-kilted-foxglove-bridge \
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

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN echo "source /opt/ros/kilted/setup.bash" >> /etc/bash.bashrc
RUN echo "if [ -f /workspaces/AutoWarehouseBot/ros2_ws/install/setup.bash ]; then source /workspaces/AutoWarehouseBot/ros2_ws/install/setup.bash; fi" >> /etc/bash.bashrc

# Also source for ubuntu user (compose runs as ubuntu not root)
RUN echo "source /opt/ros/kilted/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "if [ -f /workspaces/AutoWarehouseBot/ros2_ws/install/setup.bash ]; then source /workspaces/AutoWarehouseBot/ros2_ws/install/setup.bash; fi" >> /home/ubuntu/.bashrc && \
    echo "export ROS_DOMAIN_ID=42" >> /home/ubuntu/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /home/ubuntu/.bashrc

WORKDIR /workspaces/AutoWarehouseBot