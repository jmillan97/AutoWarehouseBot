FROM osrf/ros:kilted-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Core dev tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    udev \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# Only what the Pi needs
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-rmw-fastrtps-cpp \
    ros-kilted-foxglove-bridge \
    ros-kilted-xacro \
    ros-kilted-joint-state-publisher \
    ros-kilted-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# Python deps for summon server
RUN pip3 install --break-system-packages --ignore-installed fastapi uvicorn pydantic

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN echo "source /opt/ros/kilted/setup.bash" >> /etc/bash.bashrc
RUN echo "if [ -f /workspaces/AutoWarehouseBot/install/setup.bash ]; then source /workspaces/AutoWarehouseBot/install/setup.bash; fi" >> /etc/bash.bashrc

WORKDIR /workspaces/AutoWarehouseBot