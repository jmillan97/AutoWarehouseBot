# AutoWarehouseBot
Senior Project ECE


## Description
Our team of 5 engineers has taken on the ambitious goal of creating a fully autonomous delivery bot, capable of being dynamically summoned to a node via Wi-Fi / BLE.

## Current Docs

- [Project structure](docs/project-structure.md)
- [Handoff guide](docs/handoff-guide.md)
- [Startup scripts](docs/startup-scripts.md)
- [Firmware protocol](docs/firmware-protocol.md)

## Startup
On a Linux Environment, preferably Ubuntu 24.04 as it is compatible with the ROS-Kilted development environment we've used

# Run Docker
From root folder:
    docker compose build --no-cache
    docker compose up -d
    docker exec -it warehousebot_dev bash
    ros2 doctor --report
if ros2 doctor --report is printing info, docker container has been set correctly

Inside Dev Container:
bash:
    cd /workspaces/warehousebot
    cd ros2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source /opt/ros/kilted/setup.bash
    source install/setup.bash

