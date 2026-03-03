# Interface Contracts
**Warehouse Delivery Bot — ECE 441 Capstone Project**  

---

## ROS2 Distribution
- **Distro:** ROS2 Humble
- **OS:** Ubuntu 22.04 LTS
- **`ROS_DOMAIN_ID`:** `42` (set in every machine's `.bashrc`)
- **Sim time:** `use_sim_time: True` in sim, `False` on physical robot

---

## TF Frame Tree
```
map
 └── odom
      └── base_link
           ├── base_laser      ← 2D LiDAR
           ├── camera_link     ← RGB camera
           └── camera_optical_link
```
- `map → odom`: published by **AMCL** (localization) or **Cartographer** (mapping mode)
- `odom → base_link`: published by **robot_localization EKF** (fuses wheel odom + IMU)
- `base_link → sensors`: published by **robot_state_publisher** from URDF

---

## Topic Contracts

| Topic | Message Type | Publisher | Subscribers | Rate |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | wb_embedded (LiDAR driver) | SLAM, local costmap, global costmap | 10 Hz |
| `/imu/data` | `sensor_msgs/Imu` | wb_embedded (IMU driver) | EKF | 50 Hz |
| `/odom` | `nav_msgs/Odometry` | robot_localization EKF | Nav2 bt_navigator, AMCL | 30 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 MPPI controller | wb_embedded (motor bridge) | 20 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | map_server / Cartographer | Nav2 global costmap | 1 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` | wb_embedded (camera driver) | wb_perception obstacle node | 15 Hz |
| `/obstacles` | `sensor_msgs/PointCloud2` | wb_perception obstacle node | Nav2 local costmap | 10 Hz |
| `/goal_pose` | `geometry_msgs/PoseStamped` | wb_summon server | Nav2 bt_navigator | on demand |
| `/robot_status` | `std_msgs/String` (JSON) | wb_navigation status node | wb_summon REST server | 2 Hz |

---

## Action Interfaces

| Action | Type | Server | Client |
|---|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 bt_navigator | wb_summon server |

**The summon server calls `NavigateToPose` directly. Do not create a custom action.**

---

## Physical Constants

| Parameter | Value | Notes |
|---|---|---|
| `robot_radius` | **MEASURE** | chassis diagonal / 2, meters |
| `wheel_separation` | **MEASURE** | center-to-center of drive wheels |
| `wheel_radius` | **MEASURE** | meters |
| `max_vel_x` | 0.5 m/s | start conservative, tune up |
| `min_vel_x` | -0.35 m/s | reverse limit |
| `max_vel_theta` | 1.9 rad/s | rotational limit |

---

## Package Ownership

| Package | Lead | Description |
|---|---|---|
| `wb_description` | Autonomy lead | URDF, meshes, robot_state_publisher |
| `wb_embedded` | Embedded lead | Arduino bridge, sensor drivers, EKF |
| `wb_perception` | Perception lead | Camera obstacle node, fake sensor stubs |
| `wb_navigation` | Autonomy lead | Nav2 config, MPPI params, launch files |
| `wb_summon` | Software Team | BLE trilateration, REST server, UI |

---

## Compute Distribution

| Node | Runs On | Notes |
|---|---|---|
| Arduino firmware | Arduino UNO | PID, encoder interrupts, serial |
| micro_ros_agent | Raspberry Pi | Bridges Arduino ↔ ROS2 |
| LiDAR driver | Raspberry Pi | rplidar_ros / sllidar_ros2 |
| IMU driver | Raspberry Pi | hardware-specific |
| robot_state_publisher | Raspberry Pi | URDF → TF |
| robot_localization EKF | Raspberry Pi | odom + IMU fusion |
| Cartographer / AMCL | Offboard PC | SLAM and localization |
| Nav2 full stack | Offboard PC | planning, MPPI, BT |
| wb_perception obstacle node | Offboard PC | camera processing |
| wb_summon server | Offboard PC | FastAPI + BLE |

---

## Network Configuration

| Device | Static IP | Notes |
|---|---|---|
| Raspberry Pi |  **find** | set in `/etc/dhcpcd.conf` |
| Offboard PC | **find** | or static DHCP lease on router |
| BLE Anchor 1 | N/A (BLE only) | fixed position, record coordinates |


**ROS2 DDS Discovery:** Both machines must have `ROS_DOMAIN_ID=42` in `.bashrc`.  
For busy networks, set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` for more reliable discovery.

---
## Change Log

| Date | Change | Author |
|---|---|---|
| 03.02.2026 | Document created | Juan |
