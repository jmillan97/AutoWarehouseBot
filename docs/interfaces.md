# Interface Contracts
**Warehouse Delivery Bot — ECE 441 Capstone Project**

---

## ROS2 Distribution
- **Distro:** ROS2 Kilted
- **OS:** Ubuntu 24.04 LTS
- **`ROS_DOMAIN_ID`:** `42` (set in every machine's `.bashrc` and Docker env)
- **DDS middleware:** FastRTPS (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`)
- **Discovery server:** `100.114.38.65:11811` (Felix's laptop — must be running first)
- **Sim time:** `use_sim_time: True` in sim, `False` on physical robot

---

## Hardware Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     OFFBOARD PC(s)                       │
│              docker compose --profile compute up         │
│                                                         │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────────┐  │
│  │  Nav2 Stack │  │  wb_summon   │  │ wb_perception │  │
│  │  A* + MPPI  │  │  FastAPI     │  │  ArUco node   │  │
│  │  AMCL/SLAM  │  │  summon_node │  │  YOLO node    │  │
│  └──────┬──────┘  └──────┬───────┘  └───────┬───────┘  │
│         │                │                   │          │
└─────────┼────────────────┼───────────────────┼──────────┘
          │   ROS2 over Tailscale VPN (FastDDS) │
          │   ROS_DOMAIN_ID=42                  │
┌─────────┼────────────────┼───────────────────┼──────────┐
│         │   RASPBERRY PI │                   │          │
│    docker compose --profile robot up         │          │
│         │                │                   │          │
│  ┌──────▼──────┐  ┌──────▼───────┐  ┌───────▼───────┐  │
│  │ wb_embedded │  │  ble_tracker │  │  camera driver│  │
│  │  LiDAR drv  │  │  (BLE scan)  │  │  /image_raw   │  │
│  │  IMU driver │  └──────────────┘  └───────────────┘  │
│  │  motor brdg │                                        │
│  │  EKF node   │                                        │
│  └──────┬──────┘                                        │
│         │  USB Serial                                   │
└─────────┼─────────────────────────────────────────────-─┘
          │
┌─────────▼──────────┐
│    ARDUINO UNO     │
│  Motor PID + PWM   │
│  Encoder interrupts│
└────────────────────┘
```

---

## What Runs Where — Quick Reference

### Raspberry Pi (`docker compose --profile robot up`)

| Node / Process | Package | Topic(s) Published | Topic(s) Subscribed |
|---|---|---|---|
| LiDAR driver | `wb_embedded` | `/scan` | — |
| IMU driver | `wb_embedded` | `/imu/data` | — |
| Camera driver | `wb_embedded` | `/camera/image_raw` | — |
| Motor bridge | `wb_embedded` | — | `/cmd_vel`, `/summon/motion_cmd` |
| Encoder odometry | `wb_embedded` | `/odom` | — |
| robot_state_publisher | `wb_description` | `/tf` (static) | `/joint_states` |
| robot_localization EKF | `wb_embedded` | `/odom` (fused) | `/odom` (raw), `/imu/data` |
| BLE tracker | `wb_summon` | `/ble/target` | `/summon/ble_target` |

**Hardware connected to Pi:**
- Arduino Uno via USB serial (`/dev/ttyUSB0` or `/dev/ttyACM0`)
- 2D LiDAR (model TBD)
- IMU (model TBD — confirm if connected)
- RGB Camera (USB webcam or Pi Camera Module — TBD)
- Bluetooth adapter (for BLE scanning)

### Offboard PC(s) (`docker compose --profile compute up`)

| Node / Process | Package | Topic(s) Published | Topic(s) Subscribed |
|---|---|---|---|
| Nav2 (A* planner) | `wb_navigation` | `/plan` | `/map`, `/odom`, `/scan` |
| Nav2 (MPPI controller) | `wb_navigation` | `/cmd_vel` | `/plan`, `/odom`, `/scan` |
| AMCL / SLAM | `wb_navigation` | `/map`, `/tf` (map→odom) | `/scan`, `/odom` |
| summon_server (FastAPI) | `wb_summon` | `/summon/goal` | `/summon/status` |
| summon_node | `wb_summon` | `/summon/status`, `/summon/motion_cmd`, `/summon/ble_target` | `/summon/goal`, `/odom`, `/ble/target` |
| ArUco node | `wb_summon` | `/initialpose` | `/camera/image_raw` |
| YOLO node | `wb_perception` | `/obstacles` | `/camera/image_raw` |

### Felix's Laptop Only (`docker compose --profile discovery up`)

| Process | Notes |
|---|---|
| FastDDS Discovery Server | Must be running before any other nodes start. Replaces DDS multicast for Tailscale. Command: `fastdds discovery -i 0 -l 100.114.38.65 -p 11811` |

### Simulation Only (`docker compose --profile sim up`)

Replaces the Pi profile entirely — no real hardware needed.

| Node | Replaces |
|---|---|
| Gazebo Ionic + `warehouse_bot.sdf` | Physical environment |
| `fake_laser_scan` | LiDAR driver |
| `fake_imu` | IMU driver |
| `fake_odometry` | Encoder odometry |
| `ros_gz_bridge` | Serial motor bridge |

---

## TF Frame Tree

```
map
 └── odom
      └── base_link
           ├── base_laser          ← 2D LiDAR
           ├── camera_link         ← RGB camera
           └── camera_optical_link
```

| Transform | Publisher | Mode |
|---|---|---|
| `map → odom` | AMCL (localization) or slam_toolbox (mapping) | PC |
| `odom → base_link` | robot_localization EKF | Pi |
| `base_link → sensors` | robot_state_publisher (from URDF) | Pi |
| ArUco pose reset | ArUco node → `/initialpose` → EKF snaps | PC detects, Pi EKF applies |

---

## Topic Contracts

| Topic | Type | Publisher (device) | Subscriber (device) | Rate |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver (Pi) | Nav2 costmap (PC), SLAM (PC) | 10 Hz |
| `/imu/data` | `sensor_msgs/Imu` | IMU driver (Pi) | EKF (Pi) | 50 Hz |
| `/odom` | `nav_msgs/Odometry` | EKF (Pi) | Nav2 (PC), AMCL (PC) | 30 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 MPPI (PC) | Motor bridge (Pi) | 20 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | map_server / SLAM (PC) | Nav2 costmaps (PC) | 1 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` | Camera driver (Pi) | ArUco node (PC), YOLO (PC) | 15 Hz |
| `/obstacles` | `sensor_msgs/PointCloud2` | YOLO node (PC) | Nav2 local costmap (PC) | 10 Hz |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | ArUco node (PC) | EKF (Pi) | on detection |
| `/summon/goal` | `summon_msgs/SummonGoal` | summon_server (PC) | summon_node (PC) | on demand |
| `/summon/status` | `summon_msgs/SummonStatus` | summon_node (PC) | summon_server (PC) | 2 Hz |
| `/summon/motion_cmd` | `std_msgs/String` | summon_node (PC) | Motor bridge (Pi) | on demand |
| `/summon/ble_target` | `std_msgs/String` | summon_node (PC) | ble_tracker (Pi) | on demand |
| `/ble/target` | `std_msgs/String` (JSON) | ble_tracker (Pi) | summon_node (PC) | 5 Hz |

---

## Motion Command Interface (`/summon/motion_cmd`)

Simple string commands published by `summon_node` and executed by the Pi's motor bridge. Used for BLE homing fine approach and the Archimedean spiral search.

```
"forward <feet>"    → move straight forward N feet        e.g. "forward 2"
"backward <feet>"   → move straight backward N feet       e.g. "backward 0.5"
"rotate <degrees>"  → rotate in place                     e.g. "rotate 45" (right), "rotate -45" (left)
"stop"              → immediate halt
```

---

## ArUco Relocalization Interface

ArUco markers are physical square fiducial markers printed and mounted at known locations in the environment. The camera on the Pi streams `/camera/image_raw` to the PC. The ArUco node on the PC detects markers and publishes an absolute pose correction.

**Physical requirements:**
- Markers: ArUco 6x6, dictionary ID 250 (recommended for robustness)
- Placement: wall-mounted at camera height, or ceiling-mounted (more reliable — not blocked by obstacles)
- Minimum: 1 marker per major navigation zone. More = better drift correction.

**Landmark database** (`ros2_ws/src/summon/config/landmarks.yaml`):
```yaml
landmarks:
  - id: 101              # ArUco marker ID printed on the physical marker
    x: 0.0               # position in SLAM map frame (meters)
    y: 0.0
    yaw: 0.0             # facing direction of the marker (degrees)
    description: "entrance"
  - id: 102
    x: 5.0
    y: 0.0
    yaw: 180.0
    description: "hallway midpoint"
```

**How the pose reset works:**
1. Camera sees marker → ArUco node computes robot's position relative to marker
2. Looks up marker's global position in `landmarks.yaml`
3. Computes robot's absolute global pose
4. Publishes to `/initialpose` with covariance `1e-9` (tells EKF: this is ground truth)
5. EKF immediately snaps — odometry drift is zeroed out

**Prerequisite**: Camera must be intrinsically calibrated first (`camera_info.yaml`). Without calibration, distance estimates to the marker will be wrong and the pose reset will be inaccurate.

---

## Action Interfaces

| Action | Type | Server (device) | Client (device) |
|---|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 bt_navigator (PC) | summon_node (PC) |

**The summon node calls `NavigateToPose` directly. Do not create a custom action.**

---

## Summon Algorithm Invariants

These rules are always active — see `planning/implementation_plan_*.md` §4 for full pseudocode.

| Rule | Value | Enforced By |
|---|---|---|
| Minimum wall clearance | **0.61m (2 feet)** | Nav2 costmap inflation + `/scan` guard in summon_node |
| Arrival search pattern | **Archimedean spiral** | summon_node SPIRAL_SEARCH state |
| Spiral max radius before FAILED | **4.0m** | summon_node control loop |
| BLE lock threshold (RSSI) | **-65 dBm** (~2m range) | ble_tracker + summon_node |
| Control loop rate | **10 Hz** | summon_node timer |

---

## Physical Constants

| Parameter | Value | Notes |
|---|---|---|
| `robot_radius` | **MEASURE** | chassis diagonal / 2, meters |
| `wheel_separation` | **MEASURE** | center-to-center of drive wheels |
| `wheel_radius` | **MEASURE** | meters |
| `wall_clearance` | **0.61m** | 2 feet — hardcoded invariant |
| `max_vel_x` | 0.5 m/s | start conservative, tune up |
| `min_vel_x` | -0.35 m/s | reverse limit |
| `max_vel_theta` | 1.9 rad/s | rotational limit |

---

## Package Ownership

| Package | Lead | Runs On | Description |
|---|---|---|---|
| `description` | Autonomy lead | Pi | URDF, meshes, robot_state_publisher |
| `embedded` | Embedded lead | Pi | Motor bridge, sensor drivers, EKF, encoder odom |
| `perception` | Perception lead | PC | YOLO obstacle node, ArUco node, camera processing |
| `navigation` | Autonomy lead | PC | Nav2 config, MPPI params, SLAM, launch files |
| `summon` | Felix | PC + Pi | FastAPI server, summon node, BLE tracker, WiFi fingerprint |

---

## Network Configuration

| Device | Tailscale IP | Role |
|---|---|---|
| Felix's laptop | `100.114.38.65` | FastDDS discovery server host, primary compute |
| Raspberry Pi | `100.91.37.52` | Robot hardware interface |
| Groupmate laptops | assigned by Tailscale | Additional compute nodes |

**How to join the ROS2 network (any device):**
1. Install Tailscale, join the shared network
2. Install Docker
3. Clone the repo
4. `docker compose --profile compute up` — all env vars are set automatically

**DDS:** FastRTPS with centralized discovery server. Replaces multicast — works over Tailscale VPN across any network.

---

## Change Log

| Date | Change | Author |
|---|---|---|
| 03.02.2026 | Document created | Juan |
| 03.02.2026 | Updated Linux Distro | Juan |
| 04.09.2026 | Major rewrite: added hardware diagram, PC/Pi split tables, ArUco interface, summon algorithm invariants, network config, motion command interface | Felix |
