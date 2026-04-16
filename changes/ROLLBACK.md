# Rollback Guide — Reverting Zenoh Relay

How to completely remove the Zenoh relay layer and go back to the previous
state (FastDDS-only, manual per-machine launch).

---

## What was added (and how to undo each piece)

### 1. New files — safe to delete entirely

```
ros2_ws/src/embedded/src/zenoh_relay.py
ros2_ws/src/embedded/launch/zenoh_relay.launch.py
launch_warehouse.py
changes/                         ← this whole folder if you want
```

```bash
# From the repo root:
rm ros2_ws/src/embedded/src/zenoh_relay.py
rm ros2_ws/src/embedded/launch/zenoh_relay.launch.py
rm launch_warehouse.py
```

### 2. CMakeLists.txt — remove the zenoh_relay install block

In `ros2_ws/src/embedded/CMakeLists.txt`, remove these 5 lines:

```cmake
install(PROGRAMS
  src/zenoh_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME zenoh_relay
)
```

### 3. Rebuild after removing

```bash
# WSL
cd ~/ros2_ws
source /opt/ros/kilted/setup.bash
colcon build --packages-select embedded
source install/setup.bash

# Pi
cd ~/AutoWarehouseBot
source /opt/ros/kilted/setup.bash
colcon build --packages-select embedded
source install/setup.bash
```

### 4. WSL Python venv — remove eclipse-zenoh

The `eclipse-zenoh` package was installed into a venv at `~/zenoh_venv`
(created with `--system-site-packages` so rclpy remains accessible).
To remove it entirely:

```bash
rm -rf ~/zenoh_venv
```

Nothing outside that directory was modified by the venv.

### 5. WSL native workspace (if you want to clean it up)

The WSL native workspace at `~/ros2_ws` was created only for performance.
The source of truth is still the Windows path. To remove:

```bash
rm -rf ~/ros2_ws
```

You can then go back to building from the mounted path (slow but works):
```bash
cd '/mnt/c/Users/felix/OneDrive/Desktop/warehouse project/ros2_ws'
source /opt/ros/kilted/setup.bash
colcon build --packages-select embedded
```

### 5. Windows — stop and delete zenohd

Just close the `zenohd.exe` terminal or kill the process. Nothing is installed
system-wide — it's a single portable binary. Delete `C:\tools\zenoh\` to clean up.

### 6. pip packages — remove eclipse-zenoh (optional)

The `eclipse-zenoh` Python package is only used by the relay node.
If nothing else in your environment uses it:

```bash
# WSL
pip uninstall eclipse-zenoh -y

# Pi
pip uninstall eclipse-zenoh -y
```

---

## Restoring original launch workflow

After rollback, the original launch sequence is:

**Pi:**
```bash
source /opt/ros/kilted/setup.bash
source ~/AutoWarehouseBot/install/setup.bash
ros2 launch embedded robot_bringup.launch.py
```

**WSL (Docker dev container):**
```bash
source /opt/ros/kilted/setup.bash
source /workspaces/AutoWarehouseBot/install/setup.bash
ros2 launch navigation hardware.launch.py
```

FastDDS peer discovery uses the hardcoded IPs already in `docker/fastdds_config.xml`.
No other changes are needed.

---

## Partial rollback — keep zenohd but drop the relay node

If zenohd is useful for something else but you want to remove the ROS2
relay node:

1. Delete `zenoh_relay.py` and `zenoh_relay.launch.py`
2. Remove the CMakeLists install block and rebuild
3. Leave `eclipse-zenoh` installed — it does nothing without the node running

---

## Quick verification after rollback

```bash
# On WSL — check that /scan arrives from Pi via FastDDS (not Zenoh):
ros2 topic echo /scan --once

# Should work without zenohd running. If it doesn't, FastDDS peer
# discovery may need the IPs in fastdds_config.xml updated.
```
