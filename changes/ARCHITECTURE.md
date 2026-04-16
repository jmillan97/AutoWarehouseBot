# Warehouse Bot — Network Architecture & Zenoh Relay

## Overview

The robot system is split across three machines that all need to share ROS2 topics
in real-time. Each machine runs its own independent ROS2 graph (FastDDS, unchanged).
The Zenoh relay layer sits alongside those graphs and moves serialized messages
between them over plain TCP — no changes to RMW, DDS config, or any existing launch file.

---

## Physical topology

```
┌─────────────────────────────────────────────────┐
│                  WiFi LAN                        │
│                                                  │
│  ┌──────────────┐        ┌──────────────────┐   │
│  │  Windows PC  │        │  Raspberry Pi    │   │
│  │              │        │  (native ROS2)   │   │
│  │  zenohd.exe  │◄──TCP──│  zenoh_relay     │   │
│  │  (router)    │        │  robot_bringup   │   │
│  │              │        └──────────────────┘   │
│  │  ▲           │                               │
│  │  │ TCP (WSL  │                               │
│  │  │  → host)  │                               │
│  │  │           │                               │
│  │  WSL2        │                               │
│  │  Ubuntu      │                               │
│  │  ROS2        │                               │
│  │  zenoh_relay │                               │
│  │  Nav2        │                               │
│  └──────────────┘                               │
└─────────────────────────────────────────────────┘
```

**Why Windows as the router?**
WSL2 sits behind a virtual NAT — it cannot receive inbound connections from the Pi.
By running `zenohd` on Windows, both WSL and the Pi *dial out* to the same endpoint.
No inbound firewall rules, no static IP on WSL.

---

## Software layers

### Layer 1 — ROS2 (unchanged on every machine)

Each machine runs its own ROS2 domain with `ROS_DOMAIN_ID=42`.
FastDDS handles local-only pub/sub (nodes on the same machine discover each other normally).
No cross-machine DDS multicast is attempted.

| Machine  | ROS2 nodes running |
|----------|--------------------|
| Pi       | rplidar, serial_bridge, wheel_odometry, robot_state_publisher, EKF, usb_cam, imu_node |
| WSL      | robot_state_publisher, EKF, Nav2 (AMCL, planner, MPPI) |

### Layer 2 — Zenoh relay (new, non-intrusive)

`zenoh_relay.py` is a normal ROS2 node in the `embedded` package.
It subscribes to local ROS2 topics and re-publishes them to the Zenoh router,
and vice versa. Message bytes are CDR-serialized (same format ROS2 uses internally)
so no conversion or type re-mapping is needed.

```
Pi ROS2 graph                   Zenoh layer                   WSL ROS2 graph
─────────────────               ─────────────                 ──────────────────────
/scan  ──────────► [relay] ──► warehouse/scan  ──► [relay] ──► /scan
/odom  ──────────► [relay] ──► warehouse/odom  ──► [relay] ──► /odom
/imu/data ───────► [relay] ──► warehouse/imu   ──► [relay] ──► /imu/data
                                warehouse/cmd_vel ◄─ [relay] ◄── /cmd_vel
```

### Layer 3 — zenohd (Windows, no ROS2)

`zenohd.exe` is a pure message router. It does not understand ROS2.
It receives byte payloads on named keys and fans them out to all subscribers.
Think of it as a dumb broker — it only knows about Zenoh keys, not topics or types.

---

## Topic routing table

| ROS2 topic  | Zenoh key          | Type         | Direction      |
|-------------|--------------------|--------------|----------------|
| `/cmd_vel`  | `warehouse/cmd_vel`| TwistStamped | WSL → Pi       |
| `/odom`     | `warehouse/odom`   | Odometry     | Pi  → WSL      |
| `/scan`     | `warehouse/scan`   | LaserScan    | Pi  → WSL      |
| `/imu/data` | `warehouse/imu`    | Imu          | Pi  → WSL      |

To add a topic: append one tuple to `BRIDGE_MAP` in `zenoh_relay.py`.

---

## Network health & failure detection

### Liveliness tokens (peer drop)

Each relay declares a Zenoh liveliness token under `warehouse/relay/{role}`.
Zenoh revokes the token automatically when the session closes (crash, clean exit, or
network loss). The peer relay receives a `DELETE` event within ~1 second and logs:

```
[WARN] Peer relay (pi) went DOWN — topics will not bridge until it reconnects.
```

No heartbeat topic is published. The session itself is the heartbeat.

### Watchdog thread (router drop)

A background thread in each relay calls `session.info()` every 5 seconds.
If the router (`zenohd.exe`) stops responding:

```
[ERROR] Zenoh router at tcp/192.168.1.100:7447 is UNREACHABLE.
        Check that zenohd.exe is running on the Windows host
        and that the firewall allows TCP port 7447.
```

### Recovery

Both failure modes are logged clearly but the relay does **not** auto-restart
(to avoid silent loops). The Windows launcher script (`launch_warehouse.py`)
watches child processes and reports exits so you know which component died.

---

## Prerequisites

| Machine | Requirement |
|---------|-------------|
| Windows | `zenohd.exe` (download from github.com/eclipse-zenoh/zenoh/releases) |
| Windows | Python 3.10+, `paramiko` (`pip install paramiko`) — for the launcher script |
| WSL     | `pip install eclipse-zenoh` |
| Pi      | `pip install eclipse-zenoh` |
| Pi      | SSH server running (`sudo systemctl enable ssh`) |

---

## File map

```
warehouse project/
├── changes/
│   ├── ARCHITECTURE.md          ← this file
│   └── 2026-04-16.md            ← first zenoh relay changelog
├── launch_warehouse.py          ← Windows launcher (starts everything)
└── ros2_ws/src/embedded/
    ├── src/zenoh_relay.py       ← relay node
    └── launch/zenoh_relay.launch.py
```
