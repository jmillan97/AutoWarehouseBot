#!/usr/bin/env python3
"""
zenoh_relay.py
==============
Bridges specific ROS2 topics between the laptop (WSL) and the Raspberry Pi
through a Zenoh router running on the Windows host machine.

Does NOT touch RMW, FastDDS, or any DDS configuration. Each machine runs its
own independent ROS2 graph. This node sits alongside that graph and shuttles
serialized messages through Zenoh over TCP.

                  WSL (ROS2)                Pi (ROS2)
                 [this node]              [this node]
                      |                       |
               TCP to Windows           TCP to Windows
                      |                       |
                    [ zenohd — Windows host ]

Topic routing
-------------
  /cmd_vel   (TwistStamped) : laptop → pi
  /odom      (Odometry)     : pi     → laptop
  /scan      (LaserScan)    : pi     → laptop
  /imu/data  (Imu)          : pi     → laptop

To add a topic, append an entry to BRIDGE_MAP at the bottom of this file.

Network health
--------------
Zenoh liveliness tokens are used for peer drop detection. When the other
side's relay node exits (cleanly or via crash), the liveliness subscriber
fires a DELETE event within one keep-alive interval (~1 s). No heartbeat
topic is needed.

A watchdog thread independently checks the Zenoh session every 5 s and
logs a warning if the router is unreachable, so dead-router failures are
also surfaced.

Usage (node is launched by zenoh_relay.launch.py — do not invoke directly)
"""

import threading
import time
import argparse

import zenoh
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu


# ── Bridge map ────────────────────────────────────────────────────────────────
# Each entry: (zenoh_key, ros_topic, ros_msg_type, source_role)
# source_role = which machine *publishes* on this ROS2 topic.
#   The source subscribes to ROS2 and publishes to Zenoh.
#   The sink  subscribes to Zenoh  and publishes to ROS2.
BRIDGE_MAP = [
    ("warehouse/cmd_vel", "/cmd_vel",   TwistStamped, "laptop"),
    ("warehouse/odom",    "/odom",      Odometry,     "pi"),
    ("warehouse/scan",    "/scan",      LaserScan,    "pi"),
    ("warehouse/imu",     "/imu/data",  Imu,          "pi"),
]

# Liveliness key space — one token per relay instance.
LIVELINESS_KEY = "warehouse/relay/{role}"

# How long (s) the watchdog sleeps between router health checks.
WATCHDOG_INTERVAL = 5.0


class ZenohRelay(Node):
    """
    ROS2 node that bridges topics to/from a Zenoh session.

    Parameters (set via launch file, not constructor args):
        role        : "laptop" or "pi"
        router_ip   : IP of the Windows host running zenohd (default 192.168.1.100)
        router_port : Zenoh router port (default 7447)
    """

    def __init__(self):
        super().__init__("zenoh_relay")

        # ── Declare ROS2 parameters ───────────────────────────────────────────
        self.declare_parameter("role",        "laptop")
        self.declare_parameter("router_ip",   "192.168.1.100")
        self.declare_parameter("router_port", 7447)

        self._role   = self.get_parameter("role").get_parameter_value().string_value
        router_ip    = self.get_parameter("router_ip").get_parameter_value().string_value
        router_port  = self.get_parameter("router_port").get_parameter_value().integer_value
        self._router = f"tcp/{router_ip}:{router_port}"

        if self._role not in ("laptop", "pi"):
            raise ValueError(f"role must be 'laptop' or 'pi', got '{self._role}'")

        self.get_logger().info(
            f"Zenoh relay starting — role={self._role}, router={self._router}"
        )

        # ── Open Zenoh session ────────────────────────────────────────────────
        self._z = self._open_session()

        # ── Declare liveliness token (dropped automatically on session close) ─
        self._liveliness_token = self._z.liveliness().declare_token(
            LIVELINESS_KEY.format(role=self._role)
        )

        # ── Subscribe to peer liveliness ──────────────────────────────────────
        peer_role = "pi" if self._role == "laptop" else "laptop"
        self._peer_liveliness_sub = self._z.liveliness().declare_subscriber(
            LIVELINESS_KEY.format(role=peer_role),
            self._on_peer_liveliness,
            history=True,   # fire immediately if peer is already alive
        )
        self._peer_alive = False

        # ── Wire up topic bridges ─────────────────────────────────────────────
        self._zenoh_pubs:  dict[str, zenoh.Publisher]  = {}
        self._ros_pubs:    dict[str, rclpy.publisher.Publisher] = {}
        self._zenoh_subs:  list = []

        for key, topic, msg_type, source in BRIDGE_MAP:
            if self._role == source:
                self._setup_ros_to_zenoh(key, topic, msg_type)
            else:
                self._setup_zenoh_to_ros(key, topic, msg_type)

        # ── Watchdog thread ───────────────────────────────────────────────────
        self._running = True
        self._watchdog_thread = threading.Thread(
            target=self._watchdog, daemon=True, name="zenoh_watchdog"
        )
        self._watchdog_thread.start()

        self.get_logger().info("Zenoh relay ready.")

    # ── Session management ────────────────────────────────────────────────────

    def _open_session(self) -> zenoh.Session:
        """Open a Zenoh client session pointed at the Windows router."""
        conf = zenoh.Config()
        conf.insert_json5("mode", '"client"')
        conf.insert_json5("connect/endpoints", f'["{self._router}"]')
        # Keep-alive so liveliness events fire within ~1 s of a disconnect.
        conf.insert_json5("transport/unicast/lowlatency", "true")
        try:
            session = zenoh.open(conf)
            self.get_logger().info(f"Connected to Zenoh router at {self._router}")
            return session
        except Exception as e:
            self.get_logger().error(
                f"Failed to connect to Zenoh router at {self._router}: {e}\n"
                "Is zenohd.exe running on the Windows host?"
            )
            raise

    # ── Bridge setup ──────────────────────────────────────────────────────────

    def _setup_ros_to_zenoh(self, key: str, topic: str, msg_type):
        """Subscribe to a local ROS2 topic; forward each message to Zenoh."""
        zp = self._z.declare_publisher(key)
        self._zenoh_pubs[key] = zp

        def _cb(msg, _key=key, _zp=zp):
            try:
                _zp.put(serialize_message(msg))
            except Exception as e:
                self.get_logger().warn(f"Zenoh publish failed on {_key}: {e}")

        self.create_subscription(msg_type, topic, _cb, 10)
        self.get_logger().debug(f"  ROS2→Zenoh  {topic}  →  {key}")

    def _setup_zenoh_to_ros(self, key: str, topic: str, msg_type):
        """Subscribe to a Zenoh key; republish each message on a local ROS2 topic."""
        pub = self.create_publisher(msg_type, topic, 10)
        self._ros_pubs[key] = pub

        def _cb(sample, _key=key, _pub=pub, _type=msg_type):
            try:
                msg = deserialize_message(bytes(sample.payload), _type)
                _pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"ROS2 publish failed on {_key}: {e}")

        sub = self._z.declare_subscriber(key, _cb)
        self._zenoh_subs.append(sub)
        self.get_logger().debug(f"  Zenoh→ROS2  {key}  →  {topic}")

    # ── Liveliness callback ───────────────────────────────────────────────────

    def _on_peer_liveliness(self, event):
        peer = "pi" if self._role == "laptop" else "laptop"
        if event.kind == zenoh.SampleKind.PUT:
            if not self._peer_alive:
                self.get_logger().info(f"Peer relay ({peer}) is UP — bridge active.")
            self._peer_alive = True
        elif event.kind == zenoh.SampleKind.DELETE:
            self.get_logger().warn(
                f"Peer relay ({peer}) went DOWN — topics will not bridge until it reconnects."
            )
            self._peer_alive = False

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _watchdog(self):
        """
        Periodically verify the Zenoh session is healthy.
        Logs a clear error if the router disappears so the user knows
        immediately rather than seeing silent message drops.
        """
        router_was_ok = True
        while self._running:
            time.sleep(WATCHDOG_INTERVAL)
            try:
                # session.info() raises if the session is closed/dead.
                info = self._z.info()
                if not router_was_ok:
                    self.get_logger().info(
                        f"Zenoh router at {self._router} is reachable again."
                    )
                router_was_ok = True
                _ = info  # suppress unused warning
            except Exception:
                if router_was_ok:
                    self.get_logger().error(
                        f"Zenoh router at {self._router} is UNREACHABLE. "
                        "Check that zenohd.exe is running on the Windows host "
                        "and that the firewall allows TCP port 7447."
                    )
                router_was_ok = False

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        try:
            self._z.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ZenohRelay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[zenoh_relay] Fatal: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
