"""
relay_server.py — Pi-side WebSocket relay server
=================================================
Runs on the Raspberry Pi. Accepts a WebSocket connection from the laptop,
then relays ROS2 topics bidirectionally using CDR serialization.

Wire protocol (per message):
  Frame 1 (text):   JSON header {"topic": "/scan", "type": "sensor_msgs/msg/LaserScan"}
  Frame 2 (binary): CDR-serialized message bytes
"""

import json
import asyncio
import threading
from importlib import import_module

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message, deserialize_message

import websockets
from websockets.asyncio.server import serve

import yaml
from ament_index_python.packages import get_package_share_directory
import os


def load_topic_config():
    config_path = os.path.join(
        get_package_share_directory('tailscale_relay'),
        'config', 'relay_topics.yaml'
    )
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def resolve_msg_type(type_str):
    """Convert 'sensor_msgs/msg/LaserScan' → the actual Python class."""
    parts = type_str.split('/')
    module = import_module(f'{parts[0]}.{parts[1]}')
    return getattr(module, parts[2])


class RelayServer(Node):
    def __init__(self):
        super().__init__('relay_server')
        self.declare_parameter('port', 8765)
        self.port = self.get_parameter('port').value

        self.config = load_topic_config()
        self.clients = set()
        self.subscriptions_list = []
        self.publishers_dict = {}
        self.send_queue = asyncio.Queue()

        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.get_logger().info(f'Relay server configured on port {self.port}')

    def setup_subscriptions(self):
        """Subscribe to Pi-local topics and queue them for sending to laptop."""
        for entry in self.config.get('pi_to_laptop', []):
            topic = entry['topic']
            msg_type = resolve_msg_type(entry['type'])

            def make_callback(t, tp):
                def cb(msg):
                    data = serialize_message(msg)
                    try:
                        self.send_queue.put_nowait((t, tp, data))
                    except asyncio.QueueFull:
                        pass
                return cb

            sub = self.create_subscription(
                msg_type, topic,
                make_callback(topic, entry['type']),
                self.sensor_qos,
            )
            self.subscriptions_list.append(sub)
            self.get_logger().info(f'Subscribed to {topic} (→ laptop)')

    def setup_publishers(self):
        """Create publishers for topics coming FROM the laptop."""
        for entry in self.config.get('laptop_to_pi', []):
            topic = entry['topic']
            msg_type = resolve_msg_type(entry['type'])
            pub = self.create_publisher(msg_type, topic, self.cmd_qos)
            self.publishers_dict[topic] = (pub, msg_type)
            self.get_logger().info(f'Publisher ready for {topic} (← laptop)')

    async def handle_client(self, websocket):
        """Handle a single WebSocket client connection."""
        remote = websocket.remote_address
        self.get_logger().info(f'Client connected: {remote}')
        self.clients.add(websocket)

        try:
            async for raw in websocket:
                # Each incoming message is a text frame (JSON header)
                # followed by a binary frame (CDR payload).
                # websockets library delivers them as separate messages.
                if isinstance(raw, str):
                    self._pending_header = json.loads(raw)
                elif isinstance(raw, bytes):
                    header = getattr(self, '_pending_header', None)
                    if header is None:
                        continue
                    topic = header['topic']
                    if topic in self.publishers_dict:
                        pub, msg_type = self.publishers_dict[topic]
                        msg = deserialize_message(raw, msg_type)
                        pub.publish(msg)
                    self._pending_header = None
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            self.get_logger().info(f'Client disconnected: {remote}')

    async def broadcast_loop(self):
        """Pull from send_queue and broadcast to all connected clients."""
        while True:
            topic, type_str, data = await self.send_queue.get()
            header = json.dumps({'topic': topic, 'type': type_str})
            dead = set()
            for ws in self.clients:
                try:
                    await ws.send(header)
                    await ws.send(data)
                except websockets.exceptions.ConnectionClosed:
                    dead.add(ws)
            self.clients -= dead

    async def run(self):
        self.setup_subscriptions()
        self.setup_publishers()

        async with serve(self.handle_client, '0.0.0.0', self.port):
            self.get_logger().info(f'WebSocket server listening on 0.0.0.0:{self.port}')
            await self.broadcast_loop()


def spin_ros(node):
    """Spin rclpy in a background thread."""
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = RelayServer()

    # Spin ROS in a background thread so the asyncio event loop can run
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
