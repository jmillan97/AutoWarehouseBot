"""
relay_client.py — Laptop-side WebSocket relay client
=====================================================
Runs on the laptop (WSL). Connects to the Pi's relay_server over
Tailscale, relays ROS2 topics bidirectionally using CDR serialization.

Auto-reconnects with exponential backoff if the connection drops.
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


class RelayClient(Node):
    def __init__(self):
        super().__init__('relay_client')
        self.declare_parameter('pi_address', '100.91.37.52')
        self.declare_parameter('port', 8765)
        self.pi_address = self.get_parameter('pi_address').value
        self.port = self.get_parameter('port').value

        self.config = load_topic_config()
        self.publishers_dict = {}
        self.subscriptions_list = []
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

        self.get_logger().info(
            f'Relay client targeting ws://{self.pi_address}:{self.port}'
        )

    def setup_publishers(self):
        """Create publishers for topics coming FROM the Pi."""
        for entry in self.config.get('pi_to_laptop', []):
            topic = entry['topic']
            msg_type = resolve_msg_type(entry['type'])
            pub = self.create_publisher(msg_type, topic, self.sensor_qos)
            self.publishers_dict[topic] = (pub, msg_type)
            self.get_logger().info(f'Publisher ready for {topic} (← Pi)')

    def setup_subscriptions(self):
        """Subscribe to laptop-local topics and queue them for sending to Pi."""
        for entry in self.config.get('laptop_to_pi', []):
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
                self.cmd_qos,
            )
            self.subscriptions_list.append(sub)
            self.get_logger().info(f'Subscribed to {topic} (→ Pi)')

    async def run(self):
        self.setup_publishers()
        self.setup_subscriptions()

        backoff = 1.0
        max_backoff = 30.0
        uri = f'ws://{self.pi_address}:{self.port}'

        while True:
            try:
                self.get_logger().info(f'Connecting to {uri}...')
                async with websockets.connect(uri, max_size=10 * 1024 * 1024) as ws:
                    self.get_logger().info(f'Connected to relay server at {uri}')
                    backoff = 1.0

                    # Run receive and send concurrently
                    await asyncio.gather(
                        self._receive_loop(ws),
                        self._send_loop(ws),
                    )
            except (
                websockets.exceptions.ConnectionClosed,
                ConnectionRefusedError,
                OSError,
            ) as e:
                self.get_logger().warn(
                    f'Connection lost ({e}). Reconnecting in {backoff:.0f}s...'
                )
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, max_backoff)

    async def _receive_loop(self, ws):
        """Receive messages from Pi and publish locally."""
        pending_header = None
        async for raw in ws:
            if isinstance(raw, str):
                pending_header = json.loads(raw)
            elif isinstance(raw, bytes):
                if pending_header is None:
                    continue
                topic = pending_header['topic']
                if topic in self.publishers_dict:
                    pub, msg_type = self.publishers_dict[topic]
                    msg = deserialize_message(raw, msg_type)
                    pub.publish(msg)
                pending_header = None

    async def _send_loop(self, ws):
        """Pull from send_queue and send to Pi."""
        while True:
            topic, type_str, data = await self.send_queue.get()
            header = json.dumps({'topic': topic, 'type': type_str})
            await ws.send(header)
            await ws.send(data)


def spin_ros(node):
    """Spin rclpy in a background thread."""
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = RelayClient()

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
