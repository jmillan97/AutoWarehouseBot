import asyncio
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from bleak import BleakScanner

class BLETracker(Node):
    def __init__(self):
        super().__init__('ble_tracker')
        
        # Subscribers
        self.target_sub = self.create_subscription(
            String, '/summon/ble_target', self.target_callback, 10)
        
        # Publishers
        self.rssi_pub = self.create_publisher(Float64, '/ble/target/rssi', 10)
        
        self.target_mac = None
        self.scanning = False
        
        self.get_logger().info("BLE Tracker node started. Waiting for target MAC address...")

    def target_callback(self, msg: String):
        self.target_mac = msg.data.upper()
        self.get_logger().info(f"Setting BLE target MAC: {self.target_mac}")
        
        if not self.scanning:
            self.scanning = True
            # Start the async scanning loop in the event loop thread
            # This node assumes it's running in an environment with an event loop (started in main)

    async def scan_loop(self):
        self.get_logger().info("Starting BLE scan loop...")
        while rclpy.ok():
            if self.target_mac and self.scanning:
                try:
                    # Scan for 1 second periodically
                    devices = await BleakScanner.discover(timeout=1.0)
                    found = False
                    for d in devices:
                        if d.address.upper() == self.target_mac:
                            rssi = float(d.rssi)
                            self.rssi_pub.publish(Float64(data=rssi))
                            self.get_logger().info(f"Target {self.target_mac} seen: RSSI={rssi}")
                            found = True
                            break
                    
                    if not found:
                        # Optionally publish a 'not found' value or skip
                        pass
                except Exception as e:
                    self.get_logger().error(f"Scan error: {e}")
            
            await asyncio.sleep(0.5) # Gap between scans

def main(args=None):
    rclpy.init(args=args)
    node = BLETracker()
    
    # We need to run both ROS 2 spin and an asyncio event loop
    loop = asyncio.get_event_loop()
    
    # Run the scan loop as a task
    task = loop.create_task(node.scan_loop())
    
    # Run ROS 2 spin in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
