import threading
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from summon_msgs.msg import SummonGoal, SummonStatus

# --- FastAPI Models ---

class StaticGoalRequest(BaseModel):
    x: float
    y: float
    ble_mac: Optional[str] = ""

class DynamicGoalRequest(BaseModel):
    wifi_fingerprint: Dict[str, float]
    ble_mac: str

# --- ROS 2 Node ---

class SummonServerNode(Node):
    def __init__(self):
        super().__init__('summon_server_node')
        
        # Publishers
        self.goal_pub = self.create_publisher(SummonGoal, '/summon/goal', 10)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            SummonStatus, '/summon/status', self.status_callback, 10)
        
        self.current_status = "UNKNOWN"
        self.distance_remaining = -1.0

    def status_callback(self, msg: SummonStatus):
        self.current_status = msg.state
        self.distance_remaining = msg.distance_remaining

    def publish_goal(self, x: float, y: float, ble_mac: str, mode: str):
        msg = SummonGoal()
        msg.x = x
        msg.y = y
        msg.ble_mac = ble_mac
        msg.mode = mode
        self.goal_pub.publish(msg)
        self.get_logger().info(f"Published {mode} goal to ({x}, {y})")

# --- Global State ---

app = FastAPI(title="AutoWarehouseBot Summon API")
ros_node: Optional[SummonServerNode] = None

@app.on_event("startup")
def startup_event():
    global ros_node
    # Note: rclpy.init should have been called before starting the thread
    ros_node = SummonServerNode()
    
    # Run ROS 2 spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    thread.start()

@app.on_event("shutdown")
def shutdown_event():
    if ros_node:
        ros_node.destroy_node()
    rclpy.shutdown()

# --- API Endpoints ---

@app.post("/summon/static")
async def summon_static(req: StaticGoalRequest):
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 2 node not initialized")
    
    ros_node.publish_goal(req.x, req.y, req.ble_mac, "static")
    return {"message": "Static goal accepted", "target": {"x": req.x, "y": req.y}}

@app.post("/summon/dynamic")
async def summon_dynamic(req: DynamicGoalRequest):
    if not ros_node:
        raise HTTPException(status_code=503, detail="ROS 2 node not initialized")
    
    # TODO: Implement WiFi fingerprint lookup logic (Phase 4.5)
    # For now, return a stub coordinate (Step 3/6 of plan)
    stub_x = 2.0
    stub_y = 1.0
    
    ros_node.get_logger().info(f"Received dynamic summon request for {req.ble_mac}")
    ros_node.publish_goal(stub_x, stub_y, req.ble_mac, "dynamic")
    
    return {
        "message": "Dynamic goal accepted (WiFi lookup stub applied)",
        "estimated_target": {"x": stub_x, "y": stub_y}
    }

@app.get("/summon/status")
async def get_status():
    if not ros_node:
        return {"state": "OFFLINE", "distance_remaining": -1.0}
    
    return {
        "state": ros_node.current_status,
        "distance_remaining": ros_node.distance_remaining
    }

def main():
    rclpy.init()
    uvicorn.run(app, host="0.0.0.0", port=8080)

if __name__ == "__main__":
    main()
