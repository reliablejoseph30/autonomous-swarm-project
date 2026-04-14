#!/usr/bin/env python3
"""
swarm_dashboard_bridge.py — Toba
Reads ROS2 topics and serves live data to the dashboard via HTTP.

Run with:
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  python3 swarm_dashboard_bridge.py

Then open swarm_ops_dashboard.html in your browser.
"""

import math
import time
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32

# ── Shared state (updated by ROS2, read by HTTP server) ──────────────────────
state = {
    "uav0": {"x": 0.0, "y": 0.0, "z": 0.0, "rate_hz": 0.0, "last_seen": None},
    "uav1": {"x": 0.0, "y": 0.0, "z": 0.0, "rate_hz": 0.0, "last_seen": None},
    "distance_m": 0.0,
    "comm_status": "WAITING",
    "threat_level": "WAITING",
    "uav0_health": 0.0,
    "uav1_health": 0.0,
    "alerts": [],
    "mission_status": "WAITING FOR DRONES",
    "uptime_s": 0,
    "started_at": time.time(),
}
state_lock = threading.Lock()


# ── HTTP handler — serves JSON state ─────────────────────────────────────────
class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/state":
            with state_lock:
                data = json.dumps(state)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data.encode())
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # suppress request logs


# ── ROS2 node ─────────────────────────────────────────────────────────────────
class DashboardBridge(Node):
    def __init__(self):
        super().__init__('swarm_dashboard_bridge')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Position counters for rate calculation
        self.uav0_count = 0
        self.uav1_count = 0
        self.last_rate_check = time.time()

        # Subscribers
        self.create_subscription(PoseStamped, '/uav0/local_position/pose', self.uav0_cb, qos)
        self.create_subscription(PoseStamped, '/uav1/local_position/pose', self.uav1_cb, qos)
        self.create_subscription(String, '/net/alerts', self.alert_cb, 10)
        self.create_subscription(String, '/cyber/status', self.cyber_status_cb, 10)
        self.create_subscription(Float32, '/comms/uav0/health', self.uav0_health_cb, 10)
        self.create_subscription(Float32, '/comms/uav1/health', self.uav1_health_cb, 10)

        # 2Hz update timer
        self.create_timer(0.5, self.update_state)

        self.get_logger().info('Dashboard bridge started — open swarm_ops_dashboard.html in browser')

    def uav0_cb(self, msg):
        with state_lock:
            state["uav0"]["x"] = round(msg.pose.position.x, 1)
            state["uav0"]["y"] = round(msg.pose.position.y, 1)
            state["uav0"]["z"] = round(msg.pose.position.z, 1)
            state["uav0"]["last_seen"] = time.time()
        self.uav0_count += 1

    def uav1_cb(self, msg):
        with state_lock:
            state["uav1"]["x"] = round(msg.pose.position.x, 1)
            state["uav1"]["y"] = round(msg.pose.position.y, 1)
            state["uav1"]["z"] = round(msg.pose.position.z, 1)
            state["uav1"]["last_seen"] = time.time()
        self.uav1_count += 1

    def alert_cb(self, msg):
        with state_lock:
            state["alerts"].insert(0, {
                "text": msg.data,
                "time": time.strftime("%H:%M:%S")
            })
            state["alerts"] = state["alerts"][:20]  # keep last 20

    def cyber_status_cb(self, msg):
        text = msg.data
        level = "SAFE"
        if "CRITICAL" in text:
            level = "CRITICAL"
        elif "WARNING" in text:
            level = "WARNING"
        with state_lock:
            state["threat_level"] = level

    def uav0_health_cb(self, msg):
        with state_lock:
            state["uav0_health"] = round(float(msg.data), 2)

    def uav1_health_cb(self, msg):
        with state_lock:
            state["uav1_health"] = round(float(msg.data), 2)

    def update_state(self):
        now = time.time()
        elapsed = now - self.last_rate_check

        if elapsed >= 1.0:
            uav0_hz = round(self.uav0_count / elapsed, 1)
            uav1_hz = round(self.uav1_count / elapsed, 1)
            self.uav0_count = 0
            self.uav1_count = 0
            self.last_rate_check = now

            with state_lock:
                state["uav0"]["rate_hz"] = uav0_hz
                state["uav1"]["rate_hz"] = uav1_hz

        with state_lock:
            # Distance
            dx = state["uav0"]["x"] - state["uav1"]["x"]
            dy = state["uav0"]["y"] - state["uav1"]["y"]
            dz = state["uav0"]["z"] - state["uav1"]["z"]
            state["distance_m"] = round(math.sqrt(dx**2 + dy**2 + dz**2), 1)

            # Uptime
            state["uptime_s"] = int(now - state["started_at"])

            # Comm status from health
            h0 = state["uav0_health"]
            h1 = state["uav1_health"]
            avg = (h0 + h1) / 2 if (h0 + h1) > 0 else 0
            if state["uav0"]["last_seen"] is None:
                state["comm_status"] = "WAITING"
            elif avg >= 0.9:
                state["comm_status"] = "CLEAR"
            elif avg >= 0.7:
                state["comm_status"] = "BREEZE"
            elif avg >= 0.5:
                state["comm_status"] = "RAIN"
            else:
                state["comm_status"] = "STORM"

            # Mission status
            z0 = state["uav0"]["z"]
            z1 = state["uav1"]["z"]
            if state["uav0"]["last_seen"] is None:
                state["mission_status"] = "WAITING FOR DRONES"
            elif z0 > 25 and z1 > 80:
                state["mission_status"] = "MISSION ACTIVE"
            elif z0 > 5 or z1 > 5:
                state["mission_status"] = "CLIMBING"
            else:
                state["mission_status"] = "ON GROUND"


def run_http_server():
    server = HTTPServer(('localhost', 8765), DashboardHandler)
    print("Dashboard API running at http://localhost:8765/state")
    server.serve_forever()


def main():
    # Start HTTP server in background thread
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()

    # Start ROS2
    rclpy.init()
    node = DashboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
