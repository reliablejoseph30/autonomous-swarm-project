#!/usr/bin/env python3
"""
relay_node.py — Toba
Telecommunications & Relay Node
MSc Advanced Drone Technology — Group Project

What this node does:
  1. Subscribes to both drone positions via MAVROS
  2. Broadcasts positions on /swarm/pose_array
  3. Monitors distance between UAV0 and UAV1
  4. Publishes alerts to /net/alerts
  5. Publishes health status to /diagnostics
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# ── Mission parameters (from Peter's specs) ──────────────
CORRIDOR_WIDTH   = 3.47   # metres between drone lanes
MIN_SAFE_GAP     = 2.0    # metres — minimum safe separation
SOFT_BUFFER      = 2.0    # metres — slow down zone
HARD_GEOFENCE    = 0.5    # metres — return to home
COMM_RANGE_LIMIT = 50.0   # metres — simulated max comms range

# UAV0 hovers at 30m, UAV1 orbits at 100m
# Natural altitude separation = 70m — already safe
UAV0_ALT = 30.0
UAV1_ALT = 100.0


class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_node')

        # Position storage — start as None until first message arrives
        self.uav0_pos = None
        self.uav1_pos = None

        # ── Subscribers (MAVROS topics) ───────────────────
        self.sub_uav0 = self.create_subscription(
            PoseStamped,
            '/uav0/mavros/local_position/pose',
            self.uav0_callback,
            10
        )
        self.sub_uav1 = self.create_subscription(
            PoseStamped,
            '/uav1/mavros/local_position/pose',
            self.uav1_callback,
            10
        )

        # ── Publishers ────────────────────────────────────
        self.pose_pub = self.create_publisher(
            PoseArray, '/swarm/pose_array', 10)

        self.alert_pub = self.create_publisher(
            String, '/net/alerts', 10)

        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        # ── 10Hz timer ────────────────────────────────────
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            '\n========================================'
            '\n  Relay Node Started — Toba'
            '\n  Listening on MAVROS topics:'
            '\n  /uav0/mavros/local_position/pose'
            '\n  /uav1/mavros/local_position/pose'
            '\n  Publishing to /swarm/pose_array'
            '\n  Min safe gap   : 2.0m'
            '\n  Comm range     : 50.0m'
            '\n========================================'
        )

    def uav0_callback(self, msg):
        """Receives UAV0 position (hovering at 30m)"""
        self.uav0_pos = msg

    def uav1_callback(self, msg):
        """Receives UAV1 position (orbiting at 100m)"""
        self.uav1_pos = msg

    def timer_callback(self):
        self.broadcast_positions()

        if self.uav0_pos is not None and self.uav1_pos is not None:
            distance = self.get_distance(self.uav0_pos, self.uav1_pos)
            self.check_comm_range(distance)
            self.check_separation(distance)
            self.publish_diagnostics(distance)
        else:
            missing = []
            if self.uav0_pos is None:
                missing.append('UAV0')
            if self.uav1_pos is None:
                missing.append('UAV1')
            self.get_logger().info(
                f'Waiting for position from: {", ".join(missing)}',
                throttle_duration_sec=3.0
            )

    def broadcast_positions(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        if self.uav0_pos is not None:
            p0 = Pose()
            p0.position.x = self.uav0_pos.pose.position.x
            p0.position.y = self.uav0_pos.pose.position.y
            p0.position.z = self.uav0_pos.pose.position.z
            msg.poses.append(p0)

        if self.uav1_pos is not None:
            p1 = Pose()
            p1.position.x = self.uav1_pos.pose.position.x
            p1.position.y = self.uav1_pos.pose.position.y
            p1.position.z = self.uav1_pos.pose.position.z
            msg.poses.append(p1)

        self.pose_pub.publish(msg)

    def check_comm_range(self, distance):
        if distance > COMM_RANGE_LIMIT:
            self.publish_alert(
                f'COMM_LOSS: UAV0-UAV1 distance={distance:.1f}m '
                f'exceeds comm range={COMM_RANGE_LIMIT}m. '
                f'Relay repositioning required.'
            )
        elif distance > COMM_RANGE_LIMIT * 0.8:
            self.publish_alert(
                f'COMM_WARN: UAV0-UAV1 distance={distance:.1f}m '
                f'approaching comm limit={COMM_RANGE_LIMIT}m.'
            )
        else:
            self.get_logger().info(
                f'COMM_OK: distance={distance:.2f}m — healthy',
                throttle_duration_sec=5.0
            )

    def check_separation(self, distance):
        if distance < MIN_SAFE_GAP:
            self.publish_alert(
                f'SEPARATION_ALERT: UAV0-UAV1 only {distance:.2f}m apart! '
                f'Minimum safe gap is {MIN_SAFE_GAP}m.'
            )
            self.get_logger().error(
                f'DANGER: Drones too close! {distance:.2f}m < {MIN_SAFE_GAP}m'
            )

    def publish_alert(self, message):
        msg = String()
        msg.data = message
        self.alert_pub.publish(msg)
        self.get_logger().warn(message)

    def publish_diagnostics(self, distance):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'relay_node/swarm_comms'
        status.hardware_id = 'toba_relay'

        if distance > COMM_RANGE_LIMIT:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Communication range exceeded'
        elif distance < MIN_SAFE_GAP:
            status.level = DiagnosticStatus.WARN
            status.message = 'Drones below minimum safe separation'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Swarm communication healthy'

        status.values = [
            KeyValue(key='uav0_uav1_distance_m', value=f'{distance:.2f}'),
            KeyValue(key='uav0_altitude_m',       value=str(UAV0_ALT)),
            KeyValue(key='uav1_altitude_m',       value=str(UAV1_ALT)),
            KeyValue(key='corridor_width_m',      value=str(CORRIDOR_WIDTH)),
            KeyValue(key='min_safe_gap_m',        value=str(MIN_SAFE_GAP)),
            KeyValue(key='comm_range_limit_m',    value=str(COMM_RANGE_LIMIT)),
        ]

        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)

    def get_distance(self, pos1, pos2):
        return math.sqrt(
            (pos1.pose.position.x - pos2.pose.position.x) ** 2 +
            (pos1.pose.position.y - pos2.pose.position.y) ** 2 +
            (pos1.pose.position.z - pos2.pose.position.z) ** 2
        )


def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
