import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import math

# ─────────────────────────────────────────────────────
#  Toba – Telecommunications & Relay Node
#  MSc Advanced Drone Technology – Group Project
#
#  This node:
#  1. Subscribes to both drone positions from PX4
#  2. Broadcasts positions on /swarm/pose_array
#     so drones know each other's location
#  3. Monitors distance between UAV1 and UAV2
#  4. Publishes alerts to /net/alerts
#  5. Publishes health status to /diagnostics
#
#  Peter's specs:
#  - 3.47m corridor width between drone lanes
#  - 2.0m minimum safe separation
#  - 2.0m soft geofence buffer
#  - 0.5m hard geofence
# ─────────────────────────────────────────────────────

CORRIDOR_WIDTH   = 3.47   # metres
MIN_SAFE_GAP     = 2.0    # metres — Peter's minimum separation
SOFT_BUFFER      = 2.0    # metres — slow down zone
HARD_GEOFENCE    = 0.5    # metres — return to home
COMM_RANGE_LIMIT = 50.0   # metres — simulated max comms distance


class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_node')

        # QoS to match PX4 publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Position storage
        self.uav1_pos = None
        self.uav2_pos = None

        # ── Subscribers ──────────────────────────────────
        self.sub_uav1 = self.create_subscription(
            VehicleLocalPosition,
            '/px4_1/fmu/out/vehicle_local_position',
            self.uav1_callback,
            qos
        )
        self.sub_uav2 = self.create_subscription(
            VehicleLocalPosition,
            '/px4_2/fmu/out/vehicle_local_position',
            self.uav2_callback,
            qos
        )

        # ── Publishers ───────────────────────────────────
        self.pose_pub = self.create_publisher(
            PoseArray, '/swarm/pose_array', 10)

        self.alert_pub = self.create_publisher(
            String, '/net/alerts', 10)

        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        # 10Hz timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            '========================================\n'
            '  Relay Node Started — Toba\n'
            '  Monitoring UAV1 and UAV2 positions\n'
            '  Corridor width : 3.47m\n'
            '  Min safe gap   : 2.0m\n'
            '  Comm range     : 50.0m\n'
            '========================================'
        )

    # ── Position callbacks ───────────────────────────────
    def uav1_callback(self, msg):
        self.uav1_pos = msg

    def uav2_callback(self, msg):
        self.uav2_pos = msg

    # ── Main 10Hz loop ───────────────────────────────────
    def timer_callback(self):
        self.broadcast_positions()

        if self.uav1_pos is not None and self.uav2_pos is not None:
            distance = self.get_distance(self.uav1_pos, self.uav2_pos)
            self.check_comm_range(distance)
            self.check_separation(distance)
            self.publish_diagnostics(distance)
        else:
            self.get_logger().info(
                'Waiting for drone positions...',
                throttle_duration_sec=3.0
            )

    # ── Broadcast both positions on /swarm/pose_array ────
    def broadcast_positions(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        if self.uav1_pos is not None:
            p1 = Pose()
            p1.position.x = float(self.uav1_pos.x)
            p1.position.y = float(self.uav1_pos.y)
            p1.position.z = float(self.uav1_pos.z)
            msg.poses.append(p1)

        if self.uav2_pos is not None:
            p2 = Pose()
            p2.position.x = float(self.uav2_pos.x)
            p2.position.y = float(self.uav2_pos.y)
            p2.position.z = float(self.uav2_pos.z)
            msg.poses.append(p2)

        self.pose_pub.publish(msg)

    # ── Check communication range ────────────────────────
    def check_comm_range(self, distance):
        if distance > COMM_RANGE_LIMIT:
            self.publish_alert(
                f'COMM_LOSS: UAV1-UAV2 distance={distance:.1f}m '
                f'exceeds comm range={COMM_RANGE_LIMIT}m. '
                f'Relay repositioning required.'
            )
        elif distance > COMM_RANGE_LIMIT * 0.8:
            self.publish_alert(
                f'COMM_WARN: UAV1-UAV2 distance={distance:.1f}m '
                f'approaching comm limit={COMM_RANGE_LIMIT}m.'
            )
        else:
            self.get_logger().info(
                f'COMM_OK: distance={distance:.2f}m — healthy',
                throttle_duration_sec=5.0
            )

    # ── Check Peter's 2m minimum separation ─────────────
    def check_separation(self, distance):
        if distance < MIN_SAFE_GAP:
            self.publish_alert(
                f'SEPARATION_ALERT: UAV1-UAV2 only {distance:.2f}m apart! '
                f'Minimum safe gap is {MIN_SAFE_GAP}m. '
                f'Corridor width={CORRIDOR_WIDTH}m.'
            )
            self.get_logger().error(
                f'DANGER: Drones too close! {distance:.2f}m < {MIN_SAFE_GAP}m'
            )

    # ── Publish alert helper ─────────────────────────────
    def publish_alert(self, message):
        msg = String()
        msg.data = message
        self.alert_pub.publish(msg)
        self.get_logger().warn(message)

    # ── Publish diagnostics ──────────────────────────────
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
            KeyValue(key='uav1_uav2_distance_m',
                     value=f'{distance:.2f}'),
            KeyValue(key='corridor_width_m',
                     value=str(CORRIDOR_WIDTH)),
            KeyValue(key='min_safe_gap_m',
                     value=str(MIN_SAFE_GAP)),
            KeyValue(key='comm_range_limit_m',
                     value=str(COMM_RANGE_LIMIT)),
            KeyValue(key='soft_buffer_m',
                     value=str(SOFT_BUFFER)),
            KeyValue(key='hard_geofence_m',
                     value=str(HARD_GEOFENCE)),
        ]

        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)

    # ── Distance calculation ─────────────────────────────
    def get_distance(self, pos1, pos2):
        return math.sqrt(
            (pos1.x - pos2.x) ** 2 +
            (pos1.y - pos2.y) ** 2 +
            (pos1.z - pos2.z) ** 2
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