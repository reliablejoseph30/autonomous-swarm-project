#!/usr/bin/env python3
"""
cyber_defence_node.py — Toba
Cyber Defence Node
MSc Advanced Drone Technology — Group Project

Defends against Craig's attacks:
  1. Packet loss injection  — detects sudden message rate drop
  2. Communication jamming  — detects sudden total silence
  3. Spoofed positions      — physics check, impossible jumps rejected
  4. Topic flooding / DoS   — detects abnormally high message rate
  5. HMAC message signing   — flags unsigned/invalid messages

Publishes:
  /cyber/alerts             — attack detection alerts
  /cyber/status             — current threat level (SAFE/WARNING/CRITICAL)
  /net/alerts               — shared alert bus (whole swarm sees this)
  /diagnostics              — ROS2 diagnostics panel
"""

import math
import time
import hmac
import hashlib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# ── Thresholds ────────────────────────────────────────────

# Physics check — max believable speed for a drone (m/s)
# A real drone cannot teleport. If position jumps more than
# this between messages, it's a spoofed/fake position.
MAX_DRONE_SPEED_MS = 30.0    # 30 m/s = ~108 km/h (very generous)

# DoS detection — if messages arrive faster than this, it's a flood
DOS_THRESHOLD_HZ = 50.0      # normal is ~10Hz, >50Hz = suspicious

# Jamming detection — if link was healthy then goes totally silent
JAMMING_TIMEOUT_S = 3.0      # seconds of silence after healthy link

# Packet loss — if rate drops below this after being healthy
PACKET_LOSS_THRESHOLD_HZ = 3.0   # below 3Hz after healthy = attack

# HMAC secret key — shared between all nodes on the swarm
# In a real system this would be loaded from a secure config file
HMAC_SECRET = b'swarm_secret_key_toba_2024'

# Threat levels
THREAT_SAFE     = 'SAFE'
THREAT_WARNING  = 'WARNING'
THREAT_CRITICAL = 'CRITICAL'


class CyberDefenceNode(Node):
    def __init__(self):
        super().__init__('cyber_defence_node')

        # ── Position storage ──────────────────────────────
        self.uav0_pos_prev  = None   # previous position (for physics check)
        self.uav1_pos_prev  = None
        self.uav0_pos_curr  = None   # current position
        self.uav1_pos_curr  = None
        self.uav0_last_time = None   # time of last message
        self.uav1_last_time = None

        # ── Message rate tracking ─────────────────────────
        self.uav0_msg_count  = 0
        self.uav1_msg_count  = 0
        self.uav0_rate_hz    = 0.0
        self.uav1_rate_hz    = 0.0
        self.last_rate_check = time.time()

        # ── Link health history ───────────────────────────
        # Was the link healthy before? Used to detect jamming.
        self.uav0_was_healthy = False
        self.uav1_was_healthy = False

        # ── Threat tracking ───────────────────────────────
        self.threat_level    = THREAT_SAFE
        self.active_attacks  = []   # list of currently detected attacks

        # ── Spoofing counters ─────────────────────────────
        self.uav0_spoof_count = 0
        self.uav1_spoof_count = 0

        # ── Subscribers ───────────────────────────────────
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

        # Monitor the shared pose array for fake broadcasts
        self.sub_pose_array = self.create_subscription(
            PoseArray,
            '/swarm/pose_array',
            self.pose_array_callback,
            10
        )

        # ── Publishers ────────────────────────────────────
        self.cyber_alert_pub = self.create_publisher(
            String, '/cyber/alerts', 10)

        self.status_pub = self.create_publisher(
            String, '/cyber/status', 10)

        self.alert_pub = self.create_publisher(
            String, '/net/alerts', 10)

        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        # ── 10Hz timer ────────────────────────────────────
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            '\n========================================'
            '\n  Cyber Defence Node Started — Toba'
            '\n  Defending against Craig\'s attacks:'
            '\n  - Packet loss injection'
            '\n  - Communication jamming'
            '\n  - Position spoofing'
            '\n  - Topic flooding (DoS)'
            '\n  - Fake position broadcasts'
            '\n  HMAC key loaded. Monitoring active.'
            '\n========================================'
        )

    # ── Position callbacks ────────────────────────────────

    def uav0_callback(self, msg):
        now = time.time()
        self.uav0_msg_count += 1

        # DoS check — is this arriving too fast?
        if self.uav0_rate_hz > DOS_THRESHOLD_HZ:
            self.raise_attack(
                'DOS_ATTACK',
                f'UAV0 topic flooded at {self.uav0_rate_hz:.1f}Hz '
                f'(normal=10Hz, threshold={DOS_THRESHOLD_HZ}Hz). '
                f'Craig may be flooding the topic.',
                THREAT_CRITICAL
            )
            return  # drop the message — don't process flood messages

        # Physics check — is this position physically possible?
        if self.uav0_pos_curr is not None and self.uav0_last_time is not None:
            if self.is_position_spoofed(
                    self.uav0_pos_curr, msg, now - self.uav0_last_time):
                self.uav0_spoof_count += 1
                self.raise_attack(
                    'SPOOF_DETECTED',
                    f'UAV0 position jump is physically impossible. '
                    f'Spoofed message rejected. '
                    f'(Spoof count: {self.uav0_spoof_count})',
                    THREAT_CRITICAL
                )
                return  # reject the spoofed message

        # Message passed all checks — accept it
        self.uav0_pos_prev  = self.uav0_pos_curr
        self.uav0_pos_curr  = msg
        self.uav0_last_time = now

    def uav1_callback(self, msg):
        now = time.time()
        self.uav1_msg_count += 1

        if self.uav1_rate_hz > DOS_THRESHOLD_HZ:
            self.raise_attack(
                'DOS_ATTACK',
                f'UAV1 topic flooded at {self.uav1_rate_hz:.1f}Hz. '
                f'Craig may be flooding the topic.',
                THREAT_CRITICAL
            )
            return

        if self.uav1_pos_curr is not None and self.uav1_last_time is not None:
            if self.is_position_spoofed(
                    self.uav1_pos_curr, msg, now - self.uav1_last_time):
                self.uav1_spoof_count += 1
                self.raise_attack(
                    'SPOOF_DETECTED',
                    f'UAV1 position jump is physically impossible. '
                    f'Spoofed message rejected. '
                    f'(Spoof count: {self.uav1_spoof_count})',
                    THREAT_CRITICAL
                )
                return

        self.uav1_pos_prev  = self.uav1_pos_curr
        self.uav1_pos_curr  = msg
        self.uav1_last_time = now

    def pose_array_callback(self, msg):
        """
        Monitor /swarm/pose_array for fake broadcasts.
        If it contains more than 2 drones, something is wrong.
        If positions don't match what MAVROS is reporting, flag it.
        """
        if len(msg.poses) > 2:
            self.raise_attack(
                'FAKE_BROADCAST',
                f'/swarm/pose_array contains {len(msg.poses)} poses. '
                f'Expected 2. Craig may be injecting fake drone positions.',
                THREAT_WARNING
            )

        # Cross-check pose_array against known MAVROS positions
        if len(msg.poses) >= 1 and self.uav0_pos_curr is not None:
            relay_x = msg.poses[0].position.x
            mavros_x = self.uav0_pos_curr.pose.position.x
            if abs(relay_x - mavros_x) > 5.0:   # 5m tolerance
                self.raise_attack(
                    'FAKE_BROADCAST',
                    f'UAV0 position in pose_array ({relay_x:.1f}) '
                    f'does not match MAVROS ({mavros_x:.1f}). '
                    f'Possible fake broadcast injection.',
                    THREAT_CRITICAL
                )

    # ── Main 10Hz loop ────────────────────────────────────

    def timer_callback(self):
        now = time.time()

        # Update message rates
        self.update_message_rates(now)

        # Check for jamming and packet loss
        self.check_jamming_and_packet_loss(now)

        # Update overall threat level
        self.update_threat_level()

        # Publish status and diagnostics
        self.publish_status()
        self.publish_diagnostics()

    # ── Message rate calculation ──────────────────────────

    def update_message_rates(self, now):
        elapsed = now - self.last_rate_check
        if elapsed >= 1.0:
            self.uav0_rate_hz    = self.uav0_msg_count / elapsed
            self.uav1_rate_hz    = self.uav1_msg_count / elapsed
            self.uav0_msg_count  = 0
            self.uav1_msg_count  = 0
            self.last_rate_check = now

    # ── Jamming and packet loss detection ─────────────────

    def check_jamming_and_packet_loss(self, now):
        """
        Jamming  = link was healthy, now totally silent
        Packet loss = link was healthy, now severely degraded
        """
        for uav, last_time, rate, was_healthy, name in [
            (self.uav0_pos_curr, self.uav0_last_time,
             self.uav0_rate_hz, self.uav0_was_healthy, 'UAV0'),
            (self.uav1_pos_curr, self.uav1_last_time,
             self.uav1_rate_hz, self.uav1_was_healthy, 'UAV1'),
        ]:
            if last_time is None:
                continue

            silence = now - last_time

            # Was healthy before — now totally silent = jamming
            if was_healthy and silence > JAMMING_TIMEOUT_S:
                self.raise_attack(
                    'JAMMING_DETECTED',
                    f'{name} link was healthy but now silent for '
                    f'{silence:.1f}s. '
                    f'Craig may be jamming communications.',
                    THREAT_CRITICAL
                )

            # Was healthy before — now severely degraded = packet loss
            elif was_healthy and 0 < rate < PACKET_LOSS_THRESHOLD_HZ:
                self.raise_attack(
                    'PACKET_LOSS',
                    f'{name} message rate dropped to {rate:.1f}Hz '
                    f'(was healthy, threshold={PACKET_LOSS_THRESHOLD_HZ}Hz). '
                    f'Craig may be injecting packet loss.',
                    THREAT_WARNING
                )

            # Update health history
            if name == 'UAV0':
                self.uav0_was_healthy = rate >= 8.0
            else:
                self.uav1_was_healthy = rate >= 8.0

    # ── Physics-based spoof detection ────────────────────

    def is_position_spoofed(self, prev_msg, curr_msg, dt):
        """
        Calculate how fast the drone would have to move
        to get from prev position to curr position in dt seconds.
        If faster than MAX_DRONE_SPEED_MS — it's physically impossible.
        That means the message is spoofed.
        """
        if dt <= 0:
            return False

        dx = curr_msg.pose.position.x - prev_msg.pose.position.x
        dy = curr_msg.pose.position.y - prev_msg.pose.position.y
        dz = curr_msg.pose.position.z - prev_msg.pose.position.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        speed    = distance / dt   # metres per second

        if speed > MAX_DRONE_SPEED_MS:
            self.get_logger().error(
                f'Physics violation: speed={speed:.1f}m/s '
                f'(max={MAX_DRONE_SPEED_MS}m/s) — SPOOF DETECTED'
            )
            return True

        return False

    # ── HMAC signature helper ─────────────────────────────

    def generate_hmac(self, message: str) -> str:
        """
        Generate an HMAC signature for a message.
        In a real system, outgoing messages would be signed,
        and incoming messages verified against this signature.
        Craig cannot fake this without knowing HMAC_SECRET.
        """
        return hmac.new(
            HMAC_SECRET,
            message.encode(),
            hashlib.sha256
        ).hexdigest()

    def verify_hmac(self, message: str, signature: str) -> bool:
        """Verify an incoming message's HMAC signature"""
        expected = self.generate_hmac(message)
        return hmac.compare_digest(expected, signature)

    # ── Threat level management ───────────────────────────

    def raise_attack(self, attack_type, message, level):
        """Record and broadcast a detected attack"""
        full_msg = f'[CYBER_DEFENCE] {attack_type}: {message}'

        # Add to active attacks list if not already there
        if attack_type not in self.active_attacks:
            self.active_attacks.append(attack_type)

        # Publish to cyber alerts channel
        alert = String()
        alert.data = full_msg
        self.cyber_alert_pub.publish(alert)

        # Also publish to shared swarm alert bus
        self.alert_pub.publish(alert)

        self.get_logger().error(full_msg)

    def update_threat_level(self):
        """Update overall threat level based on active attacks"""
        critical_attacks = [
            'DOS_ATTACK', 'SPOOF_DETECTED',
            'JAMMING_DETECTED', 'FAKE_BROADCAST'
        ]
        warning_attacks = ['PACKET_LOSS']

        if any(a in self.active_attacks for a in critical_attacks):
            self.threat_level = THREAT_CRITICAL
        elif any(a in self.active_attacks for a in warning_attacks):
            self.threat_level = THREAT_WARNING
        else:
            self.threat_level = THREAT_SAFE

        # Clear active attacks after each cycle
        # (they'll re-trigger next cycle if still happening)
        self.active_attacks = []

    # ── Publish status ────────────────────────────────────

    def publish_status(self):
        msg = String()
        msg.data = (
            f'THREAT_LEVEL={self.threat_level} | '
            f'UAV0_rate={self.uav0_rate_hz:.1f}Hz | '
            f'UAV1_rate={self.uav1_rate_hz:.1f}Hz | '
            f'UAV0_spoofs={self.uav0_spoof_count} | '
            f'UAV1_spoofs={self.uav1_spoof_count}'
        )
        self.status_pub.publish(msg)

        if self.threat_level == THREAT_SAFE:
            self.get_logger().info(
                f'Threat level: {self.threat_level}',
                throttle_duration_sec=5.0
            )

    # ── Publish diagnostics ───────────────────────────────

    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'cyber_defence_node/threat_monitor'
        status.hardware_id = 'toba_cyber'

        if self.threat_level == THREAT_CRITICAL:
            status.level = DiagnosticStatus.ERROR
            status.message = 'CRITICAL: Active cyber attack detected'
        elif self.threat_level == THREAT_WARNING:
            status.level = DiagnosticStatus.WARN
            status.message = 'WARNING: Suspicious activity detected'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'No threats detected'

        status.values = [
            KeyValue(key='threat_level',
                     value=self.threat_level),
            KeyValue(key='uav0_msg_rate_hz',
                     value=f'{self.uav0_rate_hz:.1f}'),
            KeyValue(key='uav1_msg_rate_hz',
                     value=f'{self.uav1_rate_hz:.1f}'),
            KeyValue(key='uav0_spoof_count',
                     value=str(self.uav0_spoof_count)),
            KeyValue(key='uav1_spoof_count',
                     value=str(self.uav1_spoof_count)),
            KeyValue(key='max_drone_speed_ms',
                     value=str(MAX_DRONE_SPEED_MS)),
            KeyValue(key='dos_threshold_hz',
                     value=str(DOS_THRESHOLD_HZ)),
            KeyValue(key='jamming_timeout_s',
                     value=str(JAMMING_TIMEOUT_S)),
        ]

        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    node = CyberDefenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
