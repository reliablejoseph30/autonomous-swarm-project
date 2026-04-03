#!/usr/bin/env python3
"""
comms_resilience_node.py — Toba
Communications Resilience Node
MSc Advanced Drone Technology — Group Project

What this node does:
  1. Monitors heartbeat from both drones
     - Detects when a drone goes silent (obstacle/weather)
  2. Tracks message arrival rate
     - Slow rate = degraded link (simulates rain/storm/occlusion)
  3. Monitors 3D separation for anti-collision
     - WARNING at 10m, DANGER at 5m
  4. Simulates BVLOS occlusion detection
     - If no message for 2s = possible obstacle blocking link
  5. Publishes comms health to /comms/health
  6. Publishes alerts to /net/alerts
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import String, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# ── Thresholds ────────────────────────────────────────────
COLLISION_DANGER_M  = 5.0    # metres — DANGER zone
COLLISION_WARNING_M = 10.0   # metres — WARNING zone
OCCLUSION_TIMEOUT   = 2.0    # seconds — no message = possible occlusion
DEGRADED_RATE_HZ    = 5.0    # Hz — below this = degraded link
EXPECTED_RATE_HZ    = 10.0   # Hz — healthy link rate

# Weather condition thresholds (based on simulated message drop rate)
WEATHER_CLEAR    = 0.9   # >90% messages arriving = clear
WEATHER_BREEZE   = 0.7   # 70-90% = light interference
WEATHER_RAIN     = 0.5   # 50-70% = moderate degradation
WEATHER_STORM    = 0.0   # <50% = severe — storm/heavy occlusion


class CommsResilienceNode(Node):
    def __init__(self):
        super().__init__('comms_resilience_node')

        # ── Position storage ──────────────────────────────
        self.uav0_pos  = None
        self.uav1_pos  = None

        # ── Heartbeat tracking ────────────────────────────
        # We record the time of the last message from each drone
        self.uav0_last_seen = None
        self.uav1_last_seen = None

        # ── Message rate tracking ─────────────────────────
        # We count messages per second to estimate link quality
        self.uav0_msg_count  = 0
        self.uav1_msg_count  = 0
        self.uav0_rate_hz    = 0.0
        self.uav1_rate_hz    = 0.0
        self.rate_window_sec = 1.0   # measure rate every 1 second
        self.last_rate_check = time.time()

        # ── Link quality (0.0 = dead, 1.0 = perfect) ─────
        self.uav0_link_quality = 1.0
        self.uav1_link_quality = 1.0

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

        # ── Publishers ────────────────────────────────────
        # Comms health score for each drone (0.0-1.0)
        self.health_uav0_pub = self.create_publisher(
            Float32, '/comms/uav0/health', 10)

        self.health_uav1_pub = self.create_publisher(
            Float32, '/comms/uav1/health', 10)

        # Alerts for occlusion, collision, degradation
        self.alert_pub = self.create_publisher(
            String, '/net/alerts', 10)

        # Diagnostics panel
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        # ── 10Hz timer ────────────────────────────────────
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            '\n========================================'
            '\n  Comms Resilience Node Started — Toba'
            '\n  Monitoring link health for UAV0 + UAV1'
            '\n  Occlusion timeout : 2.0s'
            '\n  Collision danger  : 5.0m'
            '\n  Collision warning : 10.0m'
            '\n========================================'
        )

    # ── Position callbacks ────────────────────────────────

    def uav0_callback(self, msg):
        """Called every time UAV0 sends a position update"""
        self.uav0_pos       = msg
        self.uav0_last_seen = time.time()   # record when we last heard from it
        self.uav0_msg_count += 1            # count this message

    def uav1_callback(self, msg):
        """Called every time UAV1 sends a position update"""
        self.uav1_pos       = msg
        self.uav1_last_seen = time.time()
        self.uav1_msg_count += 1

    # ── Main 10Hz loop ────────────────────────────────────

    def timer_callback(self):
        now = time.time()

        # Update message rates every second
        self.update_message_rates(now)

        # Check if drones have gone silent (occlusion/obstacle)
        self.check_heartbeat(now)

        # Check link quality (weather simulation)
        self.check_link_quality()

        # Check collision separation
        if self.uav0_pos is not None and self.uav1_pos is not None:
            distance = self.get_distance(self.uav0_pos, self.uav1_pos)
            self.check_collision(distance)
            self.publish_diagnostics(distance)
        else:
            self.publish_diagnostics(None)

        # Publish health scores
        self.publish_health()

    # ── Message rate calculation ──────────────────────────

    def update_message_rates(self, now):
        """
        Every second, calculate how many messages arrived.
        Healthy = ~10 per second. Degraded = fewer.
        This simulates weather/obstacle signal interference.
        """
        elapsed = now - self.last_rate_check

        if elapsed >= self.rate_window_sec:
            self.uav0_rate_hz = self.uav0_msg_count / elapsed
            self.uav1_rate_hz = self.uav1_msg_count / elapsed

            # Reset counters for next window
            self.uav0_msg_count  = 0
            self.uav1_msg_count  = 0
            self.last_rate_check = now

            self.get_logger().info(
                f'Link rates — UAV0: {self.uav0_rate_hz:.1f}Hz  '
                f'UAV1: {self.uav1_rate_hz:.1f}Hz',
                throttle_duration_sec=5.0
            )

    # ── Heartbeat check (occlusion/obstacle detection) ───

    def check_heartbeat(self, now):
        """
        If we haven't heard from a drone for OCCLUSION_TIMEOUT seconds,
        it may be behind an obstacle or out of range (BVLOS scenario).
        """
        if self.uav0_last_seen is not None:
            silence = now - self.uav0_last_seen
            if silence > OCCLUSION_TIMEOUT:
                self.publish_alert(
                    f'OCCLUSION_ALERT: UAV0 silent for {silence:.1f}s. '
                    f'Possible obstacle blocking link (BVLOS). '
                    f'Last known position: '
                    f'({self.uav0_pos.pose.position.x:.1f}, '
                    f'{self.uav0_pos.pose.position.y:.1f}, '
                    f'{self.uav0_pos.pose.position.z:.1f})'
                    if self.uav0_pos else
                    f'OCCLUSION_ALERT: UAV0 silent for {silence:.1f}s. No position known.'
                )

        if self.uav1_last_seen is not None:
            silence = now - self.uav1_last_seen
            if silence > OCCLUSION_TIMEOUT:
                self.publish_alert(
                    f'OCCLUSION_ALERT: UAV1 silent for {silence:.1f}s. '
                    f'Possible obstacle blocking link (BVLOS). '
                    f'Last known position: '
                    f'({self.uav1_pos.pose.position.x:.1f}, '
                    f'{self.uav1_pos.pose.position.y:.1f}, '
                    f'{self.uav1_pos.pose.position.z:.1f})'
                    if self.uav1_pos else
                    f'OCCLUSION_ALERT: UAV1 silent for {silence:.1f}s. No position known.'
                )

        # If we have never heard from a drone at all
        if self.uav0_last_seen is None:
            self.get_logger().info(
                'Waiting for first message from UAV0...',
                throttle_duration_sec=3.0
            )
        if self.uav1_last_seen is None:
            self.get_logger().info(
                'Waiting for first message from UAV1...',
                throttle_duration_sec=3.0
            )

    # ── Link quality check (weather simulation) ──────────

    def check_link_quality(self):
        """
        Estimate link quality from message rate.
        Low rate = weather/interference degrading the link.
        Only runs after we have heard from a drone at least once.
        """
        # Don't report weather until drones are actually flying
        uav0_active = self.uav0_last_seen is not None
        uav1_active = self.uav1_last_seen is not None

        if not uav0_active and not uav1_active:
            return  # no drones heard yet — stay silent

        # Calculate quality as ratio of actual vs expected rate
        if EXPECTED_RATE_HZ > 0:
            self.uav0_link_quality = min(
                1.0, self.uav0_rate_hz / EXPECTED_RATE_HZ)
            self.uav1_link_quality = min(
                1.0, self.uav1_rate_hz / EXPECTED_RATE_HZ)

        for uav, quality in [('UAV0', self.uav0_link_quality),
                              ('UAV1', self.uav1_link_quality)]:
            condition = self.get_weather_condition(quality)

            if condition == 'STORM':
                self.publish_alert(
                    f'WEATHER_SEVERE: {uav} link quality={quality:.0%}. '
                    f'Storm/heavy occlusion detected. '
                    f'Activating redundant comms pathway.'
                )
            elif condition == 'RAIN':
                self.publish_alert(
                    f'WEATHER_MODERATE: {uav} link quality={quality:.0%}. '
                    f'Rain/moderate interference detected.'
                )
            elif condition == 'BREEZE':
                self.get_logger().warn(
                    f'WEATHER_LIGHT: {uav} link quality={quality:.0%}. '
                    f'Light interference detected.'
                )

    def get_weather_condition(self, quality):
        """Classify link quality into a weather-like condition"""
        if quality >= WEATHER_CLEAR:
            return 'CLEAR'
        elif quality >= WEATHER_BREEZE:
            return 'BREEZE'
        elif quality >= WEATHER_RAIN:
            return 'RAIN'
        else:
            return 'STORM'

    # ── Anti-collision check ──────────────────────────────

    def check_collision(self, distance):
        """
        UAV0 at 30m, UAV1 at 100m gives 70m natural separation.
        This monitors in case paths converge during mission.
        """
        if distance < COLLISION_DANGER_M:
            self.publish_alert(
                f'COLLISION_DANGER: Drones only {distance:.2f}m apart! '
                f'IMMEDIATE action required. '
                f'UAV0 target alt=30m, UAV1 target alt=100m.'
            )
            self.get_logger().error(
                f'COLLISION DANGER: {distance:.2f}m — CRITICAL'
            )
        elif distance < COLLISION_WARNING_M:
            self.publish_alert(
                f'COLLISION_WARNING: Drones {distance:.2f}m apart. '
                f'Approaching minimum safe separation.'
            )
            self.get_logger().warn(
                f'COLLISION WARNING: {distance:.2f}m — monitor closely'
            )
        else:
            self.get_logger().info(
                f'SEPARATION_OK: {distance:.2f}m — safe',
                throttle_duration_sec=5.0
            )

    # ── Publish health scores ─────────────────────────────

    def publish_health(self):
        """Publish link quality 0.0-1.0 for each drone"""
        msg0 = Float32()
        msg0.data = float(self.uav0_link_quality)
        self.health_uav0_pub.publish(msg0)

        msg1 = Float32()
        msg1.data = float(self.uav1_link_quality)
        self.health_uav1_pub.publish(msg1)

    # ── Publish alert helper ──────────────────────────────

    def publish_alert(self, message):
        msg = String()
        msg.data = message
        self.alert_pub.publish(msg)
        self.get_logger().warn(message)

    # ── Publish diagnostics ───────────────────────────────

    def publish_diagnostics(self, distance):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'comms_resilience_node/link_health'
        status.hardware_id = 'toba_comms'

        uav0_condition = self.get_weather_condition(self.uav0_link_quality)
        uav1_condition = self.get_weather_condition(self.uav1_link_quality)

        if uav0_condition == 'STORM' or uav1_condition == 'STORM':
            status.level = DiagnosticStatus.ERROR
            status.message = 'Severe link degradation — storm/heavy occlusion'
        elif uav0_condition == 'RAIN' or uav1_condition == 'RAIN':
            status.level = DiagnosticStatus.WARN
            status.message = 'Moderate link degradation — rain/occlusion'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Comms link healthy'

        status.values = [
            KeyValue(key='uav0_link_quality',
                     value=f'{self.uav0_link_quality:.2f}'),
            KeyValue(key='uav1_link_quality',
                     value=f'{self.uav1_link_quality:.2f}'),
            KeyValue(key='uav0_msg_rate_hz',
                     value=f'{self.uav0_rate_hz:.1f}'),
            KeyValue(key='uav1_msg_rate_hz',
                     value=f'{self.uav1_rate_hz:.1f}'),
            KeyValue(key='uav0_condition',
                     value=uav0_condition),
            KeyValue(key='uav1_condition',
                     value=uav1_condition),
            KeyValue(key='separation_m',
                     value=f'{distance:.2f}' if distance else 'unknown'),
            KeyValue(key='occlusion_timeout_s',
                     value=str(OCCLUSION_TIMEOUT)),
        ]

        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)

    # ── 3D distance calculation ───────────────────────────

    def get_distance(self, pos1, pos2):
        return math.sqrt(
            (pos1.pose.position.x - pos2.pose.position.x) ** 2 +
            (pos1.pose.position.y - pos2.pose.position.y) ** 2 +
            (pos1.pose.position.z - pos2.pose.position.z) ** 2
        )


def main(args=None):
    rclpy.init(args=args)
    node = CommsResilienceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
