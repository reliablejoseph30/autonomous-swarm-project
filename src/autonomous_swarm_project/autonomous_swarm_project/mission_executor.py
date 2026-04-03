import rclpy
import yaml
import os
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')

        # 1. Load the Mission Data from YAML
        self.mission_data = self.load_mission()

        # QoS Profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 2. Publishers for UAV1 (Namespace px4_1)
        self.uav1_offboard_mode_pub = self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', qos_profile)
        self.uav1_trajectory_pub = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', qos_profile)
        self.uav1_command_pub = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', qos_profile)

        # 3. Publishers for UAV2 (Namespace px4_2)
        self.uav2_offboard_mode_pub = self.create_publisher(OffboardControlMode, '/px4_2/fmu/in/offboard_control_mode', qos_profile)
        self.uav2_trajectory_pub = self.create_publisher(TrajectorySetpoint, '/px4_2/fmu/in/trajectory_setpoint', qos_profile)
        self.uav2_command_pub = self.create_publisher(VehicleCommand, '/px4_2/fmu/in/vehicle_command', qos_profile)

        # Timer variables
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.offboard_setpoint_counter = 0

    def load_mission(self):
        path = os.path.expanduser('~/autonomous_swarm_project/config/waypoints.yaml')
        try:
            with open(path, 'r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found at {path}")
            sys.exit(1)

    def timer_callback(self):
        # Must send OffboardControlMode at least 2Hz before switching to Offboard mode
        self.publish_offboard_control_mode()

        if self.offboard_setpoint_counter == 10:
            # Change to Offboard mode and Arm after 1 second of heartbeats
            self.engage_offboard_mode()
            self.arm_drones()

        # Publish the trajectory points
        self.publish_trajectory_setpoints()
        self.offboard_setpoint_counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.uav1_offboard_mode_pub.publish(msg)
        self.uav2_offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoints(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # UAV1 Logic
        if 'uav1' in self.mission_data and len(self.mission_data['uav1']) > 0:
            p1 = self.mission_data['uav1'][0]
            msg1 = TrajectorySetpoint()
            msg1.x = float(p1['x'])
            msg1.y = float(p1['y'])
            msg1.z = float(p1['z'])
            msg1.yaw = 1.5708  # Default 90 deg orientation
            msg1.timestamp = timestamp
            self.uav1_trajectory_pub.publish(msg1)

        # UAV2 Logic
        if 'uav2' in self.mission_data and len(self.mission_data['uav2']) > 0:
            p2 = self.mission_data['uav2'][0]
            msg2 = TrajectorySetpoint()
            msg2.x = float(p2['x'])
            msg2.y = float(p2['y'])
            msg2.z = float(p2['z'])
            msg2.yaw = 1.5708
            msg2.timestamp = timestamp
            self.uav2_trajectory_pub.publish(msg2)

    def arm_drones(self):
        self.get_logger().info("Arming drones...")
        self.publish_vehicle_command(1, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.publish_vehicle_command(2, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def engage_offboard_mode(self):
        self.get_logger().info("Engaging Offboard mode...")
        # 1.0, 6.0 is the standard for PX4 Offboard Mode
        self.publish_vehicle_command(1, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.publish_vehicle_command(2, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def publish_vehicle_command(self, uav_id, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        # PX4 Instance 0 usually has SysID 1, Instance 1 has SysID 2
        msg.target_system = uav_id 
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        if uav_id == 1:
            self.uav1_command_pub.publish(msg)
        else:
            self.uav2_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    swarm_controller = SwarmController()
    try:
        rclpy.spin(swarm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        swarm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
