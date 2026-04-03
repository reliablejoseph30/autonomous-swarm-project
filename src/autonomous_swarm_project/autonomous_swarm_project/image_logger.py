import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class SwarmLogger(Node):
    def __init__(self):
        super().__init__('swarm_logger')
        self.bridge = CvBridge()
        
        # CHANGE THIS to your actual local mount path if different
        self.base_path = os.path.expanduser('~/OneDrive/Autonomous_Swarm_Project/Surveys/')
        
        # Create separate folders for the swarm members
        os.makedirs(os.path.join(self.base_path, 'UAV1'), exist_ok=True)
        os.makedirs(os.path.join(self.base_path, 'UAV2'), exist_ok=True)

        # Subscribing to both drones using the Spine namespaces
        self.sub1 = self.create_subscription(Image, '/px4_1/camera/image_raw', lambda msg: self.save_img(msg, 1), 10)
        self.sub2 = self.create_subscription(Image, '/px4_2/camera/image_raw', lambda msg: self.save_img(msg, 2), 10)

    def save_img(self, msg, drone_id):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        t = self.get_clock().now().to_msg().sec
        
        # Final Path: ~/OneDrive/Autonomous_Swarm_Project/Surveys/UAV1/frame_12345.png
        filename = os.path.join(self.base_path, f'UAV{drone_id}', f'frame_{t}.png')
        cv2.imwrite(filename, cv_img)

def main():
    rclpy.init()
    rclpy.spin(SwarmLogger())

if __name__ == '__main__':
    main()
