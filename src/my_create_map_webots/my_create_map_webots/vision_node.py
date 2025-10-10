#!/usr/bin/env python3
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')

        self.bridge = CvBridge()

        self.left_image = None
        self.right_image = None

        # 建立 ROS2 訂閱者
        self.create_subscription(Image, "/Left_camera/image", self.left_callback, qos_profile_sensor_data)
        self.create_subscription(Image, "/Right_camera/image", self.right_callback, qos_profile_sensor_data)

        self.get_logger().info("✅ VisionProcessor initialized")

    def left_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.left_image = cv_image

            # 若要顯示
            cv2.imshow("Left Camera", self.left_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to process left image: {e}")

    def right_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.right_image = cv_image

            # 若要顯示
            cv2.imshow("Right Camera", self.right_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to process right image: {e}")



def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionProcessor()
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    vision_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
