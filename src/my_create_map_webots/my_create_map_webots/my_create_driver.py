#!/usr/bin/env python3

# =======================================================================================
# File: my_create_driver.py
# Description: TODO
# =======================================================================================

# Standard library imports
import cv2
import numpy as np

# ROS and CV Bridge imports
import rclpy
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError

from my_create_map_webots.vision_node import VisionProcessor
from controller import Supervisor


# --- Constants ---
LINEAR_SPEED_FORWARD = 0.22
LINEAR_SPEED_BACKWARD = 0.15
ANGULAR_SPEED = 1.0

class MyCreateDriver:
    def init(self, webots_node, properties):
        """
        Initializes the robot driver plugin.
        """

        # We still need the webots_node to get the robot instance
        self._robot = webots_node.robot

        # Get the time step of the current world.
        self._TIMESTEP = int(self._robot.getBasicTimeStep())

        # Enable keyboard (Use keyboard to control the robot when focusing the webots window)
        self._keyboard  = self._robot.getKeyboard()
        self._keyboard.enable(self._TIMESTEP)

        self._left_motor = self._robot.getDevice("left wheel motor")
        self._right_motor = self._robot.getDevice("right wheel motor")
        self._left_motor.setPosition(float('+inf'));
        self._right_motor.setPosition(float('+inf'));
        self._left_motor.setVelocity(0.0)
        self._right_motor.setVelocity(0.0)

        self._counter = 100

        self.camLeft = self._robot.getDevice("camera_left")
        self.camLeft.enable(self._TIMESTEP)

        self.camRight = self._robot.getDevice("camera_right")
        self.camRight.enable(self._TIMESTEP)

        # Both camera (should) have the same width and height
        self.camWidth = self.camLeft.getWidth()
        self.camHeight = self.camLeft.getHeight()
        self.camFov = self.camLeft.getFov()

        rclpy.init(args=None)
        self.node = rclpy.create_node("my_create_node")

        # For CameraInfo (less bandwidth, more critical)
        # info_qos_profile = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RELIABLE, # Can be reliable
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=10
        # )
        # info_qos_profile = 10
        reliable_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Keep the last 10 messages
        )

        # Publishers
        self.camLeftImgPub = self.node.create_publisher(Image, "/left/image_raw", reliable_qos_profile)
        self.camRightImgPub = self.node.create_publisher(Image, "/right/image_raw", reliable_qos_profile)
        self.camLeftInfoPub = self.node.create_publisher(CameraInfo, "/left/camera_info", reliable_qos_profile)
        self.camRightInfoPub = self.node.create_publisher(CameraInfo, "/right/camera_info", reliable_qos_profile)
        # self.camLeftImgPub = self.node.create_publisher(Image, "/camera_left/image_raw", reliable_qos_profile)
        # self.camRightImgPub = self.node.create_publisher(Image, "/camera_right/image_raw", reliable_qos_profile)
        # self.camLeftInfoPub = self.node.create_publisher(CameraInfo, "/camera_left/camera_info", reliable_qos_profile)
        # self.camRightInfoPub = self.node.create_publisher(CameraInfo, "/camera_right/camera_info", reliable_qos_profile)

        # My own /clock publisher
        # self.clockPub = self.node.create_publisher(Clock, "/clock", 10)

        #建立轉換器物件
        self.bridge = CvBridge()
        self.node.get_logger().info(f"Driver init done")

        # Calculate camera intrinsics and extrinsics (ONLY ONCE)
        self._calculate_camera_parameters()

    def step(self):
        self._counter += 1

        key = self._keyboard.getKey()
        vel = np.pi / 1
        if key == ord('W'):
            self._left_motor.setVelocity(vel)
            self._right_motor.setVelocity(vel)
        elif key == ord('S'):
            self._left_motor.setVelocity(-vel)
            self._right_motor.setVelocity(-vel)
        elif key == ord('A'):
            vel = vel / 3
            self._left_motor.setVelocity(-vel)
            self._right_motor.setVelocity(vel)
        elif key == ord('D'):
            vel = vel / 3
            self._left_motor.setVelocity(vel)
            self._right_motor.setVelocity(-vel)
        else:
            self._left_motor.setVelocity(0.0)
            self._right_motor.setVelocity(0.0)

        # if self._counter < 50:
        #     return

        self._counter = 0
        shape = (self.camHeight, self.camWidth, 4)
        # now = self.node.get_clock().now().to_msg()
        now = Time(seconds=self._robot.getTime()).to_msg()
        # self.node.get_logger().info(f"time: {now}")

        # clock_message = Clock()
        # clock_message.clock = now
        # self.clockPub.publish(clock_message)

        # sim_time_sec = self._robot.getTime()  # Webots 提供的模擬秒數（float）
        # sec = int(sim_time_sec)
        # nsec = int((sim_time_sec - sec) * 1e9)
        # clock_msg = Clock()
        # now = Time(seconds=sec, nanoseconds=nsec).to_msg()
        # clock_msg.clock = now
        # self.clockPub.publish(clock_msg)


        # === Publish camera image of both cameras ===
        # Get image from both camera
        imgRawLeft = self.camLeft.getImage()
        imgRawRight = self.camRight.getImage()

        # --- Convert ---
        # Left
        npImgLeft = np.frombuffer(imgRawLeft, dtype=np.uint8).reshape(shape)
        bgrImgLeft = cv2.cvtColor(npImgLeft, cv2.COLOR_BGRA2BGR)
        # bgrImgLeft = cv2.cvtColor(npImgLeft, cv2.COLOR_BGRA2GRAY)

        # Right
        npImgRight = np.frombuffer(imgRawRight, dtype=np.uint8).reshape(shape)
        bgrImgRight = cv2.cvtColor(npImgRight, cv2.COLOR_BGRA2BGR)
        # bgrImgRight = cv2.cvtColor(npImgRight, cv2.COLOR_BGRA2GRAY)

        # --- Create message ---
        encoding = 'bgr8' #"passthrough"
        # encoding = 'mono8'
        leftImgMsg = self.bridge.cv2_to_imgmsg(bgrImgLeft, encoding=encoding)
        rightImgMsg = self.bridge.cv2_to_imgmsg(bgrImgRight, encoding=encoding)

        # --- Set fields ---
        leftImgMsg.header.stamp = now
        leftImgMsg.header.frame_id = "camera_left_optical_frame" # Frame ID for the left camera
        rightImgMsg.header.stamp = now
        rightImgMsg.header.frame_id = "camera_right_optical_frame" # Frame ID for the right camera

        # --- Publish ---
        # if rclpy.ok():
        self.camLeftImgPub.publish(leftImgMsg)
        self.camRightImgPub.publish(rightImgMsg)

        # === Publish camera info ===
        # --- Left CameraInfo ---
        camLeftInfoMsg = CameraInfo()
        camLeftInfoMsg.header.stamp = now
        camLeftInfoMsg.header.frame_id = 'camera_left_optical_frame' # Must match image frame_id
        camLeftInfoMsg.width = self.camWidth
        camLeftInfoMsg.height = self.camHeight
        camLeftInfoMsg.distortion_model = "plumb_bob" # Common distortion model (assuming no distortion here)
        camLeftInfoMsg.d = self.D
        camLeftInfoMsg.k = self.K.flatten().tolist() # K must be flattened to a 1D list of 9 elements
        camLeftInfoMsg.r = self.R # R must be flattened to a 1D list of 9 elements
        camLeftInfoMsg.p = self.P_left # P must be flattened to a 1D list of 12 elements

        # --- Right CameraInfo ---
        # (similar to left, but P might differ if it encodes stereo baseline)
        camRightInfoMsg = CameraInfo()
        camRightInfoMsg.header.stamp = now
        camRightInfoMsg.header.frame_id = 'camera_right_optical_frame' # Must match image frame_id
        camRightInfoMsg.width = self.camWidth
        camRightInfoMsg.height = self.camHeight
        camRightInfoMsg.distortion_model = "plumb_bob"
        camRightInfoMsg.d = self.D
        camRightInfoMsg.k = self.K.flatten().tolist()
        camRightInfoMsg.r = self.R
        camRightInfoMsg.p = self.P_right # Use P_right if it's different

        # --- Publish ---
        # if rclpy.ok():
        self.camLeftInfoPub.publish(camLeftInfoMsg)
        self.camRightInfoPub.publish(camRightInfoMsg)

        rclpy.spin_once(self.node, timeout_sec=0)


    def _calculate_camera_parameters(self):
        # Intrinsics (K matrix)
        focalLengthPixels = (self.camWidth / 2.0) / np.tan(self.camFov / 2.0)
        fx, fy = focalLengthPixels, focalLengthPixels
        cx = self.camWidth / 2.0
        cy = self.camHeight / 2.0

        # K matrix: [fx 0 cx; 0 fy cy; 0 0 1]
        self.K = np.array([
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ], dtype=np.float64).reshape(3, 3) # Reshape to 3x3 for CameraInfo.K

        # Distortion coefficients (D matrix): Assuming no distortion in Webots
        self.D = np.zeros(5, dtype=np.float64).tolist() # Needs to be a list for CameraInfo.D

        # Rectification matrix (R matrix): Identity for ideal cameras
        self.R = np.eye(3, dtype=np.float64).flatten().tolist() # Flatten for CameraInfo.R

        # Projection matrix (P matrix) for rectified image: [fx' 0 cx' Tx; 0 fy' cy' Ty; 0 0 1 0]
        # For a monocular camera, Tx, Ty are 0. For stereo, Tx is -fx * baseline for right camera
        # RTAB-Map expects the P matrix to be for the *rectified* camera.
        # Since Webots cameras are often ideal, and we assume stereoRectify is done internally by RTAB-Map,
        # we provide the monocular P matrix here. RTAB-Map will use the provided CameraInfo for its internal stereoRectify.

        # P matrix for left camera (P1 from stereoRectify context)
        self.P_left = np.array([
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ], dtype=np.float64).flatten().tolist() # Flatten for CameraInfo.P

        # P matrix for right camera (P2 from stereoRectify context)
        # The last column's first element is -fx * baseline for the right camera's P matrix
        # Your Webots setup is vertical stereo, but RTAB-Map (and most stereo algorithms)
        # assume horizontal disparity. RTAB-Map will rectify based on the TFs.
        # It's safest to provide the P matrix *as if* it were for a monocular camera,
        # and let RTAB-Map's internal stereoRectify handle the baseline.
        # However, if RTAB-Map's stereo processing expects a pre-rectified P matrix that includes baseline,
        # you *might* need to adjust this. For initial setup, provide standard monocular P.

        # Let's re-verify the baseline based on your Webots setup:
        # camera_left translation: 0.17 0.05 0.05
        # camera_right translation: 0.17 -0.05 0.0499997
        # The baseline is 0.05 - (-0.05) = 0.1m along the Y-axis (vertical stereo)
        self.baseline = 0.1 # meters

        # IMPORTANT for RTAB-Map:
        # If RTAB-Map's internal stereo module expects a P matrix with the baseline for the right camera,
        # the P matrix for the right camera should be:
        # [fx 0 cx -fx*B; 0 fy cy 0; 0 0 1 0] for a horizontal stereo baseline 'B'.
        # For *your vertical stereo*, it's more complex.
        # The simplest approach is to let RTAB-Map figure it out from your TF transforms (base_link to camera_left_link and base_link to camera_right_link).
        # So, provide P matrices that are just like the K matrix, with an extra column of zeros for Tx/Ty.

        self.P_right = self.P_left # For initial setup, assume they are the same in terms of projection
        #                           # RTAB-Map will use TF for baseline information

        # self.P_right = [
        #     fx, 0.0, cx, -fx * self.baseline,
        #     0.0, fy, cy, 0.0,
        #     0.0, 0.0, 1.0, 0.0
        # ]

        # self.P_right = np.array([
        #     fx, 0.0, cx, -fx * self.baseline,
        #     0.0, fy, cy, 0.0,
        #     0.0, 0.0, 1.0, 0.0
        # ], dtype=np.float64).flatten().tolist() # Flatten for CameraInfo.P

