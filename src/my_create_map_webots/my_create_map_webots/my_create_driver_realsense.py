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
from sensor_msgs.msg import Image, CameraInfo, Imu, MagneticField
import cv2
from cv_bridge import CvBridge, CvBridgeError

from my_create_map_webots.vision_node import VisionProcessor
from controller import Supervisor


# --- Constants ---
# LINEAR_SPEED_FORWARD = 0.22
# LINEAR_SPEED_BACKWARD = 0.15
# ANGULAR_SPEED = 1.0

WHEEL_RADIUS       = 0.031
AXLE_LENGTH        = 0.271756 # 0.235


class MyCreateDriverRealsense:
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

        self.camera = self._robot.getDevice("RealsenseD455_rgb")
        self.camera.enable(self._TIMESTEP)
        self.camera_range = self._robot.getDevice("RealsenseD455_depth")
        self.camera_range.enable(self._TIMESTEP)

        self.camWidth = self.camera.getWidth()
        self.camHeight = self.camera.getHeight()
        self.camFov = self.camera.getFov()

        self.imu_accelerometer = self._robot.getDevice("MPU-9250 accelerometer")
        self.imu_accelerometer.enable(self._TIMESTEP)
        self.imu_gyro = self._robot.getDevice("MPU-9250 gyro")
        self.imu_gyro.enable(self._TIMESTEP)
        self.imu_compass = self._robot.getDevice("MPU-9250 compass")
        self.imu_compass.enable(self._TIMESTEP)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.node = rclpy.create_node("my_create_node")

        reliable_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Keep the last 10 messages
        )

        # Publishers
        self.camImgPub = self.node.create_publisher(Image, "/camera/image_raw", reliable_qos_profile)
        self.camInfoPub = self.node.create_publisher(CameraInfo, "/camera/camera_info", reliable_qos_profile)
        self.camDepthPub = self.node.create_publisher(Image, "/camera/depth/image_raw", reliable_qos_profile)
        self.depthCamInfoPub = self.node.create_publisher(CameraInfo, "/camera/depth/camera_info", reliable_qos_profile)
        self.imu_pub = self.node.create_publisher(Imu, "/imu/data_without_mag", reliable_qos_profile)
        self.mag_pub = self.node.create_publisher(MagneticField, "/imu/mag", reliable_qos_profile)

        # Subscriber
        self.node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        #建立轉換器物件
        self.bridge = CvBridge()
        self.node.get_logger().info(f"Driver init done")

        # Calculate camera intrinsics and extrinsics (ONLY ONCE)
        self._calculate_camera_parameters()

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        self._counter += 1

        key = self._keyboard.getKey()
        vel = np.pi / 1 * 1.5
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
            forward_speed = self.__target_twist.linear.x
            angular_speed = self.__target_twist.angular.z

            command_motor_left = (forward_speed - angular_speed * AXLE_LENGTH/2) / WHEEL_RADIUS / 3.5
            command_motor_right = (forward_speed + angular_speed * AXLE_LENGTH/2) / WHEEL_RADIUS / 3.5
            self._left_motor.setVelocity(command_motor_left)
            self._right_motor.setVelocity(command_motor_right)

            # self._left_motor.setVelocity(0.0)
            # self._right_motor.setVelocity(0.0)

        # if self._counter < 50:
        #     return

        self._counter = 0
        shape = (self.camHeight, self.camWidth, 4)
        now = Time(seconds=self._robot.getTime()).to_msg()
        # self.node.get_logger().info(f"time: {now}")

        # === Publish camera image ===
        # Get image from both camera
        imgRaw = self.camera.getImage()

        # --- Convert ---
        # Left
        npImg = np.frombuffer(imgRaw, dtype=np.uint8).reshape(shape)
        bgrImg = cv2.cvtColor(npImg, cv2.COLOR_BGRA2BGR)
        # bgrImg = cv2.cvtColor(npImg, cv2.COLOR_BGRA2GRAY)


        # --- Create message ---
        encoding = 'bgr8' #"passthrough"
        # encoding = 'mono8'
        imgMsg = self.bridge.cv2_to_imgmsg(bgrImg, encoding=encoding)

        # --- Set fields ---
        imgMsg.header.stamp = now
        imgMsg.header.frame_id = "camera_rgb_optical_frame" # Frame ID for the left camera

        # --- Publish ---
        self.camImgPub.publish(imgMsg)

        # === Publish camera info ===
        # --- Left CameraInfo ---
        camInfoMsg = CameraInfo()
        camInfoMsg.header.stamp = now
        camInfoMsg.header.frame_id = 'camera_rgb_optical_frame' # Must match image frame_id
        camInfoMsg.width = self.camWidth
        camInfoMsg.height = self.camHeight
        camInfoMsg.distortion_model = "plumb_bob" # Common distortion model (assuming no distortion here)
        camInfoMsg.d = self.D
        camInfoMsg.k = self.K.flatten().tolist() # K must be flattened to a 1D list of 9 elements
        camInfoMsg.r = self.R # R must be flattened to a 1D list of 9 elements
        camInfoMsg.p = self.P_left # P must be flattened to a 1D list of 12 elements

        # --- Publish ---
        self.camInfoPub.publish(camInfoMsg)

        # === Publish depth image ===
        # --- Get depth image
        depthImgRaw = self.camera_range.getRangeImageArray()
        depthImg = np.asarray(depthImgRaw, dtype=np.float32)

        # --- Create message ---
        encoding = "32FC1"
        imgMsg = self.bridge.cv2_to_imgmsg(depthImg, encoding=encoding)

        # --- Set fields ---
        imgMsg.header.stamp = now
        imgMsg.header.frame_id = "camera_depth_optical_frame" # Frame ID for the left camera

        # --- Publish ---
        self.camDepthPub.publish(imgMsg)

        # === Publish depth camera info ===
        depthCamInfoMsg = CameraInfo()
        depthCamInfoMsg.header.stamp = now
        depthCamInfoMsg.header.frame_id = 'camera_depth_optical_frame' # Must match image frame_id
        depthCamInfoMsg.width = self.camWidth
        depthCamInfoMsg.height = self.camHeight
        depthCamInfoMsg.distortion_model = "plumb_bob" # Common distortion model (assuming no distortion here)
        depthCamInfoMsg.d = self.D
        depthCamInfoMsg.k = self.K.flatten().tolist() # K must be flattened to a 1D list of 9 elements
        depthCamInfoMsg.r = self.R # R must be flattened to a 1D list of 9 elements
        depthCamInfoMsg.p = self.P_left # P must be flattened to a 1D list of 12 elements

        # --- Publish ---
        self.depthCamInfoPub.publish(depthCamInfoMsg)

        # === Publish IMU data ===
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "imu_link"

        # --- 獲取線性加速度 ---
        linear_accel = self.imu_accelerometer.getValues()
        imu_msg.linear_acceleration.x = linear_accel[0]
        imu_msg.linear_acceleration.y = linear_accel[1]
        imu_msg.linear_acceleration.z = linear_accel[2]

        # --- 獲取角速度 ---
        angular_vel = self.imu_gyro.getValues()
        imu_msg.angular_velocity.x = angular_vel[0]
        imu_msg.angular_velocity.y = angular_vel[1]
        imu_msg.angular_velocity.z = angular_vel[2]

        # --- 處理方向 ---
        # 由於我們沒有直接的方向輸出，我們需要告訴下游節點「方向數據是無效的」。
        # 標準的做法是將四元數設為 (0,0,0,1)，並將協方差矩陣的第一個元素設為 -1。
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        # [關鍵] orientation_covariance[0] = -1 代表「請忽略方向數據」
        imu_msg.orientation_covariance[0] = -1.0 

        # --- 填充其他協方差 ---
        # 我們提供一個很小的、非零的對角協方差，代表角速度和加速度數據是「可信的」。
        small_covariance = 0.01 
        imu_msg.angular_velocity_covariance[0] = small_covariance
        imu_msg.angular_velocity_covariance[4] = small_covariance
        imu_msg.angular_velocity_covariance[8] = small_covariance

        imu_msg.linear_acceleration_covariance[0] = small_covariance
        imu_msg.linear_acceleration_covariance[4] = small_covariance
        imu_msg.linear_acceleration_covariance[8] = small_covariance

        # --- Publish ---
        self.imu_pub.publish(imu_msg)

        # --- 4. 發佈磁力計訊息 ---
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = "imu_link"

        # Webots 的 Compass 提供 x, y, z 方向的磁場強度 (單位: Tesla)
        mag_values = self.imu_compass.getValues()
        mag_msg.magnetic_field.x = mag_values[0]
        mag_msg.magnetic_field.y = mag_values[1]
        mag_msg.magnetic_field.z = mag_values[2]

        # 填充磁力計的協方差
        mag_msg.magnetic_field_covariance[0] = small_covariance
        mag_msg.magnetic_field_covariance[4] = small_covariance
        mag_msg.magnetic_field_covariance[8] = small_covariance

        self.mag_pub.publish(mag_msg)

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
        self.baseline = 0.075 # meters

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

