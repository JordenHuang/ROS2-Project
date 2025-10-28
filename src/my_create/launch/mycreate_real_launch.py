import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

def generate_launch_description():
    package_dir = get_package_share_directory('my_create')
    ekf_params_path = os.path.join(package_dir, 'config', 'ekf.yaml')
    create_package_dir = get_package_share_directory('create_bringup')
    realsense_package_dir = get_package_share_directory('realsense2_camera')

    # --- 1. 準備好你的「豪華版」URDF ---
    #    定義你的主 xacro 檔案的路徑
    robot_description_path = PathJoinSubstitution(
        [package_dir, 'resource', 'create_with_realsense.urdf.xacro']
    )
    robot_description_content = Command(['xacro ', robot_description_path])

    # Create2 robot
    create_launch = PathJoinSubstitution(
        [create_package_dir, 'launch', 'create_2.launch']
    )

    # Realsense
    realsense_launch = PathJoinSubstitution(
        [realsense_package_dir, 'launch', 'rs_launch.py']
    )

    create = GroupAction(
        actions=[
            SetRemap(src='cmd_vel', dst='/cmd_vel_out'),
            # SetRemap(src='odom', dst='/wheel/odom'),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([create_launch])
            ),
        ]
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_launch]),
        launch_arguments=[
            # ('enable_rgbd', 'true'),
            ('enable_sync', 'true'),
            ('align_depth.enable', 'true'),
            ('enable_color', 'true'),
            ('enable_depth', 'true'),
            ('unite_imu_method', '1'),
            ('enable_accel', 'true'),
            ('enable_gyro', 'true'),
            ('rgb_camera.color_profile',   '424x240x15'),
            ('depth_module.depth_profile', '424x240x15'),
            ('depth_module.infra_profile', '424x240x15'),
            # ('rgb_camera.color_profile',   '640x480x30'),
            # ('depth_module.depth_profile', '640x480x30'),
            # ('depth_module.infra_profile', '640x480x30'),

            ('stereo_module.emitter_enabled', 'true'),
        ('allow_no_texture_points', 'true'),
        # a) 时间滤波器 (Temporal Filter)：平滑连续帧之间的深度值
        ('temporal_filter.enable', 'true'),
        # b) 空间滤波器 (Spatial Filter)：填补小空洞
        ('spatial_filter.enable', 'true'),
        ('spatial_filter.filter_magnitude', '2'),
        ('spatial_filter.filter_smooth_alpha', '0.5'),
        ('spatial_filter.filter_smooth_delta', '20'),
        # c) 空洞填充滤波器 (Hole Filling Filter)
        ('hole_filling_filter.enable', 'true'),
        ('hole_filling_filter.filter_mode', 'Farest_From_Around'), # 或 'Downsample'
        ]
    )

    # ekf_filter_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_params_path],
    #     remappings=[
    #         # EKF 會將融合後的結果發佈到 /odometry/filtered
    #         # 為了方便，我們把它 remapping 成 /odom
    #         # ('/odometry/filtered', '/odom')
    #     ]
    # )

    rtabmap_rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rtabmap_rgbd_sync',
        output='screen',
        parameters=[{
            'qos': 0,
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,
            'topic_queue_size': 30,
            'sync_queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ],
    )

    depth_image_to_laserscan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        output='screen',
        namespace="d2l",
        name="depth_image_to_laserscan",
        parameters=[{
# <param name="scan_height"     type="int"    value="1"/> <!-- default: 1 pixel. Number of pixel rows used to generate laser scan. -->
# <param name="scan_time"       type="double" value="0.033"/> <!-- default:0.033, 30 FPS . Time between scans. -->
# <param name="range_min"       type="double" value="0.45"/> <!--default:0.45m. Ranges less than this are considered -Inf. -->
# <param name="range_max"       type="double" value="10.0"/> <!--default: 10m. Ranges less than this are considered +Inf. -->
# <param name="output_frame_id" type="str"    value="camera_depth_frame"/> <!--default: camera_depth_frame. Frame id of the laser scan. -->
            "range_max": 4.0,
            "range_min": 0.1,
            "scan_height": 1,
            "use_sim_time": False,
            # 'qos': 1,
        }],
        remappings=[
            # ("depth", "/camera/camera/depth/image_rect_raw"),
            # ("depth_camera_info", "/camera/camera/depth/camera_info"),

            ("depth", "/camera/camera/aligned_depth_to_color/image_raw"),
            ("depth_camera_info","/camera/camera/aligned_depth_to_color/camera_info"),
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description_content,
        }]
    )


    return LaunchDescription([
        create,
        realsense,
        # rtabmap_rgbd_sync_node,
        depth_image_to_laserscan,
        robot_state_publisher,
        # ekf_filter_node,
    ])


