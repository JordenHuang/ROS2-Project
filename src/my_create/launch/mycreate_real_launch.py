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
            # ('align_depth.enable', 'true'),
            ('enable_color', 'true'),
            ('enable_depth', 'true'),
            ('unite_imu_method', '1'),
            ('enable_accel', 'true'),
            ('enable_gyro', 'true'),
            ('rgb_camera.color_profile',   '640x480x15'),
            ('depth_module.depth_profile', '640x480x15'),
            ('depth_module.infra_profile', '640x480x15'),
        ]
    )

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_path],
        remappings=[
            # EKF 會將融合後的結果發佈到 /odometry/filtered
            # 為了方便，我們把它 remapping 成 /odom
            # ('/odometry/filtered', '/odom')
        ]
    )

    # rtabmap_rgbd_sync_node = Node(
    #     package='rtabmap_sync',
    #     executable='rgbd_sync',
    #     name='rtabmap_rgbd_sync',
    #     output='screen',
    #     parameters=[{
    #         'qos': 0,
    #         'approx_sync': True,
    #         'approx_sync_max_interval': 0.05,
    #         'topic_queue_size': 30,
    #         'sync_queue_size': 30,
    #     }],
    #     remappings=[
    #         ('rgb/image', '/camera/camera/color/image_raw'),
    #         ('depth/image', '/camera/camera/depth/image_rect_raw'),
    #         ('rgb/camera_info', '/camera/camera/color/camera_info'),
    #     ],
    # )

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
        robot_state_publisher,
        ekf_filter_node,
    ])


