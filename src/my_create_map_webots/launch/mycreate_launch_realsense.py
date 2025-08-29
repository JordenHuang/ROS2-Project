import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    package_dir = get_package_share_directory('my_create_map_webots')
    robot_description_path = os.path.join(package_dir, 'resource', 'MyCreate_realsense.urdf')
    rviz_config_file = os.path.join(package_dir, 'config', 'rviz_config.rviz')
    # rviz_config_file = os.path.join(package_dir, 'config', 'rviz_config_ours.rviz')

    # world_path = os.path.join(package_dir, 'worlds', 'school-obstacle.wbt')
    world_path = os.path.join(package_dir, 'worlds', 'school-2nd-floor.wbt')
    # world_path = os.path.join(package_dir, 'worlds', 'test.wbt')

    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True
    )

    my_robot_driver = WebotsController(
        robot_name='MyCreate_realsense',
        parameters=[{
            'robot_description': robot_description_path,
            'use_sim_time': True,
        }],
        respawn=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(robot_description_path).read(),
        }]
    )

    # rgbd Odometry Node
    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'publish_tf': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,
            "Reg/Force3DoF": "true",
            "Vis/MinInliers": "12",
            'wait_imu_to_init': False, # 因為我們沒有有效的初始方向，所以關閉這個

            # [重要] 告訴 RTAB-Map IMU 的方向數據是無效的，但其他數據是有效的
            'Odom/GuessMotion': 'true',
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('imu', '/imu/data'),
        ]
    )

    # ICP Odometry Node
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'publish_tf': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,
        }],
        remappings=[
            ('scan', '/d2l/scan'),
        ]
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
            "range_max": 6.0,
            "range_min": 0.01,
            "scan_height": 10,
            "use_sim_time": True,
        }],
        remappings=[
            ("depth", "/camera/depth/image_raw"),
            ("depth_camera_info", "/camera/depth/camera_info"),
        ]
    )

    # RTAB-Map SLAM Node
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_scan': True,
            'subscribe_odom_info': True,
            'publish_tf': True,
            'approx_sync': True,

            'map_always_update': True,

            "RGBD/ProximityBySpace": "false",
            "RGBD/AngularUpdate": "0.01",
            "RGBD/LinearUpdate": "0.01",
            "RGBD/OptimizeFromGraphEnd": "false",
# "RGBD/CreateOccupancyGrid": "true",
            "Reg/Force3DoF": "true",
            "Vis/MinInliers": "12",
            # "MaxObstacleHeight": "0.1",

            "Grid/Sensor": "0",
            "Grid/Scan2dUnknownSpaceFilled": "true",
"Grid/RayTracing": "true",
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('scan', '/d2l/scan'),
            ('imu', '/imu/data'),
            # ('rgb/image', '/camera/image_rect_color'),
            # ('rgb/camera_info', '/left/camera_info'),
            # # ('depth/image', '/pointcloud2depthImage/image'),
            # ('depth/image', '/disparity2depth/depth'),
        ],
        arguments=['-d',]# "--udebug"]
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        parameters=[{
            'use_sim_time': True,
            'subscribe_depth': True,
            'subscribe_scan': True,
            'subscribe_odom_info': True,
            'sync_queue_size': 30,
            'topic_queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('scan', '/d2l/scan'),
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        my_robot_driver,
        # rtabmap_viz,
        rviz2,

        TimerAction(
            period=2.0,
            actions=[
                rtabmap_slam_node,
                rgbd_odometry_node,
                # icp_odometry_node,
                depth_image_to_laserscan,
                robot_state_publisher,
            ]
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])


