import os
import launch
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('my_create_map_webots')
    robot_description_path = os.path.join(package_dir, 'resource', 'MyCreate_realsense.urdf')
    rviz_config_file = os.path.join(package_dir, 'config', 'rviz_config.rviz')
    ekf_params_path = os.path.join(package_dir, 'config', 'ekf.yaml')
    nav2_params_file = os.path.join(package_dir, 'config', 'nav2_params_fix.yaml')
    map_file = os.path.join(package_dir, 'maps', 'rtabmap_0801.yaml')
    world_path = os.path.join(package_dir, 'worlds', 'school-obstacle.wbt')

    # 啟動 Webots
    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True
    )

    # robot driver
    my_robot_driver = WebotsController(
        robot_name='MyCreate_realsense',
        parameters=[{
            'robot_description': robot_description_path,
            'use_sim_time': True,
        }],
        respawn=True
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(robot_description_path).read(),
        }]
    )
    # IMU Filter (Madgwick)
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'publish_tf': False,
            'use_sim_time': True, # 確保使用模擬時間
        }],
        remappings=[
            # 根據您的Webots IMU發布的話題修改 '/imu/data_raw'
            ('/imu/data_raw', '/imu'), # 假設Webots的IMU數據直接發布到 /imu
            ('imu/mag', '/imu/mag'), # 如果沒有磁力計，這個remapping可能不需要
            # imu_filter_madgwick 會在 'imu/data' 上發布過濾後的IMU數據
        ]
    )

    # EKF (融合 IMU + Odometry)
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_path],
        remappings=[
            ('/odometry/filtered', '/odom')
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
            "range_max": 4.0,
            "range_min": 0.01,
            "scan_height": 10,
            "use_sim_time": True,
        }],
        remappings=[
            ("depth", "/camera/depth/image_raw"),
            ("depth_camera_info", "/camera/depth/camera_info"),
        ]
    )

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
    )

    # AMCL (定位)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', 'amcl:=debug'], # 添加这一行
         remappings=[
            ("scan", "/d2l/scan"),
        ]
    )

    # Nav2 控制器、規劃器、行為樹
    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    )

    nav2_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Lifecycle Manager (啟動所有 Nav2 節點)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )

    # RViz
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        my_robot_driver,
        robot_state_publisher,
        imu_filter,
        ekf_filter_node,

        TimerAction(
            period=2.0,
            actions=[
                depth_image_to_laserscan,
                map_server,
                amcl,
                nav2_controller,
                nav2_planner,
                nav2_bt_navigator,
                nav2_waypoint_follower,
                lifecycle_manager,
                rviz2,
            ]
        ),

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
