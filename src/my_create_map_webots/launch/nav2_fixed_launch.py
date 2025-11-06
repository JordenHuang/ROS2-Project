import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration # 導入 LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('my_create_map_webots')
    robot_description_path = os.path.join(package_dir, 'resource', 'MyCreate_realsense.urdf')
    rviz_config_file = os.path.join(package_dir, 'config', 'rviz_config.rviz')
    ekf_params_path = os.path.join(package_dir, 'config', 'ekf.yaml')
    world_path = os.path.join(package_dir, 'worlds', 'school-2nd-floor2.wbt')

    # 為清晰起見定義 use_sim_time
    use_sim_time = True

    # 為 map 和 nav2_params_file 定義 LaunchConfigurations
    # 如果這些地圖/參數不在 'my_create_map_webots' 中，您需要一個 'robot_navigation2_dir' 或類似的變數
    # 目前，我假設它們位於 'my_create_map_webots' 或指定一個佔位符
    # 如果 'my_create_map_webots' 是套件：
    map_file_path = LaunchConfiguration('map', default=os.path.join(package_dir, 'maps', 'hallway_1.yaml'))
    nav2_params_path = LaunchConfiguration('params_file', default=os.path.join(package_dir, 'config', 'nav2_params_fix2.yaml'))

    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True
    )

    my_robot_driver = WebotsController(
        robot_name='MyCreate_realsense',
        parameters=[{
            'robot_description': robot_description_path,
            'use_sim_time': use_sim_time,
        }],
        respawn=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(robot_description_path).read(),
        }]
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'publish_tf': False,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/imu/data_raw', 'imu/data_without_mag'),
            ('imu/mag', '/imu/mag'),
        ]
    )

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
            "range_max": 4.0,
            "range_min": 0.01,
            "scan_height": 10,
            "use_sim_time": use_sim_time,
        }],
        remappings=[
            ("depth", "/camera/depth/image_raw"),
            ("depth_camera_info", "/camera/depth/camera_info"),
        ]
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file_path, # 嘗試明確解析為字符串
            'use_sim_time': str(use_sim_time).lower(),
            'params_file': nav2_params_path, # 在這裡使用 LaunchConfiguration
            'autostart': 'true',
        }.items(),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        my_robot_driver,
        robot_state_publisher,
        imu_filter,
        depth_image_to_laserscan,
        ekf_filter_node,

        TimerAction(
            period=2.0,
            actions=[
                nav2_bringup_launch,
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