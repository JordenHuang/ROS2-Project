from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_dir = get_package_share_directory('my_create')

    rviz_config_file = os.path.join(package_dir, 'config', 'rviz_config.rviz')
    nav2_params_file = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    explore_params_file = os.path.join(package_dir, 'config', 'explore.yaml')
    twist_mux_params_file = os.path.join(package_dir, 'config', 'twist_mux.yaml')
    joy_params_file = os.path.join(package_dir, 'config', 'mycreate_xbox360.yaml')

    # Directories
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Paths
    # Our launch file has the same name as the nav2_bringup one
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    # Includes
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('params_file', nav2_params_file),
        ]
    )

    explore_lite = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        parameters=[explore_params_file, {"use_sim_time": False}],
        output="screen",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
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
            "use_sim_time": False,
            # 'qos': 1,
        }],
        remappings=[
            ("depth", "/camera/camera/depth/image_rect_raw"),
            ("depth_camera_info", "/camera/camera/depth/camera_info"),

            # ('depth', '/camera/depth/image_rect_raw/compressedDepth'),
        ],
    )

    # rtabmap_rgbd_sync_node = Node(
    #     package='rtabmap_sync',
    #     executable='rgbd_sync',
    #     name='rtabmap_rgbd_sync',
    #     output='screen',
    #     parameters=[{
    #         'qos': 1,
    #         'approx_sync': True,
    #         'approx_sync_max_interval': 0.05,
    #         'topic_queue_size': 30,
    #         'sync_queue_size': 30,
    #     }],
    #     remappings=[
    #          ('rgb/image', '/camera/camera/color/image_raw'),
    #          ('depth/image', '/camera/camera/depth/image_rect_raw'),
    #          ('rgb/camera_info', '/camera/camera/color/camera_info'),
    #     ],
    # )


    # RTAB-Map SLAM Node
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'base_link',
        # 'subscribe_rgbd': True,
        # 'subscribe_rgb': True,
            'subscribe_depth': True,
            'subscribe_scan': True,
            'subscribe_odom_info': False, # Set to false because we uses wheel odometry now
            'publish_tf': True,
            'approx_sync': True,
        'approx_sync_max_interval': 0.1,

            'map_always_update': True,
            'sync_queue_size': 30,
            'topic_queue_size': 30,

            "RGBD/ProximityBySpace": "false",
            "RGBD/AngularUpdate": "0.01",
            "RGBD/LinearUpdate": "0.01",
            "RGBD/OptimizeFromGraphEnd": "false",
            "RGBD/CreateOccupancyGrid": "true",
            "Reg/Force3DoF": "true",
            "Vis/MinInliers": "12",

            "Grid/Sensor": "2",
            'Grid/MaxObstacleHeight': '0.4',
            'Grid/NormalsSegmentation': 'false',
            "Grid/RayTracing": "true",
            'Grid/RangeMax': '4.0',
            'Grid/RangeMin': '0.1',
            "Rtabmap/StartNewMapOnLoopClosure": "true",

            'qos_image': 1,
            # 'qos_rgbd': 1, # 如果你订阅的是 rgbd_image，这个参数更具体
            # 同样地，为其他输入也设定可靠的 QoS 是一个好习惯
            'qos_odom': 1,
            'qos_scan': 1,
        }],
        remappings=[
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/depth/image_rect_raw'),
        # ('rgbd_image', 'rgbd_image/compressed'),
            ('scan', '/d2l/scan'),
            ('odom', '/odometry/filtered'),
        ],
        arguments=['-d',]# "--udebug"]
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params_file],
        # remappings=[('/cmd_vel_out', '/cmd_vel')] # [关键] 将 twist_mux 的输出连接到机器人 Driver
    )

    joy_launch = PathJoinSubstitution([package_dir, 'launch', 'mycreate_joy_teleop.launch'])

    joy = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([joy_launch])
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        rtabmap_slam_node,
        nav2,
        depth_image_to_laserscan,
        # rtabmap_rgbd_sync_node,
        # explore_lite,
        twist_mux_node,
        joy,
        rviz2,
    ])

# Terminal cmd vel control
# ❯ ros2 run teleop_twist_keyboard teleop_twist_keyboard
