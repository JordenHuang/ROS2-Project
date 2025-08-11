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
    robot_description_path = os.path.join(package_dir, 'resource', 'MyCreate_kinect.urdf')
    world_path = os.path.join(package_dir, 'worlds', 'school-obstacle.wbt')

    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True
    )

    my_robot_driver = WebotsController(
        robot_name='MyCreate_kinect',
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

    # window_size = 13
    # p1 = 216 #8 * 3 * window_size**2
    # p2 = 864 #32 * 3 * window_size**2
    # stereo_image_proc = IncludeLaunchDescription(
    #     PathJoinSubstitution([
    #         FindPackageShare('stereo_image_proc'),
    #         'launch',
    #         'stereo_image_proc.launch.py'
    #     ]),
    #     launch_arguments={
    #         # 'namespace': 'stereo',
    #         # 'approximate_sync': 'True',
    #         # 'stereo_algorithm': '1',
    #         # 'sgbm_mode': '2',

    #         # 'disparity_range': '192',       # 視差範圍，必須是 16 的倍數
    #         # 'correlation_window_size': str(window_size),   # 必須是奇數，通常 3-11
    #         # 'P1': "{:.1f}".format(p1),   # 視差平滑度參數 (8 * channels * blockSize^2)
    #         # 'P2': "{:.1f}".format(p2),  # 視差平滑度參數 (32 * channels * blockSize^2)
    #         # 'disp12_max_diff': '1',         # 允許左右視差的最大差異
    #         # 'speckle_window_size': '200',
    #         # 'speckle_range': '2',
    #         # 'uniqueness_ratio': "{:.1f}".format(10),      # 唯一性比率，用於濾波
    #         # 'prefilter_cap': '63',         # 預濾波器截斷值
    #     }.items(),
    # )

    # disparity_image_view = Node(
    #     package='image_view',
    #     executable='disparity_view',
    #     name='disparity_image_view',
    #     remappings=[
    #         ('image', '/disparity')
    #     ]
    # )


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
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
        ]
    )

    # rtabmap_disparity_to_depth = Node(
    #     package='rtabmap_util',
    #     executable='disparity_to_depth',
    #     name='rtabmap_disparity_to_depth',
    #     namespace='disparity2depth',
    #     remappings=[
    #         ('disparity', '/disparity')
    #     ]
    # )

    # rtabmap_pointcloud_to_depthimage = Node(
    #     package='rtabmap_util',
    #     executable='pointcloud_to_depthimage',
    #     name='rtabmap_pointcloud_to_depthimage',
    #     namespace='pointcloud2depthImage',
    #     remappings=[
    #         ('camera_info', '/left/camera_info'),
    #         ('cloud', '/points2')
    #     ],
    # )

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
            "range_min": 0.05,
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
            'publish_tf': True,
            'approx_sync': True,

            # 'map_always_update': True,
            # 'map_empty_ray_tracing': True,

            "RGBD/ProximityBySpace": "false",
            "RGBD/AngularUpdate": "0.01",
            "RGBD/LinearUpdate": "0.01",
            "RGBD/OptimizeFromGraphEnd": "false",
            "Reg/Force3DoF": "true",
            "Vis/MinInliers": "12",

            "Grid/Sensor": "1",
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('scan', '/d2l/scan'),
            # ('rgb/image', '/camera/image_rect_color'),
            # ('rgb/camera_info', '/left/camera_info'),
            # # ('depth/image', '/pointcloud2depthImage/image'),
            # ('depth/image', '/disparity2depth/depth'),
        ],
        arguments=['-d', "--udebug"]
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

    return LaunchDescription([
        webots,
        webots._supervisor,
        my_robot_driver,
        rtabmap_viz,

        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         # stereo_image_proc,
        #         # rtabmap_pointcloud_to_depthimage,
        #         # rtabmap_disparity_to_depth,
        #         # disparity_image_view,
        #     ]
        # ),
        TimerAction(
            period=2.0,
            actions=[
                rtabmap_slam_node,
                rgbd_odometry_node,
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


