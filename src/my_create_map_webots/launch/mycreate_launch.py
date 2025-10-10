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
    robot_description_path = os.path.join(package_dir, 'resource', 'MyCreate.urdf')
    # world_path = os.path.join(package_dir, 'worlds', 'school-obstacle.wbt')
    world_path = os.path.join(package_dir, 'worlds', 'school-room.wbt')

    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True
    )

    my_robot_driver = WebotsController(
        robot_name='MyCreate',
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
    window_size = 13
    p1 = 216 #8 * 3 * window_size**2
    p2 = 864 #32 * 3 * window_size**2
    stereo_image_proc = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('stereo_image_proc'),
            'launch',
            'stereo_image_proc.launch.py'
        ]),
        launch_arguments={
            # 'namespace': 'stereo',
            # 'approximate_sync': 'True',
            'stereo_algorithm': '1',
            'sgbm_mode': '2',

            'disparity_range': '192',       # 視差範圍，必須是 16 的倍數
            'correlation_window_size': str(window_size),   # 必須是奇數，通常 3-11
            'P1': "{:.1f}".format(p1),   # 視差平滑度參數 (8 * channels * blockSize^2)
            'P2': "{:.1f}".format(p2),  # 視差平滑度參數 (32 * channels * blockSize^2)
            'disp12_max_diff': '1',         # 允許左右視差的最大差異
            'speckle_window_size': '200',
            'speckle_range': '2',
            'uniqueness_ratio': "{:.1f}".format(10),      # 唯一性比率，用於濾波
            'prefilter_cap': '63',         # 預濾波器截斷值
        }.items(),
    )

    disparity_image_view = Node(
        package='image_view',
        executable='disparity_view',
        name='disparity_image_view',
        remappings=[
            ('image', '/disparity')
        ]
    )


    # Stereo Odometry Node
    stereo_odometry_node = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            # 'odom_frame_id': 'odom',
            'publish_tf': True,
            # 'approx_sync': True,
            'approx_sync_max_interval': 0.05,

'Stereo/DenseStrategy' : "1",     # [0=cv::StereoBM, 1=cv::StereoSGBM]
'Stereo/MaxDisparity' : "192",  # [Maximum disparity.]
'Stereo/MinDisparity' : "0",    # [Minimum disparity.]
# 'Stereo/OpticalFlow' : "false",
'Stereo/WinHeight' : "15",
'Stereo/WinWidth' : "15",

            # 1. Feature Detection: Increase the number of features to track. More features = more chances for good matches.
            # 'Vis/MaxFeatures': '5000',  # Default is 1000

            # 2. Inlier Threshold: Minimum number of features that must be successfully tracked.
            # The error was "18/20", so let's lower the requirement slightly to make it more tolerant of brief texture loss.
            'Vis/MinInliers': '15',     # Default is 20

            # 3. Motion Estimation: Tell the odometry to assume the robot is on a flat plane.
            # This adds a strong prior that helps reject bad feature matches that imply flying or going underground.
            'Reg/Force3DoF': 'true',

            # 4. Bundle Adjustment: This refines the pose but can sometimes fail with noisy features.
            # Let's switch to a more robust (but less accurate) mode for now to get a stable base.
            # 'OdomF2M/BundleAdjustment': '0', # 0=off, 1=refine motion, 2=refine structure

            'Odom/Strategy': '1',
            'Vis/CorType': '1',
            'OdomF2M/MaxSize': '1000',
            'Vis/MaxFeatures': '600',
        }],
        remappings=[
            ('left/image_rect', '/left/image_rect_color'),
            ('right/image_rect', '/right/image_rect_color'),
            ('left/camera_info', '/left/camera_info'),
            ('right/camera_info', '/right/camera_info'),
        ]
    )

    rtabmap_disparity_to_depth = Node(
        package='rtabmap_util',
        executable='disparity_to_depth',
        name='rtabmap_disparity_to_depth',
        namespace='disparity2depth',
        remappings=[
            ('disparity', '/disparity')
        ]
    )

    rtabmap_pointcloud_to_depthimage = Node(
        package='rtabmap_util',
        executable='pointcloud_to_depthimage',
        name='rtabmap_pointcloud_to_depthimage',
        namespace='pointcloud2depthImage',
        remappings=[
            ('camera_info', '/left/camera_info'),
            ('cloud', '/points2')
        ],
    )

    # RTAB-Map SLAM Node
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'subscribe_laserScan': False,
            # 'subscribe_depth': True,
            'subscribe_stereo': True,
            'subscribe_odom_info': True,
            'sync_queue_size': 30,
            'topic_queue_size': 30,
            'frame_id': 'base_link',
            # 'map_frame_id': 'map',
            'publish_tf': True,
            # 'approx_sync': True,
            'wait_for_transform_duration': 0.2,

            'map_always_update': True,
            'map_empty_ray_tracing': True,

'Stereo/DenseStrategy': '1',
            # ---- StereoSGBM Parameters ----
            'StereoSGBM/MinDisparity': str(0),          # 最小視差值 (通常為 0)
            'StereoSGBM/NumDisparities': str(192),       # 視差範圍，必須是 16 的倍數
            'StereoSGBM/BlockSize': str(window_size),   # 必須是奇數，通常 3-11
            'StereoSGBM/P1': str(p1), #str(8*3*window_size**2),   # 視差平滑度參數 (8 * channels * blockSize^2)
            'StereoSGBM/P2': str(p2), # str(32*3*window_size**2),  # 視差平滑度參數 (32 * channels * blockSize^2)
            'StereoSGBM/Disp12MaxDiff': str(1),         # 允許左右視差的最大差異
            'StereoSGBM/UniquenessRatio': str(10),      # 唯一性比率，用於濾波
            'StereoSGBM/SpeckleWindowSize':str(200),   # 視差斑點窗口大小，用於濾波
            'StereoSGBM/SpeckleRange': str(2),         # 視差斑點範圍，用於濾波
            'StereoSGBM/PreFilterCap': str(63),         # 預濾波器截斷值
            'StereoSGBM/Mode': str(2),                  # 0: SGBM_MODE_SGBM, 1: SGBM_MODE_HH, 2: SGBM_MODE_SGBM_3WAY, 3: SGBM_MODE_HH4


'Reg/Force3DoF': 'true',
'Optimizer/Slam2D': 'true',

            # 'Grid/3D':'false', # Use 2D occupancy
            # 'Grid/MaxHeight': '1',
            # 'Grid/MaxGroundHeight': '0.05',
            # 'Grid/MaxObstacleHeight': '2.0',
            # 'Grid/MaxGroundAngle': '60',

            'Vis/MinInliers': '15',     # Default is 20
        }],
        remappings=[
            ('left/image_rect', '/left/image_rect_color'),
            ('right/image_rect', '/right/image_rect_color'),
            ('left/camera_info', '/left/camera_info'),
            ('right/camera_info', '/right/camera_info'),
            ('odom', '/odom'),
            # ('rgb/image', '/left/image_rect_color'),
            # ('rgb/camera_info', '/left/camera_info'),
            # # ('depth/image', '/pointcloud2depthImage/image'),
            # ('depth/image', '/disparity2depth/depth'),
        ],
        arguments=['-d'] #, "--udebug"
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        parameters=[{
            'use_sim_time': True,
            'subscribe_laserScan': False,
            # 'subscribe_depth': True,
            'subscribe_stereo': True,
            'subscribe_odom_info': True,
            'sync_queue_size': 30,
            'topic_queue_size': 30,
        }],
        remappings=[
            ('left/image_rect', '/left/image_rect_color'),
            ('right/image_rect', '/right/image_rect_color'),
            ('left/camera_info', '/left/camera_info'),
            ('right/camera_info', '/right/camera_info'),
            # ('rgb/image', '/left/image_rect'),
            # ('rgb/camera_info', '/left/camera_info_color'),
            # ('depth/image', '/disparity2depth/depth'),
            # ('depth/image', '/pointcloud2depthImage/image'),
        ],
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        my_robot_driver,

        TimerAction(
            period=4.0,
            actions=[rtabmap_viz]
        ),
        TimerAction(
            period=6.0,
            actions=[
                stereo_image_proc,
                rtabmap_pointcloud_to_depthimage,
                rtabmap_disparity_to_depth,
                disparity_image_view,
            ]
        ),
        TimerAction(
            period=7.0,
            actions=[rtabmap_slam_node, stereo_odometry_node, robot_state_publisher]
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])


