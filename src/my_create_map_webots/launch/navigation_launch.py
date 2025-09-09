from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

def generate_launch_description():
    package_dir = get_package_share_directory('my_create_map_webots')
    nav2_params_file = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    explore_params_file = os.path.join(package_dir, 'config', 'explore.yaml')
    # explore_config = os.path.join(
    #     get_package_share_directory("explore_lite"), "config", "params.yaml"
    # )

    # Directories
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Paths
    # Our launch file has the same name as the nav2_bringup one
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    explore_lite_launch = PathJoinSubstitution(
        [FindPackageShare('explore_lite'), 'launch', 'explore.launch.py']
    )

    # Includes
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('params_file', nav2_params_file)
        ]
    )

    # explore_lite = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([explore_lite_launch]),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #     }.items(),
    # )

    explore_lite = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        parameters=[explore_params_file, {"use_sim_time": True}],
        output="screen",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return LaunchDescription([
        nav2,
        # TimerAction(
        #     period=2.5,
        #     actions=[
        #         explore_lite,
        #     ]
        # ),
    ])