# 用來看機器人姿態URDF

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

import xacro
import tempfile

def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    with tempfile.NamedTemporaryFile(prefix="%s_" % os.path.basename(xacro_path), delete=False) as xacro_file:
        urdf_path = xacro_file.name

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(doc.toprettyxml(indent='  '))

    return urdf_path

def generate_launch_description():
    # 獲取你的 package 路徑
    bringup_dir = get_package_share_directory('my_create')
    
    # 定義你的主 xacro 檔案的路徑
    xacro_file = os.path.join(bringup_dir, 'resource', 'create_with_realsense.urdf.xacro')
    # xacro_file = os.path.join(bringup_dir, 'resource', 'create_with_realsense.urdf.copy.xacro')
    
    # 使用 Command 來讓 xacro 處理檔案
    robot_description_content = Command(['xacro ', xacro_file])

    urdf = to_urdf(xacro_file, {'use_nominal_extrinsics': 'true'})
    # xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', "test_d435i_camera.urdf.xacro")
    # urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true'})
    # --- 1. 啟動 Robot State Publisher ---
    # 它的工作就是讀取 URDF 並發佈 /tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # parameters=[{
        #     # [重要] 這裡設為 False，因為我們沒有模擬器
        #     'use_sim_time': False, 
        #     'robot_description': robot_description_content
        # }]
        arguments=[urdf],
    )
    
    # --- 2. 啟動 RVIZ2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])