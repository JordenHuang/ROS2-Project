# 用來看機器人姿態URDF

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # 獲取你的 package 路徑
    bringup_dir = get_package_share_directory('my_create')
    
    # 定義你的主 xacro 檔案的路徑
    xacro_file = os.path.join(bringup_dir, 'resource', 'create_with_realsense.urdf.xacro')
    
    # 使用 Command 來讓 xacro 處理檔案
    robot_description_content = Command(['xacro ', xacro_file])

    # --- 1. 啟動 Robot State Publisher ---
    # 它的工作就是讀取 URDF 並發佈 /tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            # [重要] 這裡設為 False，因為我們沒有模擬器
            'use_sim_time': False, 
            'robot_description': robot_description_content
        }]
    )
    
    # --- 2. 啟動 RVIZ2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])