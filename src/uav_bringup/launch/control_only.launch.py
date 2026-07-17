"""
control_only.launch.py — 仅启动控制链（MAVROS 已单独启动时使用）

用法:
  ros2 launch uav_bringup control_only.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bridge_params = os.path.join(
        get_package_share_directory('uav_px4_bridge'), 'config', 'px4_bridge_params.yaml')
    tf_params = os.path.join(
        get_package_share_directory('uav_tf_broadcaster'), 'config', 'tf_broadcaster_params.yaml')
    controller_params = os.path.join(
        get_package_share_directory('uav_controller'), 'config', 'controller_params.yaml')

    return LaunchDescription([
        Node(package='uav_px4_bridge', executable='px4_bridge_node',
             name='px4_bridge_node', output='screen', parameters=[bridge_params]),
        Node(package='uav_tf_broadcaster', executable='tf_broadcaster_node',
             name='tf_broadcaster_node', output='screen', parameters=[tf_params]),
        Node(package='uav_controller', executable='controller_node',
             name='controller_node', output='screen', parameters=[controller_params]),
    ])
