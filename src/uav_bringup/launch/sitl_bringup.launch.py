"""
sitl_bringup.launch.py — 仿真全栈启动：MAVROS + 控制链 + 任务（默认）

用法:
  ros2 launch uav_bringup sitl_bringup.launch.py
  ros2 launch uav_bringup sitl_bringup.launch.py mission:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bridge_params = os.path.join(
        get_package_share_directory('uav_px4_bridge'), 'config', 'px4_bridge_params.yaml')
    tf_params = os.path.join(
        get_package_share_directory('uav_tf_broadcaster'), 'config', 'tf_broadcaster_params.yaml')
    controller_params = os.path.join(
        get_package_share_directory('uav_controller'), 'config', 'controller_params.yaml')

    mavros_pluginlists = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'px4_pluginlists.yaml')
    mavros_config = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'px4_config.yaml')

    mission_arg = DeclareLaunchArgument(
        'mission', default_value='true',
        description='Run demo mission')

    # ── MAVROS ──
    mavros = Node(
        package='mavros', executable='mavros_node',
        namespace='mavros', output='screen',
        emulate_tty=True,
        parameters=[mavros_pluginlists, mavros_config, {
            'fcu_url': 'udp://:14540@127.0.0.1:14580',
            'gcs_url': '',
            'tgt_system': 1,
            'tgt_component': 1,
            'fcu_protocol': 'v2.0',
        }],
    )

    # ── 控制链 ──
    bridge = Node(
        package='uav_px4_bridge', executable='px4_bridge_node',
        name='px4_bridge_node', output='screen',
        emulate_tty=True, parameters=[bridge_params],
    )
    tf_node = Node(
        package='uav_tf_broadcaster', executable='tf_broadcaster_node',
        name='tf_broadcaster_node', output='screen',
        emulate_tty=True, parameters=[tf_params],
    )
    controller = Node(
        package='uav_controller', executable='controller_node',
        name='controller_node', output='screen',
        emulate_tty=True, parameters=[controller_params],
    )

    # ── Mission ──
    mission = Node(
        package='uav_controller', executable='mission_node',
        name='mission_node', output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('mission')),
    )

    return LaunchDescription([
        mission_arg,
        mavros,
        TimerAction(period=5.0, actions=[bridge]),
        TimerAction(period=5.5, actions=[tf_node]),
        TimerAction(period=6.0, actions=[controller]),
        TimerAction(period=20.0, actions=[mission]),
    ])
