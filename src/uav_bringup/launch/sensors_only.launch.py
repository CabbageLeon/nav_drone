"""
sensors_only.launch.py — 仅启动传感器 + MAVROS + 桥接，不含 offboard 控制

用法:
  ros2 launch uav_bringup sensors_only.launch.py

启动后手动启动 mission:
  ros2 run uav_controller mission_node
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bridge_params = os.path.join(
        get_package_share_directory('uav_px4_bridge'), 'config', 'px4_bridge_params.yaml')
    tf_params = os.path.join(
        get_package_share_directory('uav_tf_broadcaster'), 'config', 'tf_broadcaster_params.yaml')

    # ── 雷达驱动 + Fast-LIO ──
    lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fast_lio'),
                         'launch', 'mapping_mid360.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items(),
    )

    # ── MAVROS / PX4 ──
    mavros_px4 = ExecuteProcess(
        cmd=['ros2', 'launch', 'mavros', 'px4.launch',
             'fcu_url:=/dev/px4:921600'],
        output='screen',
        name='mavros_px4',
    )

    # ── 桥接 + TF（不含 controller/mission）──
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

    return LaunchDescription([
        lio_launch,
        mavros_px4,
        TimerAction(period=5.0, actions=[bridge]),
        TimerAction(period=5.5, actions=[tf_node]),
    ])
