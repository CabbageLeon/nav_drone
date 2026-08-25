"""
uav_system.launch.py — unified UAV launch for SITL and real-machine profiles.

Usage:
  ros2 launch uav_bringup uav_system.launch.py profile:=sitl
  ros2 launch uav_bringup uav_system.launch.py profile:=real

Node enable flags live in uav_bringup/config/uav_common_params.yaml.
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


_REQUIRED_FLAGS = (
    'mavros',
    'lidar',
    'realsense',
    'mission',
    'spf',
    'spf_image_test',
    'ugv_tcp',
)
_REQUIRED_DELAYS = (
    'bridge_s',
    'tf_s',
    'controller_s',
    'perception_s',
    'mission_s',
)


def _launch_arg_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _load_common_params(common_params):
    if not os.path.isfile(common_params):
        raise RuntimeError('Common params file not found: %s' % common_params)

    with open(common_params, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    return data.get('/**', {}).get('ros__parameters', {})


def _load_launch_profile(common_params, profile_name):
    params = _load_common_params(common_params)
    profiles = params.get('launch_profiles')
    if not isinstance(profiles, dict):
        raise RuntimeError("Missing 'launch_profiles' block in %s" % common_params)

    profile = profiles.get(profile_name)
    if not isinstance(profile, dict):
        raise RuntimeError("Missing launch profile '%s' in %s" % (profile_name, common_params))

    missing = [name for name in _REQUIRED_FLAGS if name not in profile]
    if missing:
        raise RuntimeError(
            "Missing launch flag(s) in %s.%s: %s"
            % (profile_name, common_params, ', '.join(missing))
        )

    for name in _REQUIRED_FLAGS:
        if not isinstance(profile[name], bool):
            raise RuntimeError(
                "Launch flag '%s.%s' must be true/false in %s"
                % (profile_name, name, common_params)
            )

    delays = profile.get('delays')
    if not isinstance(delays, dict):
        raise RuntimeError("Missing launch delays in %s.%s" % (profile_name, common_params))

    missing_delays = [name for name in _REQUIRED_DELAYS if name not in delays]
    if missing_delays:
        raise RuntimeError(
            "Missing launch delay(s) in %s.%s: %s"
            % (profile_name, common_params, ', '.join(missing_delays))
        )

    for name in _REQUIRED_DELAYS:
        if float(delays[name]) < 0.0:
            raise RuntimeError(
                "Launch delay '%s.%s' must be >= 0 in %s"
                % (profile_name, name, common_params)
            )

    return profile, params


def _load_realsense_launch_arguments(params, common_params):
    realsense = params.get('realsense')
    if not isinstance(realsense, dict):
        raise RuntimeError("Missing 'realsense' block in %s" % common_params)

    required_args = (
        'enable_color',
        'enable_depth',
        'enable_infra1',
        'enable_infra2',
        'initial_reset',
    )
    missing = [name for name in required_args if name not in realsense]
    if missing:
        raise RuntimeError(
            "Missing RealSense launch argument(s) in %s: %s"
            % (common_params, ', '.join(missing))
        )

    return {name: _launch_arg_value(realsense[name]) for name in required_args}


def _load_px4_parameters(params, profile_name, common_params):
    px4 = params.get('px4')
    if not isinstance(px4, dict):
        raise RuntimeError("Missing 'px4' block in %s" % common_params)

    profile = px4.get(profile_name)
    if not isinstance(profile, dict):
        raise RuntimeError("Missing PX4 profile '%s' in %s" % (profile_name, common_params))

    required_args = (
        'fcu_url',
        'gcs_url',
        'tgt_system',
        'tgt_component',
        'fcu_protocol',
    )
    missing = [name for name in required_args if name not in profile]
    if missing:
        raise RuntimeError(
            "Missing PX4 parameter(s) in %s.%s: %s"
            % (profile_name, common_params, ', '.join(missing))
        )

    return {name: profile[name] for name in required_args}


def _load_lidar_launch(params, common_params):
    lidar = params.get('lidar')
    if not isinstance(lidar, dict):
        raise RuntimeError("Missing 'lidar' block in %s" % common_params)

    required_args = ('package', 'launch', 'rviz')
    missing = [name for name in required_args if name not in lidar]
    if missing:
        raise RuntimeError(
            "Missing LiDAR launch argument(s) in %s: %s"
            % (common_params, ', '.join(missing))
        )

    return (
        str(lidar['package']),
        str(lidar['launch']),
        {'rviz': _launch_arg_value(lidar['rviz'])},
    )


def _append_with_delay(actions, delay_s, launch_actions):
    if not actions:
        return
    if delay_s <= 0.0:
        launch_actions.extend(actions)
        return
    launch_actions.append(TimerAction(period=delay_s, actions=actions))


def _launch_setup(context, *args, **kwargs):
    profile_name = LaunchConfiguration('profile').perform(context)
    bringup_share = get_package_share_directory('uav_bringup')
    controller_share = get_package_share_directory('uav_controller')
    common_params = os.path.join(bringup_share, 'config', 'uav_common_params.yaml')
    profile, common = _load_launch_profile(common_params, profile_name)
    delays = profile['delays']

    bridge_params = os.path.join(
        get_package_share_directory('uav_px4_bridge'), 'config', 'px4_bridge_params.yaml')
    tf_params = os.path.join(
        get_package_share_directory('uav_tf_broadcaster'), 'config', 'tf_broadcaster_params.yaml')
    controller_params = os.path.join(controller_share, 'config', 'controller_params.yaml')
    mission_params = os.path.join(controller_share, 'config', 'mission_params.yaml')
    spf_params = os.path.join(controller_share, 'config', 'spf_params.yaml')
    spf_image_test_params = os.path.join(
        controller_share, 'config', 'spf_image_goal_test_params.yaml')
    ugv_params = os.path.join(controller_share, 'config', 'ugv_communicate_params.yaml')

    launch_actions = []

    if profile['lidar']:
        lidar_package, lidar_launch_file, lidar_args = _load_lidar_launch(
            common, common_params)
        lidar_launch = os.path.join(
            get_package_share_directory(lidar_package), 'launch', lidar_launch_file)
        launch_actions.append(GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_launch),
                    launch_arguments=lidar_args.items(),
                ),
            ],
        ))

    if profile['mavros']:
        mavros_pluginlists = os.path.join(
            get_package_share_directory('mavros'), 'launch', 'px4_pluginlists.yaml')
        mavros_config = os.path.join(
            get_package_share_directory('mavros'), 'launch', 'px4_config.yaml')
        px4_params = _load_px4_parameters(common, profile_name, common_params)
        launch_actions.append(Node(
            package='mavros', executable='mavros_node',
            namespace='mavros', output='screen',
            emulate_tty=True,
            parameters=[mavros_pluginlists, mavros_config, px4_params],
        ))

    if profile['realsense']:
        realsense_launch = os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        realsense_args = _load_realsense_launch_arguments(common, common_params)
        launch_actions.append(GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(realsense_launch),
                    launch_arguments=realsense_args.items(),
                ),
            ],
        ))

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

    perception_nodes = []
    if profile['spf']:
        perception_nodes.append(Node(
            package='uav_controller', executable='spf.py',
            name='spf_node', output='screen',
            emulate_tty=True, parameters=[spf_params],
        ))
    if profile['spf_image_test']:
        perception_nodes.append(Node(
            package='uav_controller', executable='spf_image_goal_test.py',
            name='spf_image_goal_test', output='screen',
            emulate_tty=True, parameters=[spf_image_test_params],
        ))
    if profile['ugv_tcp']:
        perception_nodes.append(Node(
            package='uav_controller', executable='ugv_communicate.py',
            name='ugv_communicate_node', output='screen',
            emulate_tty=True, parameters=[ugv_params],
        ))

    mission_nodes = []
    if profile['mission']:
        mission_nodes.append(Node(
            package='uav_controller', executable='mission_node',
            name='mission_node', output='screen',
            emulate_tty=True, parameters=[mission_params],
        ))

    _append_with_delay([bridge], float(delays['bridge_s']), launch_actions)
    _append_with_delay([tf_node], float(delays['tf_s']), launch_actions)
    _append_with_delay([controller], float(delays['controller_s']), launch_actions)
    _append_with_delay(perception_nodes, float(delays['perception_s']), launch_actions)
    _append_with_delay(mission_nodes, float(delays['mission_s']), launch_actions)

    return launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='sitl',
            description='Launch profile in uav_common_params.yaml: sitl or real',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
