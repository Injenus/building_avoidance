"""Полный запуск миссии: Gazebo + ArduPilot SITL + MAVROS + мост + планировщик.

Цель задаётся аргументами target_x / target_y, править код не требуется:
    ros2 launch avoidance_sim mission.launch.py target_x:=20.0 target_y:=80.0

Сервер Gazebo и GUI поднимаются раздельно: сенсоры (gpu_lidar) считаются
софтверным ogre2, потому что в виртуальной машине ogre1 их не отрисовывает,
а интерфейс остаётся на аппаратном ogre1 ради скорости.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = get_package_share_directory('avoidance_sim')
    plan = get_package_share_directory('avoidance_planner')

    world_default = os.path.join(sim, 'worlds', 'building.sdf')
    params_default = os.path.join(plan, 'config', 'mission.yaml')
    sitl_parm = os.path.join(sim, 'config', 'sitl.parm')

    res = os.pathsep.join([os.path.join(sim, 'models'), os.path.join(sim, 'worlds')])
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        res += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']

    world = LaunchConfiguration('world')
    params = LaunchConfiguration('params_file')
    gui = LaunchConfiguration('gui')

    num = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '2', '-r', '-s', '--render-engine', 'ogre2', world],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        output='screen')

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '--render-engine', 'ogre'],
        condition=IfCondition(gui), output='screen')

    sitl = ExecuteProcess(
        cmd=['sim_vehicle.py', '-v', 'ArduCopter', '-f', 'gazebo-iris',
             '--model', 'JSON', '--add-param-file=' + sitl_parm,
             '--no-rebuild'],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')),
        launch_arguments={'fcu_url': 'udp://:14550@'}.items())

    planner = Node(
        package='avoidance_planner', executable='planner', output='screen',
        parameters=[params, {'target_x': num('target_x'),
                             'target_y': num('target_y'),
                             'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('target_x', default_value='0.0',
                              description='Цель, восток (ENU), м'),
        DeclareLaunchArgument('target_y', default_value='60.0',
                              description='Цель, север (ENU), м'),
        DeclareLaunchArgument('world', default_value=world_default),
        DeclareLaunchArgument('params_file', default_value=params_default),
        DeclareLaunchArgument('gui', default_value='true'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', res),

        gz_server,
        TimerAction(period=4.0, actions=[gz_gui]),
        TimerAction(period=7.0, actions=[sitl]),
        TimerAction(period=9.0, actions=[bridge]),
        TimerAction(period=25.0, actions=[mavros]),
        TimerAction(period=32.0, actions=[planner]),
    ])
