import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('host', default_value='192.0.2.1',
                          description='port for cerebri'),
    DeclareLaunchArgument('port', default_value='4242',
                          description='tcp port for cerebri'),
    DeclareLaunchArgument('log_level', default_value='error',
                          choices=['info', 'warn', 'error'],
                          description='log level'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():

    # Launch configurations
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')

    synapse_ros = Node(
        #prefix='xterm -e gdb --args',
        namespace='cerebri',
        package='synapse_ros',
        executable='synapse_ros',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
        remappings=[
            ('/cerebri/in/cmd_vel', '/cmd_vel')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        on_exit=Shutdown(),
        #prefix=['xterm -e gdb -ex=r --args'],
        )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(synapse_ros)
    return ld

