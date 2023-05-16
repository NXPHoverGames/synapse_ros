import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('host', default_value='192.0.2.2',
                          description='port for cerebri'),
    DeclareLaunchArgument('port', default_value='4242',
                          description='tcp port for cerebri'),
]


def generate_launch_description():

    # Launch configurations
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')

    synapse_ros = Node(
        package='synapse_ros',
        executable='synapse_ros',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port')
        }],
        output='screen')

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(synapse_ros)
    return ld

