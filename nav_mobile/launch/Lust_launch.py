#!/usr/bin/env  python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            namespace='joy_node',
            executable='joy_node',
            name='sim'
        ),
        Node(
            package='communication',
            namespace='joy_to_vel',
            executable='joy_2_cmd_vel',
            name='sim'
        ),
        Node(
            package='communication',
            namespace='subscriber',
            executable='my_subscriber_xbox',
            name='sim'
        )
        
    ])