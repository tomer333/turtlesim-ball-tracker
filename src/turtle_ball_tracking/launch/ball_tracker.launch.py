from __future__ import annotations

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            output='screen',
        ),

        Node(
            package='turtle_ball_tracking',
            executable='ball_image_publisher',
            output='screen',
        ),
        Node(
            package='turtle_ball_tracking',
            executable='ball_tracker',
            output='screen',
        ),
    ])
