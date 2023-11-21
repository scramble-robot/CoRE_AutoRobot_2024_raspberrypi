from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scramble_auto_robot',
            executable='servo_node',
            name='servo0',
            parameters=[{'pin':12}]
        ),
        Node(
            package='scramble_auto_robot',
            executable='servo_node',
            name='servo1',
            parameters=[{'pin':13}]
        ),
        Node(
            package='scramble_auto_robot',
            executable='gpio_node',
        ),
        Node(
            package='scramble_auto_robot',
            executable='can_node',
        ),
    ])