from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_avoidance',
            executable='avoid_obstacle',
            name='avoid_obstacle',
            output='screen'
        )
    ])
