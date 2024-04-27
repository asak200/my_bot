from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen',
            arguments=['diff_cont', 'joint_broad'],
        ),
    ])