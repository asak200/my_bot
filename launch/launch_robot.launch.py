import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot'

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', "use_ros2_control":'false'}.items()
    )
    
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params = os.path.join(
        get_package_share_directory('my_bot'),
        'config',
        'my_controllers.yaml'
        )

    diff_drive_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen',
            arguments=['diff_cont', 'joint_broad'],
        )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    controller_manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, 
                        controller_params]
        )
    delay_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    


    # Launch them all!
    return LaunchDescription([
        rsp,
        delay_controller_manager,
        delayed_diff_drive_spawner
    ])






