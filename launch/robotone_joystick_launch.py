import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    joy_config = LaunchConfiguration('joy_config')
    config_filepath = LaunchConfiguration('config_filepath')
    logger = LaunchConfiguration('log_level')

    # Declare launch arguments for flexibility
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'joy_config', 
            default_value='xbox', 
            description='Joystick configuration (e.g., xbox, ps4)'
        ),
        DeclareLaunchArgument(
            'config_filepath', 
            default_value=[
                TextSubstitution(text=os.path.join(
                    get_package_share_directory('robotone_joystick'), 'config', '')),
                joy_config, 
                TextSubstitution(text='_config.yaml')
            ],
            description='Path to the joystick configuration file'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (info or debug)'
        )
    ])

    # Define the joystick node with remappings and parameters
    joy_node = Node(
        package='robotone_joystick',
        executable='robotone_joystick_node',
        name='robotone_joystick_node',
        output='screen',
        arguments=['--ros-args', '--log-level', logger],
        remappings=[('/joy', '/robotone_joy')],
        parameters=[config_filepath]
    )

    # Add the node to the launch description
    ld.add_action(joy_node)

    return ld
