from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_filepath = PathJoinSubstitution([
        FindPackageShare('robotone_joystick'),
        'config',
        'xbox_config.yaml'
    ])

    logger = LaunchConfiguration('log_level')

    # Declare launch arguments for flexibility
    ld = LaunchDescription([
        DeclareLaunchArgument(
            name='config_filepath',
            default_value=config_filepath,
            description='Path to the joystick configuration file'
        ),

        DeclareLaunchArgument(
            name='log_level',
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
