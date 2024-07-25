# Import necessary modules from the launch and launch_ros packages
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# This function generates the launch description
def generate_launch_description():
    ld = LaunchDescription()

    # Declare the launch argument
    trajectory_type_arg = DeclareLaunchArgument(
        'trajectory_type',
        default_value='circle',
        description='Type of trajectory for the custom mode'
    )

    # C++ node for custom mode
    custom_mode_cpp = Node(
        package="custom_mode",
        executable="custom_mode",
        parameters=[
            {"trajectory_type": LaunchConfiguration('trajectory_type')}
        ]
    )

    # Add the argument and the node action to the launch description
    ld.add_action(trajectory_type_arg)
    ld.add_action(custom_mode_cpp)

    return ld
