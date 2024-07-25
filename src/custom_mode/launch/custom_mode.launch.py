# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
# This function is needed
def generate_launch_description():
    ld = LaunchDescription()


    # C++ nodes
    precision_land_cpp = Node(
        package="custom_mode",
        executable="custom_mode",
        parameters=[
                {"trajectory_type": "figure_8"},
            ]
    )


    ld.add_action(precision_land_cpp)


    return ld