from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    # Location of teleop launch file (https://github.com/ros2/teleop_twist_joy/blob/foxy/launch/teleop-launch.py)
    launch_file = [get_package_share_directory('teleop_twist_joy'), '/launch/teleop-launch.py']
    # Location of our own config
    config_file = '../config/xbox.config.yaml'

    return LaunchDescription([
        # Launch another launch file and supply different config as argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'config_filepath': config_file}.items(),
        ),
    ])
