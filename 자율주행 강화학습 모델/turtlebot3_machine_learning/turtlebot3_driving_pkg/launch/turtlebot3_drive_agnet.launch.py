from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'launch',
                'robot.launch.py'
            )
        )
    )

    drive_agent = Node(
        package='turtlebot3_driving_pkg',
        executable='drive_agent',
        name='drive_agent',
        output='screen'
    )

    return LaunchDescription([
        bringup,
        drive_agent
    ])