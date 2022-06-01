from sys import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='delivery_api_server',
            executable='delivery_api_server',
            name='delivery_api_server',
            output="screen",
            parameters=[]
        ),
    ])