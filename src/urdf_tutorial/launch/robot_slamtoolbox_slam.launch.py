from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('urdf_tutorial'), '/launch/robot_joint.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('ydlidar_ros2_driver'), '/launch/ydlidar_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('slam_toolbox'), '/launch/online_async_launch.py']),
        ),


    ])
