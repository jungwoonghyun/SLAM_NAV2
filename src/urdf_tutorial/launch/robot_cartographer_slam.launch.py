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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [get_package_share_directory('urdf_tutorial'), '/launch/robot_joint.launch.py']),
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [get_package_share_directory('ydlidar_ros2_driver'), '/launch/ydlidar_launch.py']),
        # ),

        Node(
            package = 'cartographer_ros',
            executable = 'cartographer_node',
            parameters = [{'use_sim_time': use_sim_time}],
            arguments = [
                '-configuration_directory', 'robot_ws/src/urdf_tutorial/config',
                '-configuration_basename', 'robot_cartographer_slam.lua'],
            remappings = [
                ('echoes', 'horizontal_laser_2d')],
            output = 'screen'),

        Node(
            package = 'cartographer_ros',
            executable = 'cartographer_occupancy_grid_node',
            parameters = [
                {'use_sim_time': True}],
            arguments = [
                '-resolution','0.05',
                '-publish_period_sec','0.25']),

        Node(
            package='tf2_ros',
            namespace='scan_to_map',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "0", "map", "odom", "20"],
            ),

            # rviz2

    ])
