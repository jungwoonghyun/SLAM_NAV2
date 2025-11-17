import os
import launch

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode

def generate_launch_description():

    ld = launch.launchDescription()

    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ydlidar_ros2_driver_node'

    rviz_config_file = os.path.join(get_package_share_directory('clustering_cpp'), 'rviz', 'config.rviz')
    rviz_node = Node(
        name = 'rviz2',
        package = 'rviz2',
        executable = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config_file],
    )

    clustering_wagon = Node(
        package='clustering_cpp',
        executable='clustering_cpp',
        name='clustering_wagon',
        output='screen',
    )

    clustering_robot = Node(
        package='clustering_cpp',
        executable='clustering_cpp',
        name='clustering_robot',
        output='screen',
    )

    clustering_nav_robot = Node(
        package='clustering_cpp',
        executable='clustering_cpp',
        name='clustering_nav_robot',
        output='screen',
    )

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'TminiPro.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','2', '0', '0', '1','base_link','laser_frame'],
    )

    ld.add_action( rviz_node )
    ld.add_action( clustering_wagon )
    ld.add_action( clustering_robot )
    ld.add_action( params_declare )
    ld.add_action( driver_node )
    ld.add_action( tf2_node )

    return ld