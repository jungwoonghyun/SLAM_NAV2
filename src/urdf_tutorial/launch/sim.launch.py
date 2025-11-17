import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "urdf_tutorial"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot_3.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
    )

    # odometry publisher
    odometry_publisher = Node(
        package='car_odom',
        executable='car_odom',
        output='screen',
        parameters=[],
        #remappings=[('/odom', 'odom')],
    )

    # fake driver
    fake_driver = Node(
        package='urdf_tutorial',
        executable='joint_driver',
        output='screen',
        parameters=[],
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot"],
        output="screen",
    )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            fake_driver,
            #odometry_publisher,
            spawn_entity,
        ]
    )
