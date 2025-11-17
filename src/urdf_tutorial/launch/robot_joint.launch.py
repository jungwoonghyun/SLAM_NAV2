import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'urdf_tutorial'

    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'E_motion_rooty.xacro')
    robot_description = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description.toxml(), 'use_sim_time': False}

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        #remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    # fake driver
    joint_driver = Node(
        package='urdf_tutorial',
        executable='joint_driver',
        output='screen',
        parameters=[],
    )

    # odometry publisher
    odometry_publisher = Node(
        package='car_odom',
        executable='car_odom',
        output='screen',
        parameters=[],
        #remappings=[('/odom', 'odom')],
    )

    # rviz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d','/home/robot/robot_ws/src/urdf_tutorial/config/robot_follow.rviz','use_sim_time','False'],
    )

    # scan_to_map
    base_scan = Node(
        package='tf2_ros',
        namespace='scan_to_map',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "map", "scan"],
        )

    return LaunchDescription(
        [
            rsp,
            joint_driver,
            odometry_publisher,
            #rviz,
            #base_scan,
        ]
    )
