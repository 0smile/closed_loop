from launch import LaunchDescription, launch_description
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), 
            ' ',
            PathJoinSubstitution([FindPackageShare('closed_loop'), 'urdf', 'closed_loop.xacro']),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros,"launch","gazebo.launch.py"))
    )

    gazebo_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-entity', 'closed_loop', '-topic', 'robot_description']
        )

    controllers_config = PathJoinSubstitution(
        [
            FindPackageShare("closed_loop"),
            "ros2_control_config",
            "robot_state_publisher.yaml",
        ]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    rviz_config_file = PathJoinSubstitution([FindPackageShare('closed_loop'), 'rviz', 'closed_loop.rviz'])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file]
    )

    nodes = [
        robot_state_pub_node,
        gazebo_spawner,
        load_joint_state_controller,
        rviz
    ]

    return LaunchDescription(
        nodes +  [gazebo_launch])