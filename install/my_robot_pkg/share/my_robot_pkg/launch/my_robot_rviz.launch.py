"""
Simple launch to visualize the robot arm in RViz
This file opens RViz, visualize the robot and open a GUI interface to move the joints
"""

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    xacro_file = (
        get_package_share_directory("my_robot_pkg")
        + "/description"
        + "/xacro/"
        + "panda.urdf.xacro"
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("my_robot_pkg") + "/rviz/view_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)}],
    )

    # Joint State Publisher
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        name="joint_state_publisher_gui",
    )

    return LaunchDescription([robot_state_publisher, joint_state_publisher_gui, rviz_node])