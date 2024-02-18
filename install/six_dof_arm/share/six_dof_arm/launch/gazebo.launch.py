import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Include the Gazebo launch file. This starts Gazebo with the necessary ROS plugins
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )
    
    # Determine the path to the 'six_dof_arm' package
    package_path = os.path.join(
        get_package_share_directory('six_dof_arm'))
    
    # Define the path to the URDF file
    xacro_file = os.path.join(package_path,
                              'urdf',
                              'six_dof_arm.urdf')
    
    # Parse the URDF file and convert it to XML format
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    
    # Define a node for the robot state publisher, which publishes the state of the robot's joints
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # Define an action to load and start the joint state controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )
    
    # Define an action to load and start the arm controller
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )
    
    # Define a node to spawn the robot in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'six_dof_arm'],
                        output='screen')
    
    # Create the launch description with all the defined actions and event handlers
    return LaunchDescription([
        # Register an event handler to load controllers once the robot entity is spawned in Gazebo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])