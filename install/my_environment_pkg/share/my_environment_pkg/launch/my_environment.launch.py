'''
This is the main launch file for the environment simulation

Here start the simulation and call the previously developed launch of both cubes and robot arm
and spawn them together in a Gazebo.

my_robot_pkg --> my_robot_gazebo_controller.launch.py
my_cubes_pkg --> my_cubes.launch.py
'''

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    my_cubes_files = get_package_share_directory('my_cubes_pkg')
    my_robot_files = get_package_share_directory('my_robot_pkg')
    my_environments_files = get_package_share_directory('my_environment_pkg')
    
    # Start robot and controller
    panda_robot = IncludeLaunchDescription(PythonLaunchDescriptionSource(my_robot_files + '/launch/my_robot_gazebo_controller.launch.py'))
    
    # Start cubes spawn
    cubes_spawn = IncludeLaunchDescription(PythonLaunchDescriptionSource(my_cubes_files + '/launch/my_cubes.launch.py'))
    
    # Start Gazebo
    world_file_name = 'my_world.world'
    world = os.path.join(get_package_share_directory('my_environment_pkg'), 'worlds', world_file_name)
    gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'], output='screen')
    
    # Node
    
    ld = LaunchDescription()
    
    ld.add_action(panda_robot)
    ld.add_action(cubes_spawn)
    ld.add_action(gazebo_node)
    
    return ld