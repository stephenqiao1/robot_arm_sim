"""
This script launches the cubes in gazebo using a sdf file. 
"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("my_cubes_pkg")

    # Spawn Green Cube
    sdf_green_cube = os.path.join(pkg_dir, "models", "sdf/green_cube.sdf")
    spawn_entity_green_cube = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "green_cube",
            "-file",
            sdf_green_cube,
            "-x",
            "0",
            "-y",
            "0.75",
            "-z",
            "0.05",
        ],
    )
    
    # Spawn Blue Cube
    sdf_blue_cube = os.path.join(pkg_dir, "models", "sdf/blue_cube.sdf")
    spawn_entity_blue_cube = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "blue_cube",
            "-file",
            sdf_blue_cube,
            "-x",
            "-0.5",
            "-y",
            "0.75",
            "-z",
            "0.05",
        ],
    )

    # Spawn Red Cube
    sdf_red_cube = os.path.join(pkg_dir, "models", "sdf/red_cube.sdf")
    spawn_entity_red_cube = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "red_cube",
            "-file",
            sdf_red_cube,
            "-x",
            "0.5",
            "-y",
            "0.75",
            "-z",
            "0.05",
        ],
    )

    return LaunchDescription([spawn_entity_green_cube, spawn_entity_blue_cube, spawn_entity_red_cube])
