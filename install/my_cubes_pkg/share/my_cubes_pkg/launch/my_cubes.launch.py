'''
This script launches the cubes in gazebo using a sdf file. 
'''

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_cubes_pkg')
    
    # SDF
    sdf_file_name = 'sdf/model.sdf'
    sdf = os.path.join(pkg_dir, 'models', sdf_file_name)
    
    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-entity', 'my-cubes', '-file', sdf],
                        output='screen')
    
    return LaunchDescription([spawn_entity])