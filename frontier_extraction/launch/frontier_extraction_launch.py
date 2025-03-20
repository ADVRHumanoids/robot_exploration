import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    #LaunchConfiguration
    frontier_extraction_dir = get_package_share_directory('frontier_extraction')

    use_sim_time = LaunchConfiguration('use_sim_time')
    frontier_2d = LaunchConfiguration('frontier_2d')

    #LaunchArgument
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value= 'true',
        description='Use simulation (Gazebo) clock if true',
    )
    
    declare_frontier_2d_cmd = DeclareLaunchArgument(
        'frontier_2d', 
        default_value='False', 
        description='Whether to use 2D or 3D frontiers'
    )

    # Launch Exploration BT
    frontier2d_extr_params_file = os.path.join(frontier_extraction_dir, 'config', 'frontier2d_extraction_params.yaml')
    frontier3d_extr_params_file = os.path.join(frontier_extraction_dir, 'config', 'frontier3d_extraction_params.yaml')

    #Launch Frontier Extraction Module
    frontier_2dseg_cmd = Node(
        condition=IfCondition(frontier_2d),
        package='frontier_extraction',
        executable='frontier_2d_extraction_node',
        name='frontier_2d_extraction_node',
        output='screen',
        parameters=[frontier2d_extr_params_file,
                    {'use_sim_time': use_sim_time}]
    )

    frontier_3dseg_cmd = Node(
        condition=UnlessCondition(frontier_2d),
        package='frontier_extraction',
        executable='frontier_3d_extraction_node',
        name='frontier_3d_extraction_node',
        output='screen',
        parameters=[frontier3d_extr_params_file,
                    {'use_sim_time': use_sim_time}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_frontier_2d_cmd)

    ld.add_action(frontier_2dseg_cmd)
    ld.add_action(frontier_3dseg_cmd)
    
    return ld