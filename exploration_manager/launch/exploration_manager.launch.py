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

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    #LaunchConfiguration
    exploration_manager_dir = get_package_share_directory('exploration_manager')
    frontier_extraction_dir = get_package_share_directory('frontier_extraction')

    use_sim_time = LaunchConfiguration('use_sim_time')
    bt_file = LaunchConfiguration('bt_file')
    frontier_2d = LaunchConfiguration('frontier_2d')

    #LaunchArgument
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value= 'true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_bt_file_cmd = DeclareLaunchArgument(
        'bt_file',
        default_value= os.path.join(exploration_manager_dir, 'behavior_trees', 'exploration_bt.xml'),
        description='File of the BT xml',
    )

    declare_frontier_2d_cmd = DeclareLaunchArgument(
        'frontier_2d', 
        default_value='False', 
        description='Whether to use 2D or 3D frontiers'
    )

    # Launch Exploration BT
    exploration_params_file = os.path.join(exploration_manager_dir, 'config', 'exploration_config_centauro.yaml')

    exploration_main_cmd = Node(
        package='exploration_manager',
        executable='exploration_main',
        name='exploration_main',
        output='screen',
        parameters=[exploration_params_file,
                    {'bt_file'   : bt_file}]
    )

    #Launch Frontier Extraction Module
    frontier_seg_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(frontier_extraction_dir, 'launch', 'frontier_extraction_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'frontier_2d': frontier_2d,
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_bt_file_cmd)
    ld.add_action(declare_frontier_2d_cmd)

    ld.add_action(exploration_main_cmd)
    ld.add_action(frontier_seg_cmd)
    
    return ld