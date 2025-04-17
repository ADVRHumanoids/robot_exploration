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
    define_inspection_goals_dir = get_package_share_directory('define_inspection_goals')

    use_sim_time = LaunchConfiguration('use_sim_time')

    #LaunchArgument
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value= 'true',
        description='Use simulation (Gazebo) clock if true',
    )

    inspection_params_file = os.path.join(define_inspection_goals_dir, 'config', 'inspection_goals_params.yaml')

    #Launch Frontier Extraction Module
    define_inspect_goals_cmd = Node(
        package='define_inspection_goals',
        executable='inspection_goals_srv_node',
        name='inspection_goals_srv_node',
        output='screen',
        parameters=[inspection_params_file,
                    {'use_sim_time': use_sim_time}]
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(define_inspect_goals_cmd)
    
    return ld