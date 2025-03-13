from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from subprocess import check_output


def generate_launch_description():
    # Declare launch arguments
    centauro_gazebo_dir = get_package_share_directory('centauro_gazebo')
    centauro_urdf_dir = get_package_share_directory('centauro_urdf')
    centauro_srdf_dir = get_package_share_directory('centauro_srdf')
    
    xacro_urdf_gz  =  os.path.join(centauro_gazebo_dir, 'urdf', 'centauro_gazebo.urdf.xacro')
    robot_description_gz = check_output(f'xacro {xacro_urdf_gz}', shell=True).decode()

    xacro_urdf  =  os.path.join(centauro_urdf_dir, 'urdf', 'centauro.urdf.xacro')
    robot_description = check_output(f'xacro {xacro_urdf} velodyne:=true realsense:=true gz_odometry:=true', shell=True).decode()

    xacro_srdf  =  os.path.join(centauro_srdf_dir, 'srdf', 'centauro.srdf.xacro')
    robot_description_semantic = check_output(f'xacro {xacro_srdf} velodyne:=true realsense:=true gz_odometry:=true', shell=True).decode()

    gz_odometry = LaunchConfiguration('gz_odometry')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_gz_odometry_cmd = DeclareLaunchArgument(
       'gz_odometry',
       default_value='True',
       description='Gazebo Odometry')

    arg_launch_arguments = [
        DeclareLaunchArgument('gazebo', default_value='true'),
        DeclareLaunchArgument('xbot2', default_value='true'),
        DeclareLaunchArgument('xbot2_config', default_value=os.path.join(get_package_share_directory('centauro_config'), 'centauro_basic.yaml')),
        # DeclareLaunchArgument('realsense', default_value='false'),
        # DeclareLaunchArgument('velodyne', default_value='true'),
        # DeclareLaunchArgument('gz_odometry', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(get_package_share_directory('centauro_gazebo'), 'world/centauro.world'))
    ]

    # Construct `gz_args` with conditional '-r' based on `paused`
    gz_args = [
        LaunchConfiguration('world_file'),
        TextSubstitution(text=' '),
        TextSubstitution(text='-v ') if LaunchConfiguration('verbose') == 'true' else TextSubstitution(text=''),
        TextSubstitution(text='-s ') if LaunchConfiguration('gui') == 'false' else TextSubstitution(text=''),
        TextSubstitution(text='-r') if UnlessCondition(LaunchConfiguration('paused')) else TextSubstitution(text='')
    ]

    # Robot description publisher node
    gazedo_odom_publisher_node = Node(
        condition=IfCondition(gz_odometry),
        package='gazebo_odom',
        executable='sim_odom_connect',
        name='sim_odom_connect',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    description_publisher_node = Node(
        package='xbot2_ros',
        executable='robot_description_publisher',
        name='robot_description_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Gazebo group
    gazebo_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items()
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='urdf_spawner',
            parameters=[{'string': robot_description_gz,
                         'z': 1.0}]
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                #'/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/D435i_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/D435i_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/D435i_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/D435i_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/centauro/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
            ],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])

    # Xbot2 process
    xbot2_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('xbot2')),
        cmd=[
            'xbot2-core', '-V', '--hw', 'sim', '--simtime',
            '--config', LaunchConfiguration('xbot2_config'), '--'
        ],
        output='screen'
    )

    # RViz node
    # rviz_node = Node(
    #     condition=IfCondition(LaunchConfiguration('rviz')),
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz',
    #     output='screen',
    #     arguments=['-d', os.path.join(get_package_share_directory('concert_gazebo'), 'rviz/concert_sensors.rviz')],
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )

    # Create and return launch description
    return LaunchDescription(arg_launch_arguments + [
        declare_gz_odometry_cmd,
        declare_use_sim_time_cmd,
        
        description_publisher_node,
        gazedo_odom_publisher_node,
        gazebo_group,
        xbot2_process#,
        #rviz_node
    ])