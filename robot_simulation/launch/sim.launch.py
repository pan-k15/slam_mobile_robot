import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Use empty world from ros_gz_sim package
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    empty_world_file = os.path.join(gz_sim_pkg, 'worlds', 'empty.sdf')

    # Other file paths
    urdf_file = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.urdf')
    rviz_config = os.path.join(get_package_share_directory('robot_simulation'), 'rviz', 'urdf_config2.rviz')
    ekf_config = os.path.join(get_package_share_directory('robot_simulation'), 'config', 'ekf.yaml')

    # Declare launch arguments
    declare_world = DeclareLaunchArgument('world_fname', default_value=empty_world_file, description='Absolute path to Gazebo world file')
    declare_gui = DeclareLaunchArgument('gui', default_value='true', description='Set to false to run gzserver headless')
    declare_rviz = DeclareLaunchArgument('rvizconfig', default_value=rviz_config, description='Absolute path to RViz config file')

    # Launch configurations
    world = LaunchConfiguration('world_fname')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Include gzserver launch
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v4', world]
        }.items()
    )

    # Include gzclient launch (only if gui is true)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v4'
        }.items(),
        condition=IfCondition(gui)
    )

    # Spawn robot entity in simulation
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot', '-file', urdf_file, '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # Include robot_state_publisher launch (assumed to be in your robot_simulation package)
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_simulation'), 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # EKF node for localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_world,
        declare_gui,
        declare_rviz,
        gzserver,
        gzclient,
        spawn_entity,
        rsp_launch,
        rviz,
        ekf_node
    ])
