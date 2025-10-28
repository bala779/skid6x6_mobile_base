from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Package Directories ---
    pkg_skid6x6_nav = get_package_share_directory('skid6x6_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # --- File Paths ---
    map_yaml = os.path.join(pkg_skid6x6_nav, 'maps', 'map.yaml')
    nav2_params = os.path.join(pkg_skid6x6_nav, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_skid6x6_nav, 'rviz', 'nav2_view.rviz')

    # --- Launch Arguments ---
    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=map_yaml,
        description='Full path to map YAML file to load.'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time if true.'
    )

    # --- SLAM Toolbox Node ---
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True'
        }.items()
    )

    # --- Navigation2 Bringup ---
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'True',
            'params_file': nav2_params
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_map_yaml,
        declare_use_sim_time,

        slam_toolbox_launch,
        nav2_bringup_launch,
        rviz_node,
    ])
