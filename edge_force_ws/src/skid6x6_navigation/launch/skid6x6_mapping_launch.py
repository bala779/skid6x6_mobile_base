from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    skid6x6_navigation = get_package_share_directory("skid6x6_navigation")

    main_param_dir = LaunchConfiguration(
        "main_param_dir",
        default=os.path.join(skid6x6_navigation, "config", "lidar_slam.yaml"),
    )
    ekf_param_file = os.path.join(skid6x6_navigation, "config", "ekf.yaml")

    # --- Launch Arguments ---
    declare_main_param = DeclareLaunchArgument(
        "main_param_dir",
        default_value=main_param_dir,
        description="Full path to lidar_slam parameter file"
    )

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="True",
        description="Use /clock as time source."
    )

    # --- EKF Node ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_param_file, {'use_sim_time': True}]
    )

    # --- Scan Matcher (Front LiDAR) ---
    scan_matcher = TimerAction(
        period=3.0,  
        actions=[
            Node(
                package="scanmatcher",
                executable="scanmatcher_node",
                parameters=[main_param_dir, {"use_sim_time": True}],
                remappings=[
                    ('/input_cloud', '/scan/front'),
                    ("/odom", "/odometry/filtered"),
                ],
                output="screen"
            )
        ]
    )

    # --- Graph-based SLAM (Back-end optimization) ---
    graph_based_slam = TimerAction(
        period=6.0, 
        actions=[
            Node(
                package="graph_based_slam",
                executable="graph_based_slam_node",
                parameters=[main_param_dir, {"use_sim_time": True}],
                output="screen"
            )
        ]
    )

    # --- RViz ---
    rviz_param_dir = LaunchConfiguration(
        'rviz_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'rviz',
            'mapping.rviz'
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_param_dir],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    return LaunchDescription([


        declare_main_param,
        declare_use_sim,

        ekf_node,
        scan_matcher,
        # graph_based_slam,
        # rviz
    ])
