from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    # -------------------------------------------------------------
    #                       DECLARATIONS
    # -------------------------------------------------------------

    # Package and path setup
    pkg_path = get_package_share_directory('skid6x6_description')
    urdf_path = Path(pkg_path) / 'urdf' / 'skid6x6.urdf'
    rviz_path = Path(pkg_path) / 'rviz' / 'skid6x6.rviz'
    world_path = '/home/balaji/edge_force_ws/src/skid6x6_description/worlds/world1.world'

    # Load URDF file content
    urdf_content = urdf_path.read_text()

    # Common simulation parameters
    sim_time_param = {'use_sim_time': True}
    robot_description_param = {'robot_description': urdf_content, 'use_sim_time': True}

    # -------------------------------------------------------------
    #                       NODE DEFINITIONS
    # -------------------------------------------------------------

    # 1. Gazebo process
    gazebo_process = ExecuteProcess(
        cmd=[
            "gazebo", "--verbose",
            "-s", "libgazebo_ros_factory.so",
            "-s", "libgazebo_ros_init.so",
            str(world_path)
        ],
        output="screen"
    )

    # 2. Spawn robot entity
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_skid6x6',
        arguments=[
            '-entity', 'skid6x6',
            '-file', str(urdf_path),
            '-x', '0', '-y', '0', '-z', '0.25'
        ],
        output='screen',
        parameters=[sim_time_param]
    )

    # 3. Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]
    )

    # 4. Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[sim_time_param]
    )

    # 5. laser merger nodes
    scan_merger = Node(
        package='scans_merger',
        executable='scans_merger',
        name='cloud_merger_node',
        output='screen',
        parameters=[{
            'destination_frame': 'base_link',
            'input_cloud_1': '/scan/front',
            'input_cloud_2': '/scan/front',
            'merged_cloud': '/merged_points'
        }]
    ),

    dual_laser_merger_node = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        name='dual_laser_merger_node',
        output='screen',
        parameters=[{
            'laser_1_topic': '/scan/front',
            'laser_2_topic': '/scan/rear',
            'merged_scan_topic': '/merged_points',
            'target_frame': 'lsc_mount',
            'laser_1_x_offset': 0.0,
            'laser_1_y_offset': 0.0,
            'laser_1_yaw_offset': 0.0,
            'laser_2_x_offset': -0.04,
            'laser_2_y_offset': 0.0,
            'laser_2_yaw_offset': 0.0,
            'tolerance': 0.01,
            'queue_size': 5,
            'angle_increment': 0.001,
            'scan_time': 0.067,
            'range_min': 0.01,
            'range_max': 25.0,
            'min_height': -1.0,
            'max_height': 1.0,
            'angle_min': -3.141592654,
            'angle_max': 3.141592654,
            'inf_epsilon': 1.0,
            'use_inf': True,
            'allowed_radius': 0.45,
            'enable_shadow_filter': True,
            'enable_average_filter': True
        }]
    )

    custom_point_merger = Node(
        package='skid6x6_description',
        executable='cloud_merger',
        name='cloud_merger',
        parameters=[{'use_sim_time': True}],
        output='screen'
    ),

    # 6. RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_path)],
        output='screen',
        parameters=[robot_description_param]
    )

    # -------------------------------------------------------------
    #                       EXECUTION ORDER
    # -------------------------------------------------------------
    return LaunchDescription([
        gazebo_process,
        spawn_robot,
        robot_state_publisher,
        joint_state_publisher,
        scan_merger,
        # dual_laser_merger_node,
        # custom_point_merger,
        rviz_node
    ])