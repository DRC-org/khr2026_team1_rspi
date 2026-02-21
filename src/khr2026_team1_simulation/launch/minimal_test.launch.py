import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_simulation = get_package_share_directory('khr2026_team1_simulation')
    urdf_path = os.path.join(pkg_simulation, 'urdf', 'robot.urdf.xacro')
    world_path = os.path.join(pkg_simulation, 'worlds', 'khr2026_field.sdf')

    # Robot Description
    doc = xacro.process_file(urdf_path)
    robot_description = {'robot_description': doc.toxml()}

    # Nodes
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'khr2026_robot', '-x', '0.8', '-y', '1.0', '-z', '0.1'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/khr2026_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/model/khr2026_robot/odometry', '/odom'),
        ],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
    ])
