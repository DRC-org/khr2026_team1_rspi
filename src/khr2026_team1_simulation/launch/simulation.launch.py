import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('khr2026_team1_simulation')
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'khr2026_field.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={'gz_args': [f'-r -s {world_file}']}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'khr2026_robot', '-x', '0.3', '-y', '0.5', '-z', '0.1'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/khr2026_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/khr2026_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/khr2026_robot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/khr2026_field/model/khr2026_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/khr2026_robot/joint/lift_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/khr2026_robot/joint/left_finger_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/khr2026_robot/joint/right_finger_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        remappings=[
            ('/model/khr2026_robot/cmd_vel', '/cmd_vel'),
            ('/model/khr2026_robot/odometry', '/odom'),
            ('/model/khr2026_robot/pose', '/tf'),
        ],
        output='screen'
    )

    scoring_node = Node(package='robot_control', executable='scoring_node.py', output='screen')
    hand_bridge = Node(package='robot_control', executable='hand_bridge_node.py', output='screen')
    mission_control = Node(package='robot_control', executable='mission_control_node.py', output='screen')
    gz_attachment = Node(package='robot_control', executable='gz_attachment_node.py', output='screen')

    return LaunchDescription([
        robot_state_publisher, gazebo, spawn_entity, bridge, 
        scoring_node, hand_bridge, mission_control, gz_attachment
    ])
