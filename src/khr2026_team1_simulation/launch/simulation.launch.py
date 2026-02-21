import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('khr2026_team1_simulation')
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'khr2026_field.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_data = LaunchConfiguration('use_sim_data', default='true')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', xacro_file])
        }],
        arguments=["--ros-args", "--log-level", "ERROR"]
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
        arguments=['-topic', 'robot_description', '-name', 'khr2026_robot', '-x', '0.8', '-y', '1.0', '-z', '0.1', '--ros-args', '--log-level', 'ERROR'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/khr2026_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/khr2026_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/khr2026_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/khr2026_field/model/khr2026_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/khr2026_robot/joint/lift_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/khr2026_robot/joint/left_finger_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/khr2026_robot/joint/right_finger_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '--ros-args', '--log-level', 'ERROR'
        ],
        remappings=[
            ('/model/khr2026_robot/cmd_vel', '/cmd_vel_nav'),
            ('/model/khr2026_robot/odometry', '/odom'),
            ('/model/khr2026_robot/tf', '/tf'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_control'), 'launch', 'robot_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map_mode': 'true',
            'use_sim_data': use_sim_data,
        }.items()
    )

    scoring_node = Node(
        package='robot_control',
        executable='scoring_node.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["--ros-args", "--log-level", "ERROR"],
        condition=IfCondition(use_sim_data)
    )
    hand_bridge = Node(
        package='robot_control',
        executable='hand_bridge_node.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["--ros-args", "--log-level", "ERROR"],
        condition=IfCondition(use_sim_data)
    )
    mission_control = Node(
        package='robot_control',
        executable='mission_control_node.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["--ros-args", "--log-level", "ERROR"]
    )
    gz_attachment = Node(
        package='robot_control',
        executable='gz_attachment_node.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["--ros-args", "--log-level", "ERROR"],
        condition=IfCondition(use_sim_data)
    )

    # ---------------------------------------------------------------------------
    # Static TF for scoring_node: yagura / ring supply positions in map frame
    # ---------------------------------------------------------------------------
    # These let scoring_node detect yagura_*/ring_* frames via /tf
    tf_yagura_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['2.96', '0.6', '0.125', '0', '0', '0', 'map', 'yagura_1', '--ros-args', '--log-level', 'ERROR'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_data)
    )
    tf_yagura_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['5.0', '2.0', '0.125', '0', '0', '0', 'map', 'yagura_2', '--ros-args', '--log-level', 'ERROR'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_data)
    )
    tf_ring_supply = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.7', '6.2', '0.0', '0', '0', '0', 'map', 'ring_supply_1', '--ros-args', '--log-level', 'ERROR'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_data)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_data', default_value='true',
                              description='true: use simulation data, false: use real robot data'),
        robot_state_publisher, gazebo, spawn_entity, bridge,
        nav2_bringup, scoring_node, hand_bridge, mission_control, gz_attachment,
        tf_yagura_1, tf_yagura_2, tf_ring_supply,
    ])
