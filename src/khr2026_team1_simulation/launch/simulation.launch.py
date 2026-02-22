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

    # オリジナル座標(x=0.8)でスポーン
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
            '/world/khr2026_field/model/khr2026_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/khr2026_robot/joint/lift_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/khr2026_robot/joint/left_finger_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/khr2026_robot/joint/right_finger_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '--ros-args', '--log-level', 'ERROR'
        ],
        remappings=[
            ('/model/khr2026_robot/cmd_vel', '/cmd_vel_inverted'),
            ('/model/khr2026_robot/odometry', '/odom'),
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

    cmd_vel_inverter = Node(
        package='khr2026_team1_simulation',
        executable='invert_cmd_vel.py',
        name='cmd_vel_inverter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_data)
    )

    # ターゲット座座標もオリジナルに復元
    tf_yagura_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['3.15', '2.0', '0.25', '0', '0', '0', 'map', 'yagura_1', '--ros-args', '--log-level', 'ERROR'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_data)
    )
    tf_yagura_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['3.15', '3.2', '0.25', '0', '0', '0', 'map', 'yagura_2', '--ros-args', '--log-level', 'ERROR'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_data)
    )
    tf_yagura_3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['3.15', '4.4', '0.25', '0', '0', '0', 'map', 'yagura_3', '--ros-args', '--log-level', 'ERROR'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_data)
    )
    tf_ring_supply = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.7', '6.2', '0.15', '0', '0', '0', 'map', 'ring_supply_1', '--ros-args', '--log-level', 'ERROR'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_data)
    )
    tf_supply_y = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['2.96', '0.6', '0.15', '0', '0', '0', 'map', 'supply_y', '--ros-args', '--log-level', 'ERROR'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_data)
    )
    tf_honmaru = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['3.25', '1.4', '0.4', '0', '0', '0', 'map', 'honmaru', '--ros-args', '--log-level', 'ERROR'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_data)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_data', default_value='true',
                              description='true: use simulation data, false: use real robot data'),
        robot_state_publisher, gazebo, spawn_entity, bridge, cmd_vel_inverter,
        nav2_bringup, scoring_node, hand_bridge, mission_control, gz_attachment,
        tf_yagura_1,
        tf_yagura_2,
        tf_yagura_3,
        tf_ring_supply,
        tf_supply_y,
        tf_honmaru
    ])
