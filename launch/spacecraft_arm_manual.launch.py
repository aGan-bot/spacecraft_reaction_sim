"""Launch a free-floating six-axis geometric arm for manual effort control."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('spacecraft_reaction_sim')
    model_path = PathJoinSubstitution([package_share, 'urdf', 'spacecraft_arm.urdf.xacro'])
    world_path = PathJoinSubstitution([package_share, 'worlds', 'space_world.sdf'])
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    robot_description = Command(['xacro ', model_path])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
        ])),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
    )
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True,
        }], output='screen')
    spawn_model = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'spacecraft_arm', '-z', '2'],
        output='screen')
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')
    odometry_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/model/spacecraft_arm/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen')
    actuator_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/spacecraft_arm/command/duty_cycle@actuator_msgs/msg/Actuators]gz.msgs.Actuators'], output='screen')
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_joint_state_broadcaster', '-c', '/controller_manager'], output='screen')
    effort_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_effort_controller', '-c', '/controller_manager'], output='screen')
    wheel_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['reaction_wheel_effort_controller', '-c', '/controller_manager'],
        output='screen')

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        clock_bridge,
        odometry_bridge,
        actuator_bridge,
        TimerAction(period=2.0, actions=[spawn_model]),
        TimerAction(period=5.0, actions=[joint_state_broadcaster]),
        TimerAction(period=7.0, actions=[effort_controller]),
        TimerAction(period=8.0, actions=[wheel_controller]),
    ])
