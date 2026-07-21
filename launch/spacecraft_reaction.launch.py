"""Launch a zero-gravity, free-floating spacecraft reaction experiment."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run_experiment = LaunchConfiguration('run_experiment')
    package_share = FindPackageShare('spacecraft_reaction_sim')
    model_path = PathJoinSubstitution([package_share, 'urdf', 'spacecraft_reaction.urdf.xacro'])
    world_path = PathJoinSubstitution([package_share, 'worlds', 'space_world.sdf'])
    robot_description = Command(['xacro ', model_path])
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'gz_args': ['-r ', world_path]}.items())
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True,
        }],
        output='screen')
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'spacecraft_reaction',
                   '-x', '0', '-y', '0', '-z', '2'],
        output='screen')
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')
    odometry_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/spacecraft_reaction/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen')
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen')
    effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wrist_effort_controller', '-c', '/controller_manager'],
        output='screen')
    experiment = Node(
        package='spacecraft_reaction_sim',
        executable='reaction_experiment',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(run_experiment),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_experiment', default_value='true',
            description='Start the automatic +1, 0, -1 N m torque experiment.'),
        gazebo,
        robot_state_publisher,
        clock_bridge,
        odometry_bridge,
        TimerAction(period=2.0, actions=[spawn_model]),
        TimerAction(period=5.0, actions=[joint_state_broadcaster]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[effort_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=effort_controller,
                on_exit=[experiment],
            )
        ),
    ])
