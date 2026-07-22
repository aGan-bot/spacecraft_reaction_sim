"""Launch the free-floating spacecraft with closed-loop pose GNC."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('spacecraft_reaction_sim')
    manual_launch = PathJoinSubstitution([
        package_share, 'launch', 'spacecraft_arm_manual.launch.py'])
    gnc = Node(
        package='spacecraft_reaction_sim',
        executable='spacecraft_gnc',
        parameters=[{
            'use_sim_time': True,
            'attitude_kp': LaunchConfiguration('attitude_kp'),
            'attitude_kd': LaunchConfiguration('attitude_kd'),
            'max_wheel_torque': LaunchConfiguration('max_wheel_torque'),
        }],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('attitude_kp', default_value='1.2'),
        DeclareLaunchArgument('attitude_kd', default_value='2.0'),
        DeclareLaunchArgument('max_wheel_torque', default_value='1.5'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(manual_launch)),
        TimerAction(period=10.0, actions=[gnc]),
    ])
