"""Launch the six-axis spacecraft arm with reaction-wheel attitude hold."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('spacecraft_reaction_sim')
    manual_launch = PathJoinSubstitution([
        package_share, 'launch', 'spacecraft_arm_manual.launch.py'])
    attitude_hold = Node(
        package='spacecraft_reaction_sim', executable='attitude_hold',
        parameters=[{'use_sim_time': True, 'kp': 0.4, 'kd': 1.0,
                     'max_wheel_torque': 0.5}],
        output='screen')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(manual_launch)),
        TimerAction(period=10.0, actions=[attitude_hold]),
    ])
