"""Launch the free-floating spacecraft with closed-loop pose GNC."""

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
    gnc = Node(
        package='spacecraft_reaction_sim',
        executable='spacecraft_gnc',
        parameters=[{'use_sim_time': True}],
        output='screen')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(manual_launch)),
        TimerAction(period=10.0, actions=[gnc]),
    ])
