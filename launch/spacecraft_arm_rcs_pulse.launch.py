"""Launch the arm and fire a selected off-centre +X RCS pulse."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("spacecraft_reaction_sim")
    manual_launch = PathJoinSubstitution([
        package_share, "launch", "spacecraft_arm_manual.launch.py"])
    pulse = Node(
        package="spacecraft_reaction_sim", executable="thruster_pulse",
        parameters=[{"use_sim_time": True, "duration_sec": LaunchConfiguration("duration_sec"), "actuator_index": LaunchConfiguration("actuator_index"), "command_rate_hz": 20.0}],
        output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("actuator_index", default_value="0"),
        DeclareLaunchArgument("duration_sec", default_value="0.5"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(manual_launch)),
        TimerAction(period=6.0, actions=[pulse]),
    ])
