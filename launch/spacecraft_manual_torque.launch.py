"""Launch the free-floating spacecraft with manual effort control only."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    main_launch = PathJoinSubstitution([
        FindPackageShare('spacecraft_reaction_sim'),
        'launch',
        'spacecraft_reaction.launch.py',
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch),
            launch_arguments={'run_experiment': 'false'}.items(),
        ),
    ])
