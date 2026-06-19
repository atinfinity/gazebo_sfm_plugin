#!/usr/bin/env python3
# Launch the cafe SFM-pedestrians world in Gazebo Harmonic via ros_gz_sim.

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_sfm_plugin')
    pkg_prefix = get_package_prefix('gazebo_sfm_plugin')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # Bundled actor/animation meshes (walk*.dae, stand.dae, ...).
    models_path = os.path.join(pkg_share, 'models')
    # Installed plugin shared library directory.
    plugin_path = os.path.join(pkg_prefix, 'lib')

    default_world = os.path.join(pkg_share, 'worlds', 'cafe3.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r -v 3'],
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the world file to load.',
        ),
        # Let Gazebo find the bundled meshes and the plugin library.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
        gz_sim,
    ])
