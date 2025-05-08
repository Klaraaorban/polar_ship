from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get shared directories
    polar_ship_dir = get_package_share_directory('polar_ship')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Full paths to config and map
    nav2_params_file = os.path.join(polar_ship_dir, 'config', 'nav2_params.yaml')
    map_yaml_file = os.path.join(polar_ship_dir, 'rtab_maps', 'first_rtabmap.yaml')  # <--- Adjust filename if different

    return LaunchDescription([
        # Include Nav2 bringup launch with required arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params_file,
                'map': map_yaml_file,
            }.items()
        )
    ])
