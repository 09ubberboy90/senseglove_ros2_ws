import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'senseglove_hardware_interface'), 'launch/hardware.launch.py')
            ),
            launch_arguments={
                'robot': 'dk1',
                'is_right': 'false',
                "nr_of_gloves": "0"
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'senseglove_hardware_interface'), 'launch/hardware.launch.py')
            ),
            launch_arguments={
                'robot': 'dk1',                
                'is_right': 'true',
                "nr_of_gloves": "1"
                
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
