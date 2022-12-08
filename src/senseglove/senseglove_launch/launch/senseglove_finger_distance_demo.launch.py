import os
import sys

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='calibration_mode',
            default_value='nothing',
            description='Choose the calibration mode of the senseglove; nothing, minimum, normalized'
        ),
        launch_ros.actions.Node(
            package='senseglove_finger_distance',
            executable='senseglove_finger_distance_node',
            name='senseglove_finger_distance_left',
            args=[0, LaunchConfiguration("calibration_mode")]

        ),
        launch_ros.actions.Node(
            package='senseglove_finger_distance',
            executable='senseglove_finger_distance_node',
            name='senseglove_finger_distance_right',
            args=[1, LaunchConfiguration("calibration_mode")]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
