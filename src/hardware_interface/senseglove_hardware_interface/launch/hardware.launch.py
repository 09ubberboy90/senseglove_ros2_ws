import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression,PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def load_file(file_path):
    try:
        with open(file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None




def generate_launch_description():
    declared_args = [
        launch.actions.DeclareLaunchArgument(
            name='robot',
            default_value='dk1',
            description='The robot to run. Can be: dk1, nova, beta, unknown.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='is_right',
            default_value='true',
            description='specify the righthandedness of the glove'
        ),
    ]
    robot_type = LaunchConfiguration("robot")
    is_right = LaunchConfiguration("is_right")

    handedness = PythonExpression(['"right" if "', is_right, '" == "true" else "left"'])
    short_handedness = PythonExpression(['"r" if "', is_right, '" == "true" else "l"'])
    # gloves_ns = PythonExpression(['"/senseglove/" + str(int(int(',nr_of_gloves,')/2)) + "/', short_handedness,'h/robot_description"'])
    controller_manager_ns = PythonExpression(['"/senseglove/', short_handedness,'h/controller_manager"'])
    gloves_ns = PythonExpression(['"/senseglove/', short_handedness,'h/"'])

    robot_description_path = PythonExpression(['str("urdf/',robot_type,'_', handedness, '.xacro")'])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("senseglove_description"),robot_description_path]),
            " ",
            "robot_type:=",
            robot_type,
            " ",
            "is_right:=",
            is_right,
        ])

    robot_description = {"robot_description": robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
                [FindPackageShare("senseglove_hardware_interface"), "config", robot_type, "controllers.yaml"])


    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output="both",
            parameters=[robot_description, {"ignore_timestamp":True, "frame_prefix": gloves_ns}],
            namespace=gloves_ns,
        ),
        launch_ros.actions.Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", controller_manager_ns],
        ),
        launch_ros.actions.Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", controller_manager_ns],

        ),
        launch_ros.actions.Node(
            package='senseglove_hardware_interface',
            executable='senseglove_hardware_interface_node',
            parameters=[robot_description, initial_joint_controllers],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
            namespace=gloves_ns,
        ),
        
        # launch.actions.LogInfo(msg=nr_of_gloves),
        # launch.actions.LogInfo(msg=robot_description_content)
    ]+ declared_args)
    return ld


if __name__ == '__main__':
    generate_launch_description()
