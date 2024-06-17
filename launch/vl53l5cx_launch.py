# <arg name="resolution" default="4"/>
# <arg name="mode" default="XYZ"/>

import os
import pathlib
from launch import LaunchDescription, substitutions, actions
import launch_ros

parameters_file_name = "default.yaml"

def generate_launch_description():

    parameters_file_path = str(pathlib.Path(__file__).parents[1])
    parameters_file_path += '/config/' + parameters_file_name

    logger = substitutions.LaunchConfiguration("log_level")
    return LaunchDescription([
        actions.DeclareLaunchArgument("log_level", default_value=["debug"], description="Logging level"),
        launch_ros.actions.Node(
            package='vl53l5cx',
            namespace='vl53l5cx',
            executable='vl53l5cx_node',
            name='laser_sensor',
            parameters=[
                parameters_file_path
            ],
            arguments=['--ros-args', '--log-level', logger]
        )
    ])