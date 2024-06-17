# <arg name="resolution" default="4"/>
# <arg name="mode" default="XYZ"/>

import os
import pathlib
from launch import LaunchDescription, substitutions, actions
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('vl53l5cx'),
        'config',
        'default.yaml'
    )

    logger = substitutions.LaunchConfiguration("log_level")
    return LaunchDescription([
        actions.DeclareLaunchArgument("log_level", default_value=["debug"], description="Logging level"),
        launch_ros.actions.Node(
            package='vl53l5cx',
            namespace='tof_sensor',
            executable='vl53l5cx_node',
            name='vl53l5cx_node',
            emulate_tty=True,
            parameters=[
                config
            ],
            arguments=['--ros-args', '--log-level', logger]
        )
    ])