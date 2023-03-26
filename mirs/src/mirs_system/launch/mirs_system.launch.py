#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

# All launch file should have following function
def generate_launch_description():
    mirs_controller_pkg = get_package_share_directory('mirs_controller')
    

    #Importing launch file from different package
    robot = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(mirs_controller_pkg,'launch','/mirs_controller.launch.py')
    ]))


    recorder = Node(
        package="mirs_system",
        executable="task_recorder",
        name="mirs_task_recorder"
        )
    

    task_extractor = Node(
        package="mirs_system",
        executable="task_extractor",
        name="mirs_task_extractor"
        )
    
    task_executor = Node(
        package="mirs_system",
        executable="task_executor",
        name="mirs_task_executor"
        )
    

    return LaunchDescription([
        robot,recorder,task_extractor,task_executor
    ])
    
    