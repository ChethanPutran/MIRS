from launch import LaunchDescription
from launch_ros.actions import Node


# All launch file should have following function
def generate_launch_description():
    recorder = Node(
        package="mirs_controller",
        executable="recorder",
        name="mirs_recorder"
                        )
    
    return LaunchDescription([recorder])