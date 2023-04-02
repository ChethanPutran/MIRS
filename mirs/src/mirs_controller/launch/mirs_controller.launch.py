from launch import LaunchDescription
from launch_ros.actions import Node


# All launch file should have following function
def generate_launch_description():
    robot = Node(
        package="mirs_controller",
        executable="mirs_robot",
        name="mirs_robot"
                        )
    joint_state_publisher = Node(
        package="mirs_controller",
        executable="mirs_joint_state_publisher",
        name="mirs_joint_state_publisher",)
    
    trajectory_follower = Node(
        package="mirs_controller",
        executable="mirs_trajectory_follower",
        name="mirs_trajectory_follower",
        parameters=[{"robot":robot}]
                        )
    
    return LaunchDescription([robot,joint_state_publisher,trajectory_follower])