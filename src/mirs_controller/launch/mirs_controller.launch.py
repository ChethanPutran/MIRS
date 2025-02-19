import os,xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import xacro

# All launch file should have following function
def generate_launch_description():
    
    pkg_name = "mirs_controller"
    file_path = "resource\\urdf\\mirs_description.urdf"

    # # Process xacro file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_path)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot = Node(
        package="mirs_controller",
        executable="mirs_robot",
        name="mirs_robot"
    )

    # Configuring the node
    node_robot_state_publisher = Node(package="robot_state_publisher",
                                      executable="robot_state_publisher", output="screen",
                                      parameters=[{'robot_description': robot_description_raw}])
    
    # node_joint_state_publisher = Node(package="joint_state_publisher_gui",
    #                                   executable="joint_state_publisher_gui", output="screen")
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    # )
    
    
    return LaunchDescription([robot,node_robot_state_publisher])