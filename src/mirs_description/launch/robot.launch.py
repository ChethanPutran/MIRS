import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import xacro


def generate_launch_description():
    pkg_name = "mirs_description"
    #file_path = "src\\description\\mirs_description.urdf"
    file_path = "urdf\\robot.xacro"

    # # Process xacro file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_path)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configuring the node
    node_robot_state_publisher = Node(package="robot_state_publisher",
                                      executable="robot_state_publisher", output="screen",
                                      parameters=[{'robot_description': robot_description_raw}])
    # node_joint_state_publisher = Node(package="joint_state_publisher_gui",
    #                                   executable="joint_state_publisher_gui", output="screen")
    # node_mirs_controller = Node(package="mirs_controller",
    #                             executable="mirs_robot",
    #                             output="screen",
    #                             parameters=[{"robot_description":xacro_file}])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    # Run the node
    return LaunchDescription([node_robot_state_publisher,rviz_node])
    #return LaunchDescription([node_robot_state_publisher, node_joint_state_publisher])
    #return LaunchDescription([node_robot_state_publisher,node_mirs_controller])
    # return LaunchDescription([node_robot_state_publisher, node_joint_state_publisher,node_mirs_controller])
