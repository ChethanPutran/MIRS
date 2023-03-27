import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
#from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
   pkg_name = "mirs_robot_description"
   file_path = "urdf/robot.xacro"

   # Process xacro file
   xacro_file = os.path.join(get_package_share_directory(pkg_name),file_path)

   # # Setting env variable for gazebo
   # pkg_share_path = os.pathsep + get_package_share_directory(pkg_name)
   # if 'GAZEBO_MODEL_PATH' in os.environ:
   #    os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
   # else:
   #    os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path

   robot_description_raw = xacro.process_file(xacro_file).toxml()

   gazebo = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),'launch'),'/gazebo.launch.py']),
         )

   # Configuring the node
   node_robot_state_publisher = Node(package="robot_state_publisher",
                                 executable="robot_state_publisher",
                                 output="screen",
                                 parameters=[{'robot_description':robot_description_raw,
                                                'use_sim_time':True
                                 }])
   
   # Joint state publisher
   # node_joint_state_publisher = Node(package="joint_state_publisher_gui",
   #                               executable="joint_state_publisher_gui",
   #                               output="screen")
   
   
   
   spawn_entity = Node(package='gazebo_ros',
                     executable="spawn_entity.py",
                     arguments=['-topic','/robot_description','-entity','mirs_robot'],
                     output="screen")
   
   # Run the node
   return LaunchDescription([gazebo,
                           node_robot_state_publisher,#node_joint_state_publisher,
                           spawn_entity])
