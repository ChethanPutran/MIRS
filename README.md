# Instructions

## 1. Install ROS2-Humble (use following link for reference)

> https://docs.ros.org/en/humble/InstallationUbuntu-Install-Debians.html

### Prerequisits (Type folllowing in terminal)

-   1. sudo apt install python3-colcon-common-extensions
-   2. To avoid repetative sourcing, Open terminal
        - Open .bashrc file and put the following lines (i.e type command :gedit ~/.bashrc )
        1. source /opt/ros/humble/setup.bash
        2. source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

## 2. Create a ROS2 Workspace for Python

-   1. Create a project folder (e.g- mkdir ros_tutes && cd ros_tutes)
-   2. Create a src folder inside project folder (e.g : mkdir src)
-   3. Source the ROS2 workspace (Use command : source ~/ros_tutes/install/setup.bash )
-   4. Open .bashrc file (i.e type command :gedit ~/.bashrc )
    5. Put the line : source ~/ros_tutes/install/setup.bash

### Installing dependencies (if needed)

> cd ~/ros2_ws/src
> source ~/ros_tutes/install/setup.bash
> sudo apt-get update
> rosdep install --from-paths ~/ros_tutes/src --ignore-src -r -y

## 3. Create a ROS2 Pakage

-   1. Navigate to src folder & type command: ros2 pkg create mirs_controller --build-type ament_python --dependencies rclpy

## 4. Build project

-   1. Move to base folder of your workspace
-   2. Type command :
        > colcon build (build everything) or colcon build --packages-select my_package (for specific package build)

-   > sudo colcon build --symlink-install

## 5. Run nodes

-   > ros2 run mirs_controller controller

### Ros Transforms

1. Static Transforms
2. Dynamic Transforms

#### 1. Static Transforms

-   Static Broadcasting syntax

    > ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll parent_name child_name

-   Visualizing with rviz
    > ros2 run rviz2 rviz2
    > Go to add>tf>ok
    > Change reference frame to world
    > Enable show labels to view frame names

#### 2. Dynamic Transforms

> Requires joint state publisher & xacro
> Install using sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui

-   Step 1. Run Robot State Publisher
    > ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/test/test.urdf.xacro)"
    > ros2 run joint_state_publisher_gui joint_state_publisher_gui
    > ros2 rviz rviz
    > Run following to view
    > ros2 tf2_tools view_frames.py

### Robot Simulation

Gazebo - a robot environment simulator

-   Install gazebo

    > sudo apt install ros-humble-gazebo-ros-pkgs

-   Running Gazebo

    > ros2 launch gazebo_ros gazebo.launch.py

-   Spawning scripts
    > ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_bot

#### URDF FILES

#### Useful commands

> ros2 node list
> ros2 topic list
> ros2 topic info topic_name
> ros2 interface show msg_type
> ros2 topic echo topic_name
> rqt_graph
> ros2 run tf2_tools view_frames.py
> 

### Using Launch files to run node/s

> Create a launch folder inside your package folder
> Add package_name.launch.py file
> Link the launch file in setup.py file
> Run: ros2 launch package_name package_name.launch.py

### Motion Planning

> Using Moveit
> Install Moveit
> sudo apt install ros-humble-moveit
> Examples : ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_moveit_config_demo_empty.rviz
