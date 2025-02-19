### Sterio Camera with Android

###### Sensor

    * https://developer.android.com/guide/topics/sensors/sensors_position

###### Depth-Sensing

    * https://developers.google.com/ar/develop/java/depth/developer-guide

###### Multi-Camera

    * https://developer.android.com/training/camera2/multi-camera

###### Hand identification

    * https://techvidvan.com/tutorials/hand-gesture-recognition-tensorflow-opencv/

https://www.mathworks.com/help/robotics/ref/jointspacemotionmodel.html

https://www.mathworks.com/help/robotics/ug/perform-safe-trajectory-tracking.html

https://www.mathworks.com/help/robotics/examples.html?category=trajectory-generation&s_tid=CRUX_topnav

https://www.mathworks.com/help/robotics/ref/inversedynamics.html

https://www.youtube.com/watch?v=Fd7wjZDoh7g

https://www.mathworks.com/help/robotics/ref/jointspacemotionmodel.html

https://www.mathworks.com/help/robotics/ug/simulate-joint-space-trajectory-tracking.html


https://articulatedrobotics.xyz/ready-for-ros-8-gazebo/

### Ultralytics YOLOv8
https://www.freecodecamp.org/news/how-to-detect-objects-in-images-using-yolov8/



## Interfacing MPU-6050 to Raspberry pi
https://circuitdigest.com/microcontroller-projects/mpu6050-gyro-sensor-interfacing-with-raspberry-pi



# Robot Maker

https://www.roboticslibrary.org/tutorials/create-a-robot-model/

# Instructions

# App Usage
####  Run bat file
>> activate.bat

#### Build the App
>>> build.bat (pakage no)

### Run the package using run command
>>> ros2 run package_name app_name
Eg.
>>> ros2 run mirs_controller controller

### Using launch file to run the package
>>> ros2 launch package_name package_name.launch.py  (using lanch)
Eg.
>>> ros2 launch mirs_description robot.launch.py


### Useful commands

> ros2 node list
> ros2 topic list
> ros2 topic info topic_name
> ros2 interface show msg_type
> ros2 topic echo topic_name
> rqt_graph
> ros2 run tf2_tools view_frames.py

### Using Launch files to run node/s

> Create a launch folder inside your package folder
> Add package_name.launch.py file
> Link the launch file in setup.py file
> Run: ros2 launch package_name package_name.launch.py



#### Add C:\opt\ros\foxy\x64\tools\vcpkg\installed\x64-windows' in the environment variable CMAKE_PREFIX_PATH doesn't exist


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

#
### Motion Planning

> Using Moveit
> Install Moveit
> sudo apt install ros-humble-moveit
> Examples : ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_moveit_config_demo_empty.rviz


### Connecting to remote machine / raspberry pi
=======
### 6D pose Estimation Models

> https://github.com/liuyuan-pal/Gen6D > https://github.com/zubair-irshad/CenterSnap > https://analyticsindiamag.com/guide-to-6d-object-pose-estimation-using-posecnn/

#### Block diagrams and flowchart creator

> Microsoft Visio
> Textografo



### Raspberry Pi installations
#### About os flash and user creation
https://www.raspberrypi.com/news/raspberry-pi-bullseye-update-april-2022/


#### Depth sensing camera
https://www.waveshare.com/cm4-dual-camera-base.html
https://www.waveshare.com/wiki/CM4-DUAL-CAMERA-BASE


#### About FLash
https://www.pcguide.com/raspberry-pi/how-to/set-up-compute-module-4-cm4/
https://linuxhint.com/use-laptop-as-monitor-raspberry-pi/

#### Enabling USB
dtoverlay=dwc2,dr_mode=host

#### Enabling HBMI to VGI
hdmi_force_hotplug=1
hdmi_drive=2

#### Raspberry connection to HDMI display
https://linuxhint.com/connect-raspberry-pi-monitor/#:~:text=Simply%20connect%20the%20HDMI%20cable,of%20HDMI%20to%20VGA%20adapter.


#### Connecting to laptop
https://linuxhint.com/connect-raspberry-pi-laptop/
https://beebom.com/how-use-windows-laptop-as-monitor-raspberry-pi/


#### Connecting to wifi
> systemctl start NetworkManager
> systemctl enable NetworkManager
> nmcli device wifi list
> sudo nmcli --ask dev wifi connect <SSID>

- Download Angry IP Scanner:
http://angryip.org/download/#windows

- Download Putty:
http://www.putty.org/

- Download VNC-Viewer:
https://www.realvnc.com/download/viewer/


#### Using depth sensing cameras

#### Test camera in Bullseye image
> sudo libcamera-vid -t 0 -cs 0
> sudo libcamera-vid -t 0 -cs 1

#### Check if the camera is detected
> libcamera-hello --list-cameras

#### Test camera in Bullseye image
> sudo libcamera-vid -t 0 --camera 0
> sudo libcamera-vid -t 0 --camera 1


#### Opening the corresponding camera
> libcamera-hello --camera 1
> libcamera-hello --camera 0

#### Taking Pictures
> libcamera-jpeg -o test-left.jpg --camera 0
> libcamera-jpeg -o test-right.jpg --camera 1

#### Getting image to host
> scp cheth@raspberrypi.local:test-left.jpg .
> libcamera-jpeg -o test-right.jpg --camera 1 --width=640 --height=640
> sudo libcamera-vid -t 5000 -cs 0 -o t-l.h264 --width 640 --height 640


#### Connecting to wifi through command line
> sudo ifconfig wlan0 up
> nmcli dev wifi list
or
> sudo iw wlan0 scan | grep SSID
> sudo nmcli --ask dev wifi connect <SSID>