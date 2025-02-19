# Instructions to run the software

## Activate the ROS environment using batch file
>> activate.bat

## Build the package 
>>> build.bat (pakage no)

## Run the package using either of the 2 ways
### 1. Using run command
>>> ros2 run package_name app_name
Eg.
>>> ros2 run mirs_controller controller
### 2. Using launch file to run the package
>>> ros2 launch package_name package_name.launch.py  (using lanch)
Eg.
>>> ros2 launch mirs_description robot.launch.py

## Useful commands to debug
> ros2 node list
> ros2 topic list
> ros2 topic info topic_name
> ros2 interface show msg_type
> ros2 topic echo topic_name
> rqt_graph
> ros2 run tf2_tools view_frames.py

