@echo off 
if %1%==1 (colcon build --packages-select mirs_system --symlink-install)
if %1%==2 ( colcon build --packages-select mirs_interfaces --symlink-install) 
if %1%==3 ( colcon build --packages-select mirs_description --symlink-install) 
if %1%==4 ( Colcon build --packages-select mirs_controller --symlink-install) 
    
call ./install/local_setup.bat
:: ros2 launch mirs_description robot.launch.py