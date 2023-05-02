if [["$1" == "1"]] 
then  
colcon build --packages-select mirs_system --symlink-install
else 
colcon build --packages-select mirs_interfaces --symlink-install
fi
source ./install/setup.bash



