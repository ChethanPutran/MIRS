colcon build --packages-select mirs_controller
colcon test --packages-select mirs_controller --pytest-args -k test_kinematics
colcon test-result --all --verbose