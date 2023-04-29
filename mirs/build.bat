:: colcon build --packages-select mirs_interfaces
:: colcon build --packages-select mirs_system

if %1 == 1 (colcon build --packages-select mirs_interfaces) else colcon build --packages-select mirs_system