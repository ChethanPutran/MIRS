IF %1 == 1 (
    colcon build --packages-select mirs_system 
)
ELSE (
    IF %1 == 2 (
        colcon build --packages-select mirs_interfaces
    ) 
    ELSE (
        IF %1 == 3 (
            colcon build --packages-select mirs_controller
        ) 
        ELSE
        (
            echo Not found.
        )
    )

)
    

call ./install/local_setup.bat
:: ros2 launch mirs_system robot.launch.py