cmake_minimum_required(VERSION 3.8)
project(mirs_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraState.msg"
  "msg/Error.msg"
  "msg/JointState.msg"
  "msg/MotorSensorFeedback.msg"
  "msg/MotorState.msg"
  "msg/RobotState.msg"
  "msg/SystemState.msg"
  "msg/Task.msg"
  "msg/TaskExecutorState.msg"
  "msg/TaskExtractorState.msg"
  "msg/TaskRecorderState.msg"
  "msg/Theta.msg"
  "msg/Trajectory.msg"
  "msg/VoiceState.msg"
  "msg/GuiState.msg"

  DEPENDENCIES geometry_msgs std_msgs
  
 )
 
install(DIRECTORY devices hardware
  DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}/
)


#"srv/ExecTask.srv"
ament_export_dependencies(rosidl_default_runtime)

ament_package()
