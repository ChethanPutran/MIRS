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
  "msg/RobotState.msg"
  "msg/Task.msg"
  "msg/SystemState.msg"
  "msg/TaskExecutorState.msg"
  "msg/TaskExtractorState.msg"
  "msg/TaskRecorderState.msg"
  "msg/Tasks.msg"
  "msg/VoiceState.msg"
  DEPENDENCIES geometry_msgs std_msgs
  
 )


#install(DIRECTORY devices
#  DESTINATION lib/${PROJECT_NAME}
#)


#"srv/ExecTask.srv"
#ament_export_dependencies(rosidl_default_runtime)

ament_package()
