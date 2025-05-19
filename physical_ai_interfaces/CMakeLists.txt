cmake_minimum_required(VERSION 3.16)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()
project(physical_ai_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RecordingStatus.msg"
  "srv/SendRecordingCommand.srv"
)

ament_package()