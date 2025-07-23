cmake_minimum_required(VERSION 3.16)
project(physical_ai_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskInfo.msg"
  "msg/TaskStatus.msg"
  "msg/TrainingInfo.msg"
  "msg/TrainingStatus.msg"
  "srv/GetDatasetList.srv"
  "srv/GetHFUser.srv"
  "srv/GetImageTopicList.srv"
  "srv/GetModelWeightList.srv"
  "srv/GetPolicyList.srv"
  "srv/GetRobotTypeList.srv"
  "srv/GetSavedPolicyList.srv"
  "srv/GetUserList.srv"
  "srv/SendCommand.srv"
  "srv/SendTrainingCommand.srv"
  "srv/SetHFUser.srv"
  "srv/SetRobotType.srv"
)

install(
  DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
