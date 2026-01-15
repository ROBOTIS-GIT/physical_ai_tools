cmake_minimum_required(VERSION 3.16)
project(rosbag_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SendCommand.srv"
  "msg/ImageMetadata.msg"
  "msg/EncodingStatus.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Build service_bag_recorder with image compression
add_executable(service_bag_recorder
  src/service_bag_recorder.cpp
  src/image_compressor.cpp
  src/topic_health_checker.cpp
)

target_include_directories(service_bag_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(service_bag_recorder PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  service_bag_recorder
  "rclcpp"
  "rosbag2_cpp"
  "sensor_msgs"
  "std_msgs"
  "cv_bridge"
  "image_transport"
  "ament_index_cpp"
)

# Link against the generated interfaces using the modern approach
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(service_bag_recorder
  "${cpp_typesupport_target}"
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

# Build image_bag_recorder with image compression
add_executable(image_bag_recorder
  src/image_bag_recorder.cpp
  src/image_compressor.cpp
)

target_include_directories(image_bag_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(image_bag_recorder PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  image_bag_recorder
  "rclcpp"
  "rosbag2_cpp"
  "sensor_msgs"
  "std_msgs"
  "cv_bridge"
  "image_transport"
)

rosidl_get_typesupport_target(cpp_typesupport_target_image
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(image_bag_recorder
  "${cpp_typesupport_target_image}"
  ${OpenCV_LIBRARIES}
)

add_executable(synced_image_bag_recorder
  src/synced_image_bag_recorder.cpp
  src/synced_image_compressor.cpp
)

target_include_directories(synced_image_bag_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(synced_image_bag_recorder PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  synced_image_bag_recorder
  "rclcpp"
  "rosbag2_cpp"
  "sensor_msgs"
  "std_msgs"
  "cv_bridge"
  "image_transport"
)

rosidl_get_typesupport_target(cpp_typesupport_target_synced
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(synced_image_bag_recorder
  "${cpp_typesupport_target_synced}"
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  service_bag_recorder
  image_bag_recorder
  synced_image_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
