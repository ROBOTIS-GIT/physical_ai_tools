cmake_minimum_required(VERSION 3.16)
project(rosbag_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SendCommand.srv"
)

add_executable(service_bag_recorder src/service_bag_recorder.cpp)

target_include_directories(service_bag_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(service_bag_recorder PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  service_bag_recorder
  "rclcpp"
  "rosbag2_cpp"
)

# Link against the generated interfaces using the modern approach
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(service_bag_recorder "${cpp_typesupport_target}")

install(TARGETS service_bag_recorder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
