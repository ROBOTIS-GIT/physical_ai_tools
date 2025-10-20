// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from physical_ai_interfaces:msg/TrainingStatus.idl
// generated code does not contain a copyright notice

#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "physical_ai_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "physical_ai_interfaces/msg/detail/training_status__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace physical_ai_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_physical_ai_interfaces
cdr_serialize(
  const physical_ai_interfaces::msg::TrainingStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_physical_ai_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  physical_ai_interfaces::msg::TrainingStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_physical_ai_interfaces
get_serialized_size(
  const physical_ai_interfaces::msg::TrainingStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_physical_ai_interfaces
max_serialized_size_TrainingStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_physical_ai_interfaces
cdr_serialize_key(
  const physical_ai_interfaces::msg::TrainingStatus & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_physical_ai_interfaces
get_serialized_size_key(
  const physical_ai_interfaces::msg::TrainingStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_physical_ai_interfaces
max_serialized_size_key_TrainingStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace physical_ai_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_physical_ai_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, physical_ai_interfaces, msg, TrainingStatus)();

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
