// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from physical_ai_interfaces:msg/TrainingStatus.idl
// generated code does not contain a copyright notice
#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "physical_ai_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "physical_ai_interfaces/msg/detail/training_status__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_physical_ai_interfaces
bool cdr_serialize_physical_ai_interfaces__msg__TrainingStatus(
  const physical_ai_interfaces__msg__TrainingStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_physical_ai_interfaces
bool cdr_deserialize_physical_ai_interfaces__msg__TrainingStatus(
  eprosima::fastcdr::Cdr &,
  physical_ai_interfaces__msg__TrainingStatus * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_physical_ai_interfaces
size_t get_serialized_size_physical_ai_interfaces__msg__TrainingStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_physical_ai_interfaces
size_t max_serialized_size_physical_ai_interfaces__msg__TrainingStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_physical_ai_interfaces
bool cdr_serialize_key_physical_ai_interfaces__msg__TrainingStatus(
  const physical_ai_interfaces__msg__TrainingStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_physical_ai_interfaces
size_t get_serialized_size_key_physical_ai_interfaces__msg__TrainingStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_physical_ai_interfaces
size_t max_serialized_size_key_physical_ai_interfaces__msg__TrainingStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_physical_ai_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, physical_ai_interfaces, msg, TrainingStatus)();

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
