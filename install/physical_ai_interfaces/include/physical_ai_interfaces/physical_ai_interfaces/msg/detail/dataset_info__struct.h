// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:msg/DatasetInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/dataset_info.h"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'codebase_version'
// Member 'robot_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DatasetInfo in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__msg__DatasetInfo
{
  rosidl_runtime_c__String codebase_version;
  rosidl_runtime_c__String robot_type;
  uint16_t total_episodes;
  uint16_t total_tasks;
  uint8_t fps;
} physical_ai_interfaces__msg__DatasetInfo;

// Struct for a sequence of physical_ai_interfaces__msg__DatasetInfo.
typedef struct physical_ai_interfaces__msg__DatasetInfo__Sequence
{
  physical_ai_interfaces__msg__DatasetInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__msg__DatasetInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__DATASET_INFO__STRUCT_H_
