// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:msg/TaskInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/task_info.h"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'task_name'
// Member 'task_type'
// Member 'user_id'
// Member 'task_instruction'
// Member 'policy_path'
// Member 'tags'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TaskInfo in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__msg__TaskInfo
{
  rosidl_runtime_c__String task_name;
  rosidl_runtime_c__String task_type;
  rosidl_runtime_c__String user_id;
  rosidl_runtime_c__String__Sequence task_instruction;
  rosidl_runtime_c__String policy_path;
  uint8_t fps;
  rosidl_runtime_c__String__Sequence tags;
  uint16_t warmup_time_s;
  uint16_t episode_time_s;
  uint16_t reset_time_s;
  uint16_t num_episodes;
  bool push_to_hub;
  bool private_mode;
  bool use_optimized_save_mode;
  bool record_inference_mode;
} physical_ai_interfaces__msg__TaskInfo;

// Struct for a sequence of physical_ai_interfaces__msg__TaskInfo.
typedef struct physical_ai_interfaces__msg__TaskInfo__Sequence
{
  physical_ai_interfaces__msg__TaskInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__msg__TaskInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_INFO__STRUCT_H_
