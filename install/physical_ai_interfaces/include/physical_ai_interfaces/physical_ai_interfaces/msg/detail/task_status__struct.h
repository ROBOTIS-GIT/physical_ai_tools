// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:msg/TaskStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/task_status.h"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'READY'.
enum
{
  physical_ai_interfaces__msg__TaskStatus__READY = 0
};

/// Constant 'WARMING_UP'.
enum
{
  physical_ai_interfaces__msg__TaskStatus__WARMING_UP = 1
};

/// Constant 'RESETTING'.
enum
{
  physical_ai_interfaces__msg__TaskStatus__RESETTING = 2
};

/// Constant 'RECORDING'.
enum
{
  physical_ai_interfaces__msg__TaskStatus__RECORDING = 3
};

/// Constant 'SAVING'.
enum
{
  physical_ai_interfaces__msg__TaskStatus__SAVING = 4
};

/// Constant 'STOPPED'.
enum
{
  physical_ai_interfaces__msg__TaskStatus__STOPPED = 5
};

/// Constant 'INFERENCING'.
enum
{
  physical_ai_interfaces__msg__TaskStatus__INFERENCING = 6
};

// Include directives for member types
// Member 'task_info'
#include "physical_ai_interfaces/msg/detail/task_info__struct.h"
// Member 'robot_type'
// Member 'current_task_instruction'
// Member 'error'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TaskStatus in the package physical_ai_interfaces.
/**
  * Constants
  *
  * phase
 */
typedef struct physical_ai_interfaces__msg__TaskStatus
{
  physical_ai_interfaces__msg__TaskInfo task_info;
  rosidl_runtime_c__String robot_type;
  /// (0: READY, 1: WARMING_UP, 2: RESETTING 3: RECORDING, 4: SAVING, 5: STOPPED, 6: INFERENCING)
  uint8_t phase;
  uint16_t total_time;
  uint16_t proceed_time;
  uint16_t current_episode_number;
  uint16_t current_scenario_number;
  rosidl_runtime_c__String current_task_instruction;
  float encoding_progress;
  float used_storage_size;
  float total_storage_size;
  float used_cpu;
  float used_ram_size;
  float total_ram_size;
  rosidl_runtime_c__String error;
} physical_ai_interfaces__msg__TaskStatus;

// Struct for a sequence of physical_ai_interfaces__msg__TaskStatus.
typedef struct physical_ai_interfaces__msg__TaskStatus__Sequence
{
  physical_ai_interfaces__msg__TaskStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__msg__TaskStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TASK_STATUS__STRUCT_H_
