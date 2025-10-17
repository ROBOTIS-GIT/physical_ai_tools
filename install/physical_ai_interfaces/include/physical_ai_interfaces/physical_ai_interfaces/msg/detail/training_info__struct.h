// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:msg/TrainingInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/training_info.h"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'dataset'
// Member 'policy_type'
// Member 'output_folder_name'
// Member 'policy_device'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TrainingInfo in the package physical_ai_interfaces.
/**
  * Must be given
 */
typedef struct physical_ai_interfaces__msg__TrainingInfo
{
  /// ROBOTIS/ffw_bg2_example
  rosidl_runtime_c__String dataset;
  rosidl_runtime_c__String policy_type;
  rosidl_runtime_c__String output_folder_name;
  rosidl_runtime_c__String policy_device;
  /// Options
  uint32_t seed;
  uint8_t num_workers;
  uint16_t batch_size;
  uint32_t steps;
  uint32_t eval_freq;
  uint32_t log_freq;
  uint32_t save_freq;
} physical_ai_interfaces__msg__TrainingInfo;

// Struct for a sequence of physical_ai_interfaces__msg__TrainingInfo.
typedef struct physical_ai_interfaces__msg__TrainingInfo__Sequence
{
  physical_ai_interfaces__msg__TrainingInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__msg__TrainingInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_INFO__STRUCT_H_
