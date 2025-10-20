// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:msg/TrainingStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/training_status.h"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'training_info'
#include "physical_ai_interfaces/msg/detail/training_info__struct.h"
// Member 'error'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TrainingStatus in the package physical_ai_interfaces.
/**
  * Constants
 */
typedef struct physical_ai_interfaces__msg__TrainingStatus
{
  physical_ai_interfaces__msg__TrainingInfo training_info;
  uint32_t current_step;
  float current_loss;
  bool is_training;
  rosidl_runtime_c__String error;
} physical_ai_interfaces__msg__TrainingStatus;

// Struct for a sequence of physical_ai_interfaces__msg__TrainingStatus.
typedef struct physical_ai_interfaces__msg__TrainingStatus__Sequence
{
  physical_ai_interfaces__msg__TrainingStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__msg__TrainingStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__TRAINING_STATUS__STRUCT_H_
