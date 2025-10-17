// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from physical_ai_interfaces:msg/HFOperationStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "physical_ai_interfaces/msg/hf_operation_status.h"


#ifndef PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__STRUCT_H_
#define PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'operation'
// Member 'status'
// Member 'local_path'
// Member 'repo_id'
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/HFOperationStatus in the package physical_ai_interfaces.
typedef struct physical_ai_interfaces__msg__HFOperationStatus
{
  /// upload, download
  rosidl_runtime_c__String operation;
  /// Idle, Uploading, Downloading, Success, Failed
  rosidl_runtime_c__String status;
  /// local path of the dataset or model
  rosidl_runtime_c__String local_path;
  /// repo id of the dataset or model
  rosidl_runtime_c__String repo_id;
  /// message of the operation
  rosidl_runtime_c__String message;
  /// current progress
  uint16_t progress_current;
  /// total progress
  uint16_t progress_total;
  /// percentage of the progress
  float progress_percentage;
} physical_ai_interfaces__msg__HFOperationStatus;

// Struct for a sequence of physical_ai_interfaces__msg__HFOperationStatus.
typedef struct physical_ai_interfaces__msg__HFOperationStatus__Sequence
{
  physical_ai_interfaces__msg__HFOperationStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} physical_ai_interfaces__msg__HFOperationStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PHYSICAL_AI_INTERFACES__MSG__DETAIL__HF_OPERATION_STATUS__STRUCT_H_
